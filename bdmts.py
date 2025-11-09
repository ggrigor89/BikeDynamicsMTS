# -*- coding: utf-8 -*-
"""
Bicycle stabilization dynamics for microscopic traffic simulation (SUMO/TraCI).

Implements a bicycle dynamics model that models lateral position as a function
of stabilization control effort. The model can be used to study operational
width vs. speed and overtaking/platoon behavior in microscopic traffic simulation SUMO (code implementation) or VISSIM (using the respective function and python COM interface code not provided here.).

Reference:
G. Grigoropoulos, H. Kaths, F. Busch (2019).
"Introducing the Effect of Bicyclist Stabilization Control in Microscopic Traffic Simulation."
2019 IEEE ITSC, pp. 1373–1378. doi:10.1109/ITSC.2019.8916880

Relevant work: 
Kaths, Heather, and Georgios Grigoropoulos. "Integration of an external bicycle model in sumo." SUMO User Conference. 2016.
https://mediatum.ub.tum.de/doc/1310495/document.pdf

Function bikaccdecmod implements the aceleration model from the followingp paper
Twaddle, Heather, and Georgios Grigoropoulos. Modeling the Speed, Acceleration, and Deceleration of Bicyclists for Microscopic Traffic Simulation. No. 16-0198. 2016.


Notes:
- Python 2.x.
- SUMO paths and scenario files must be defined by user.

Environment variables (optional):
- SUMO_HOME : path to SUMO installation (to locate 'tools')
- SUMO_CFG  : path to SUMO configuration file (.sumo.cfg)
- SUMO_GUI  : "1" to start sumo-gui, anything else/omitted starts sumo (CLI)
- BIKE_TRAJ_CSV : optional CSV with initial trajectory data (x,y) used by the script
"""

import os
import sys
import time
import random
import subprocess
import math
import csv

# Third-party libraries
from numpy import asarray
import traci
import sumolib
from sumolib import checkBinary

# -------------------------------------------------------------------------
# Resolve SUMO paths without hard-coded local references
# -------------------------------------------------------------------------
# If SUMO_HOME is set, extend sys.path so we can import TraCI tools when needed.
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    if tools not in sys.path:
        sys.path.append(tools)

# -------------------------------------------------------------------------
# Model functions
# -------------------------------------------------------------------------
def create_demand(demand_dict, truck_share, routes_path='spontroutes.rou.xml'):
    """
    Write a minimal routes file for bicycles to the given path.

    Parameters
    ----------
    demand_dict : dict
        Mapping of route IDs to lists of hourly demand values. (Here kept for
        compatibility with original signature; only used for length check.)
    truck_share : list[float]
        Per-hour truck share (unused in bicycle-only example, kept for parity).
    routes_path : str
        Output path for the routes XML file (relative or absolute).

    Returns
    -------
    int
        1 on success, 0 on failure (e.g., inconsistent list lengths).
    """
    hours = len(truck_share)
    for route in demand_dict:
        if len(demand_dict[route]) != hours:
            print "Demand cannot be created. Wrong length of lists given."
            return 0

    # Minimal bicycle-only routes file consistent with SUMO schema.
    routes = open(routes_path, "w")
    routes.write(
        '<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
        'xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">\n'
        '    <vType id="bicycle" vClass="bicycle" accel="0.5" decel="1.0" '
        'latAlignment="compact" departPos="random_free" sigma="0" length="2" '
        'minGap="2" maxSpeed="5" speedFactor="1.2" speedDev="0.7" color="0,1,0" guiShape="bicycle"/>\n'
        '    <route id="NE" edges="NSE"/>\n'
        '    <flow id="NE_bike" type="bicycle" route="NE" begin="0" end="3600" '
        'arrivalSpeed="current" vehsPerHour="4000" color="0,0,1"/>\n'
        '</routes>\n'
    )
    routes.close()
    return 1


def start_sumo(gui_on, sumo_cfg):
    """
    Build the SUMO command using checkBinary and return it; no OS-specific logic.

    Parameters
    ----------
    gui_on : int
        1 to use 'sumo-gui', 0 to use 'sumo'.
    sumo_cfg : str
        Path to the SUMO configuration file (.sumo.cfg).

    Returns
    -------
    list[str]
        Command suitable for traci.start().
    """
    exe = 'sumo-gui' if gui_on == 1 else 'sumo'
    sumo_binary = checkBinary(exe)
    return [sumo_binary, "-c", sumo_cfg]


def embed_traci_if_needed(sumo_cmd, port=8813):
    """
    Optionally embed TraCI if not already embedded. Uses provided SUMO command.

    Parameters
    ----------
    sumo_cmd : list[str]
        Command list built by `start_sumo`.
    port : int
        TraCI port (default 8813).

    Returns
    -------
    None
    """
    if not traci.isEmbedded():
        # SUMO start; TraCI will connect on default port.
        traci.start(sumo_cmd)


def deltadeviationfunction(v, state):
    """
    Compute the max steering angle deviation (in degrees) as a function of
    current speed v and a qualitative 'state' (e.g., 'dec' or 'acc').

    Parameters
    ----------
    v : float
        Current speed (simulation units consistent with the main loop; original
        model expects m/s and internally scales by 36).
    state : str
        'dec' (decelerating) or other (accelerating/cruising).

    Returns
    -------
    float
        Randomized maximum steering deviation (deg); upper-bounded later.
    """
    if state == 'dec':
        A = 12.77550655
        L = 0.242505744
        c = 0.2
    else:
        A = 13.4574892
        L = 0.190175429
        c = 0.8
    deltadev = (A * math.exp(-L * float(v) * 36)) + c
    # Randomize to recreate unstable motion patterns.
    deltadev = random.uniform(0.001, 1.2) * deltadev
    return deltadev


def effec_pathdev(v, deltadev, right, epsilon, dyt, x, y, lr, action, D):
    """
    Update bicycle position and steering direction given stabilization effort.

    Parameters
    ----------
    v : float
        Current forward speed.
    deltadev : float
        Steering angle deviation (deg).
    right : bool
        Current steering direction flag (True/False toggles sign).
    epsilon : float
        Stabilization-related parameter (deg).
    dyt : float
        Accumulated lateral deviation.
    x, y : float
        Current Cartesian position (simulation coordinates).
    lr : float
        Lateral deviation threshold before switching steering direction.
    action : int
        Steps remaining to keep current steering action before reconsidering.
    D : float
        Current effective steering angle (deg).

    Returns
    -------
    tuple
        Updated (x, y, right, dyt, D, dy, lr, action)
    """
    if deltadev > 10: # Avoid having too large value for deltadev so that bicycles do not move out of control or laterally
        deltadev = 10

    if action == 0:
        action = int((1 / 0.1) / random.randint(1, 3)) - 1
        right = not right
        D = 180 * (math.atan(math.cos(math.pi * epsilon / 180.0) *
                              (math.tan(deltadev * math.pi / 180.0)))) / math.pi
    else:
        action = action - 1

    dx = float(v) * math.cos((D) * math.pi / 180.0)
    dy = float(v) * math.sin((abs(D)) * math.pi / 180.0)

    # Steering direction management and lateral deviation accumulation. Check if a steering direction change is necessary and aggregate lateral deviation changes

    if right == False and ((dyt >= lr) or (y < 290 and y + dy >= 290)):
        right = True
        dyt = dyt - dy
        dy = (-1) * dy
        if D > 0:
            D = (-1) * D
        lr = random.uniform(0.1, 0.4)
        action = int((1 / 0.1) / random.randint(1, 3)) - 1
        D = 180 * (math.atan(math.cos(math.pi * epsilon / 180.0) *
                              (math.tan(deltadev * math.pi / 180.0)))) / math.pi
    elif right == True and dyt <= -lr:
        right = False
        dyt = dyt + dy
        lr = random.uniform(0.1, 0.4)
        action = int((1 / 0.1) / random.randint(1, 3)) - 1
        D = 180 * (math.atan(math.cos(math.pi * epsilon / 180.0) *
                              (math.tan(deltadev * math.pi / 180.0)))) / math.pi
    elif right == False:
        dyt = dyt + dy
    else:
        dyt = dyt - dy
        if D > 0:
            D = (-1) * D
        dy = (-1) * dy

    x = x + dx
    y = y + dy
    return x, y, right, dyt, D, dy, lr, action


def bikaccdecmod(vtlast, vi, vdes):
    """
    Compute an acceleration/deceleration increment and the normalized progress
    parameter theta toward the desired speed.

    Parameters
    ----------
    vtlast : float
        Previous timestep speed.
    vi : float
        Initial speed (used in theta).
    vdes : float
        Desired speed.

    Returns
    -------
    (float, float)
        accdec : speed increment to apply
        theta  
    """
    theta = (abs(vtlast) - abs(vi)) / (vdes - abs(vi))
    if round(vdes, 2) <= 0.4:
        m, n, r, cm, Acc_max = 0.6, 2.4, 1.9, 1.5, 0.572
    elif round(vdes, 2) <= 0.5:
        m, n, r, cm, Acc_max = 0.6, 1.9, 1.9, 1.7, 0.609
    elif round(vdes, 2) <= 0.6:
        m, n, r, cm, Acc_max = 0.4, 2.8, 1.9, 1.2, 0.707
    else:
        m, n, r, cm, Acc_max = 0.5, 2.5, 1.9, 1.4, 0.761
        print theta, Acc_max, vtlast, vi, vdes

    accdec = (r * abs(Acc_max) * math.pow(theta, m) * (1 - math.pow(theta, n)) *
              (1 - math.pow(theta, n)) + (((-1 / (1 + cm)) + (1 / (theta * theta + cm))))) * 0.1 * 0.1
    return accdec, theta


# -------------------------------------------------------------------------
# SUMO / scenario defaults
# -------------------------------------------------------------------------
gui_on_env = os.environ.get('SUMO_GUI', '1')
gui_on = 1 if gui_on_env == '1' else 0

sumo_cfg = os.environ.get('SUMO_CFG', 'spont.sumo.cfg')
sumoCmd = start_sumo(gui_on, sumo_cfg)

# -------------------------------------------------------------------------
# Scenario inputs (sanitized; no local absolute paths)
# -------------------------------------------------------------------------
demand_dict = {'NE': [60], 'NW': [70], 'NS': [450]}
demand_dictK = {'NEK': [290], 'NWK': [140], 'NSK': [650]}  
demand_dictTrP = {'NEK': [0.01], 'NWK': [0.01], 'NSK': [0.07]}
truck_share = [0.0]

create_demand(demand_dict, truck_share)

# Optional bicycle trajectory CSV
CSVin = os.environ.get('BIKE_TRAJ_CSV', '')
Data = []
btx, bty = [], []
if CSVin and os.path.exists(CSVin):
    with open(CSVin, 'rb') as csvfile:
        data = csv.reader(csvfile, delimiter=',')
        for row in data:
            Data.append(row)
    Data = asarray(Data)
    try:
        btx = map(float, list(Data[:, 0]))
        bty = map(float, list(Data[:, 1]))
    except Exception as _:
        pass

# -------------------------------------------------------------------------
# Simulation state containers
# -------------------------------------------------------------------------
BikeList = {}
BDYT = {}
Bright = {}
EpsilonB = {}
Bvi = {}
Bvdes = {}
Blr = {}
EDGEVEHLIST = []
lead = []
Baction = {}
BD = {}

# -------------------------------------------------------------------------
# Start TraCI/SUMO
# -------------------------------------------------------------------------
traci.start(sumoCmd)
import traci.constants as tc

# Colors (RGBA)
red = (255, 0, 0, 0)
green = (0, 255, 0, 0)
blue = (0, 0, 255, 0)
white = (0, 0, 0, 0)

# -------------------------------------------------------------------------
# control variables
# -------------------------------------------------------------------------
k = 0
inst = 0
VIDN = 0

# -------------------------------------------------------------------------
#### begin main loop (simulation) 
fd = csv.writer(open("results.csv", "a"), delimiter=',', quoting=csv.QUOTE_ALL)
while k < 1800 * 10:
    traci.simulationStep()
    print 'ok', k
    print traci.edge.getIDList(), 0

    for edgeid in traci.edge.getIDList():
        print traci.edge.getIDList()
        EDGEVEHLIST = traci.edge.getLastStepVehicleIDs(edgeid)
        for VID in EDGEVEHLIST:
            lead = traci.vehicle.getLeader(VID, 0)
            x, y = traci.vehicle.getPosition(VID)
            if not lead == None:
                xl, yl = traci.vehicle.getPosition(lead[0])
                ldist = lead[1]
            else:
                ldist = 1
            if traci.vehicle.getTypeID(VID) == "bicycle" and traci.vehicle.getLanePosition(VID) > 15 and traci.vehicle.getLanePosition(VID) < 19:
                LIND = traci.vehicle.getLaneIndex(VID)
                traci.vehicle.moveToVTD(VID, '', LIND, 52.7, 105.5, 90)
            if ldist > 0.5 and traci.vehicle.getTypeID(VID) == "bicycle" and traci.vehicle.getLanePosition(VID) > 19 and traci.vehicle.getLanePosition(VID) < traci.lane.getLength("NSE_0") - 20:
                check = False
                ########BICYCLE EXISTS?#################
                for bv_iden in BikeList:
                    if VID == bv_iden:
                        vtlast = BikeList[bv_iden]
                        dyt = BDYT[bv_iden]
                        right = Bright[bv_iden]
                        epsilon = EpsilonB[bv_iden]
                        vi = Bvi[bv_iden]
                        vdes = Bvdes[bv_iden]
                        lr = Blr[bv_iden]
                        action = Baction[bv_iden]
                        D = BD[bv_iden]
                        traci.vehicle.setColor(VID, (0, 0, 100, 0))
                        check = True
                        break
                ########NEW BICYCLE#################
                if check == False:
                    vtlast = traci.vehicle.getSpeed(VID) * 0.1
                    dyt = 0
                    right = True
                    epsilonmin = 0
                    epsilonmax = 28
                    epsilon = random.uniform(epsilonmin, epsilonmax)
                    EpsilonB[VID] = epsilon
                    lr = random.uniform(0.1, 0.4)
                    vdes = random.uniform(3, 8) * 0.1
                    vi = 0

                    Bvdes[VID] = vdes
                    action = 0
                    D = 0
                    traci.vehicle.setColor(VID, (255, 0, 0, 0))

                ###################################
                angle = traci.vehicle.getAngle(VID)
                LIND = traci.vehicle.getLaneIndex(VID)
                print vtlast, vi, vdes
                if vtlast > 0.9:
                    vtlast = vdes
                accdec, theta = bikaccdecmod(vtlast, vi, vdes)
                vb = vtlast + accdec
                accdec = 0
                # vb=0.1*10/3.6
                # vdes=vb
                theta = 1
                dv = vb - vtlast
                state = "acc"

                deltadev = deltadeviationfunction(vb, state)
                x, y, right, dyt, D, dy, lr, action = effec_pathdev(vb, deltadev, right, epsilon, dyt, x, y, lr, action, D)
                traci.vehicle.moveToVTD(VID, '', LIND, x, y, 90 - D)
                BDYT[VID] = dyt
                Bright[VID] = right
                Blr[VID] = lr
                BikeList[VID] = vb
                Bvi[VID] = vi
                Baction[VID] = action
                BD[VID] = D
                LID = traci.vehicle.getLaneID(VID)
                EID = traci.lane.getEdgeID(LID)
                LIND = traci.vehicle.getLaneIndex(VID)
                x, y = traci.vehicle.getPosition(VID)
                res = [traci.simulation.getCurrentTime(), VID, x, y, vb, accdec, theta, vdes, state, D, dy, dyt, right, deltadev, EpsilonB[VID]]
                fd.writerow(res)
    k = k + 1
    traci.simulationStep()
traci.close()
# -------------------------------------------------------------------------
