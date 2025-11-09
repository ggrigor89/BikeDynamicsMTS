# Bicycle Stabilization Dynamics for SUMO (Python 2)

This repository provides a Python **2.x** script that integrates a bicycle
stabilization dynamics model into a SUMO/TraCI simulation. The model modulates
each bicycle's lateral position as a function of stabilization control effort,
producing an **operational width that decreases with speed** and enabling more
realistic studies of **overtaking** and **platoon** behavior.

## Reference
> G. Grigoropoulos, H. Kaths, F. Busch (2019).  
> *Introducing the Effect of Bicyclist Stabilization Control in Microscopic Traffic Simulation*.  
> 2019 IEEE ITSC, pp. 1373–1378. doi:10.1109/ITSC.2019.8916880

## The script will:
construct a minimal bicycle routes file (spontroutes.rou.xml),
start SUMO with the provided config, and run the dynamics (see #### begin main loop (simulation) in the code),
writing per-timestep results to results.csv.

## Notes
The code is intentionally kept at Python 2 to preserve compatibility, as this has been a legacy project
All machine(local)-specific paths have been removed; configure on your own via environment vars.
For scientific background and model details, please see the paper above. Additionally function bikaccdecmod implements the aceleration model 
from the following paper:
Twaddle, Heather, and Georgios Grigoropoulos. Modeling the Speed, Acceleration, and Deceleration of Bicyclists for Microscopic Traffic Simulation. No. 16-0198. 2016.
[Modeling the Speed, Acceleration, and Deceleration of Bicyclists for Microscopic Traffic Simulation
](https://doi.org/10.3141/2587-02)

## Requirements
- Python **2.x** Script is from 2017 :-)
- [SUMO](https://www.eclipse.org/sumo/) with TraCI
- Python packages: `traci`, `sumolib`, `numpy`

## Setup
Set environment variables (or rely on defaults):
```bash
# Optional, helps find SUMO tools
export SUMO_HOME=/path/to/sumo

# SUMO configuration to run (defaults to 'spont.sumo.cfg' in CWD)
export SUMO_CFG=/path/to/your/scenario.sumo.cfg

# "1" to use GUI (sumo-gui), otherwise headless sumo, simulation will run without GUI
export SUMO_GUI=1

# Optional CSV with initial trajectory data (x,y);
export BIKE_TRAJ_CSV=/path/to/bicycle_traj.csv
