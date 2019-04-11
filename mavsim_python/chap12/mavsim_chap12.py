"""
mavsim_python
    - Chapter 12 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        4/3/2019 - BGM
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM
import parameters.planner_parameters as PLAN

from chap3.data_viewer import data_viewer
from chap4.wind_simulation import wind_simulation
from chap6.autopilot import autopilot
from chap7.mav_dynamics import mav_dynamics
from chap8.observer import observer
from chap10.path_follower import path_follower
from chap11.path_manager import path_manager
from chap12.world_viewer import world_viewer
from chap12.path_planner import path_planner

# initialize the visualization
VIDEO = False  # True==write video, False==don't write video
world_view = world_viewer()  # initialize the viewer
data_view = data_viewer()  # initialize view of data plots
if VIDEO == True:
    from chap2.video_writer import video_writer
    video = video_writer(video_name="chap12_video.avi",
                         bounding_box=(0, 0, 1000, 1000),
                         output_rate=SIM.ts_video)

# initialize elements of the architecture
wind = wind_simulation(SIM.ts_simulation)
mav = mav_dynamics(SIM.ts_simulation)
ctrl = autopilot(SIM.ts_simulation)
obsv = observer(SIM.ts_simulation)
path_follow = path_follower()
path_manage = path_manager()
path_plan = path_planner()

from message_types.msg_map import msg_map
map = msg_map(PLAN)


# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    #-------observer-------------
    measurements = mav.sensors()  # get sensor measurements
    estimated_state = obsv.update(measurements)  # estimate states from measurements

    # -------path planner - ----
    if path_manage.flag_need_new_waypoints == 1:
        waypoints = path_plan.update(map, estimated_state)

    #-------path manager-------------
    path = path_manage.update(waypoints, PLAN.R_min, estimated_state)

    #-------path follower-------------
    autopilot_commands = path_follow.update(path, estimated_state)

    #-------controller-------------
    delta, commanded_state = ctrl.update(autopilot_commands, estimated_state)

    #-------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update_state(delta, current_wind)  # propagate the MAV dynamics

    #-------update viewer-------------
    world_view.update(map, waypoints, path, mav.true_state)  # plot path and MAV
    data_view.update(mav.true_state, # true states
                     estimated_state, # estimated states
                     commanded_state, # commanded states
                     SIM.ts_simulation)
    if VIDEO == True: video.update(sim_time)

    #-------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO == True: video.close()




