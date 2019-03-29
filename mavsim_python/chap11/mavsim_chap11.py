"""
mavsim_python
    - Chapter 11 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        3/26/2019 - RWB
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
from chap11.waypoint_viewer import waypoint_viewer

# initialize the visualization
VIDEO = False  # True==write video, False==don't write video
waypoint_view = waypoint_viewer()  # initialize the viewer
data_view = data_viewer()  # initialize view of data plots
if VIDEO == True:
    from chap2.video_writer import video_writer
    video = video_writer(video_name="chap11_video.avi",
                         bounding_box=(0, 0, 1000, 1000),
                         output_rate=SIM.ts_video)

# initialize elements of the architecture
wind = wind_simulation(SIM.ts_simulation)
mav = mav_dynamics(SIM.ts_simulation)
ctrl = autopilot(SIM.ts_simulation)
obsv = observer(SIM.ts_simulation)
path_follow = path_follower()
path_manage = path_manager()

# waypoint definition
from message_types.msg_waypoints import msg_waypoints
waypoints = msg_waypoints()
#waypoints.type = 'straight_line'
#waypoints.type = 'fillet'
waypoints.type = 'dubins'
waypoints.num_waypoints = 4
Va = PLAN.Va0
waypoints.ned[:, 0:waypoints.num_waypoints] \
    = np.array([[0, 0, -100],
                [1000, 0, -100],
                [0, 1000, -100],
                [1000, 1000, -100]]).T
waypoints.airspeed[:, 0:waypoints.num_waypoints] \
    = np.array([[Va, Va, Va, Va]])
waypoints.course[:, 0:waypoints.num_waypoints] \
    = np.array([[np.radians(0),
                 np.radians(45),
                 np.radians(45),
                 np.radians(-135)]])

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    #-------observer-------------
    measurements = mav.sensors()  # get sensor measurements
    estimated_state = obsv.update(measurements)  # estimate states from measurements

    #-------path manager-------------
    path = path_manage.update(waypoints, PLAN.R_min, estimated_state)

    #-------path follower-------------
    autopilot_commands = path_follow.update(path, estimated_state)

    #-------controller-------------
    delta, commanded_state = ctrl.update(autopilot_commands, estimated_state)

    #-------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics

    #-------update viewer-------------
    waypoint_view.update(waypoints, path, mav.true_state)  # plot path and MAV
    data_view.update(mav.true_state, # true states
                     estimated_state, # estimated states
                     commanded_state, # commanded states
                     SIM.ts_simulation)
    if VIDEO == True: video.update(sim_time)

    #-------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO == True: video.close()




