"""
mavsimPy
    - Chapter 4 assignment for Beard & McLain, PUP, 2012
    - Update history:
        12/27/2018 - RWB
        1/17/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
import parameters.simulation_parameters as SIM

from chap2.mav_viewer import mav_viewer
from chap2.video_writer import video_writer
from chap3.data_viewer import data_viewer
from chap4.mav_dynamics import mav_dynamics
from chap4.wind_simulation import wind_simulation

from IPython.core.debugger import Pdb

# initialize the visualization
VIDEO = False  # True==write video, False==don't write video
DATA = True
mav_view = mav_viewer()  # initialize the mav viewer
if DATA:
    data_view = data_viewer()  # initialize view of data plots
if VIDEO == True:
    video = video_writer(video_name="chap4_video.avi",
                         bounding_box=(0, 0, 1000, 1000),
                         output_rate=SIM.ts_video)

# initialize elements of the architecture
wind = wind_simulation(SIM.ts_simulation)
mav = mav_dynamics(SIM.ts_simulation)
Va = 0

# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    #-------set control surfaces-------------
    delta_e = -0.097
    delta_t = 0.99
    delta_a = 0.03
    delta_r = -0.002

    delta = np.array([delta_e, delta_t, delta_a, delta_r])

    #-------physical system-------------
    Va = mav._Va  # grab updated Va from MAV dynamics
    current_wind = wind.update(Va)  # get the new wind vector
    mav.update_state(delta, current_wind)  # propagate the MAV dynamics

    #-------update viewer-------------
    mav_view.update(mav.msg_true_state)  # plot body of MAV
    if DATA:
        data_view.update(mav.msg_true_state, # true states
                         mav.msg_true_state, # estimated states
                         mav.msg_true_state, # commanded states
                         SIM.ts_simulation)
    if VIDEO == True:
        video.update(sim_time)

    #-------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO == True:
    video.close()
