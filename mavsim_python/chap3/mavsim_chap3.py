"""
mavSimPy
    - Chapter 3 assignment for Beard & McLain, PUP, 2012
    - Update history:
        1/10/2019 - RWB
"""
import sys
sys.path.append('..')

# import viewers and video writer
from chap2.mav_viewer import mav_viewer
from chap2.video_writer import video_writer
from chap3.mav_dynamics import mav_dynamics
from chap3.data_viewer import data_viewer

# import parameters
import parameters.simulation_parameters as SIM
# import message types
from message_types.msg_state import msg_state

from IPython.core.debugger import Pdb

# initialize viewers and video
VIDEO = True  # True==write video, False==don't write video
mav_view = mav_viewer()
data_view = data_viewer()
mav = mav_dynamics(SIM.ts_simulation)
cmd_state = msg_state()
Pdb().set_trace()
if VIDEO:
    video = video_writer(video_name="chap3_video.avi",
                         bounding_box=(0, 0, 1000, 1000),
                         output_rate=SIM.ts_video)


# main simulation loop
for i in range(6):
    # initialize the simulation time
    sim_time = SIM.start_time
    forces_moments = np.zeros(6)  # fx, fy, fz, l, m, n
    if i < 3:
        val = 100
    else:
        val = 0.05
    forces_moments[i] = val
    mav.reset_state()
    while sim_time < SIM.end_time:
        #-------vary states to check viewer-------------
        mav.update_state(forces_moments)

        #-------update viewer and video-------------
        # Pdb().set_trace()
        mav_view.update(mav.msg_true_state)  # plot body of MAV
        data_view.update(mav.msg_true_state, # true states
                         mav.msg_true_state, # estimated states
                         mav.msg_true_state, # commanded states
                         SIM.ts_simulation)

        if VIDEO: video.update(sim_time)

        #-------increment time-------------
        sim_time += SIM.ts_simulation

print("Press Ctrl-Q to exit...")
if VIDEO == True: video.close()
