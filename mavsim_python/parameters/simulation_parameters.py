import sys
sys.path.append('..')
import numpy as np

######################################################################################
                #   sample times, etc
######################################################################################
ts_simulation = 0.02  # smallest time step for simulation
start_time = 0.  # start time for simulation
end_time = 50.  # end time for simulation

ts_plotting = 0.5  # refresh rate for plots

ts_video = 0.1  # write rate for video

ts_control = ts_simulation  # sample rate for the controller


