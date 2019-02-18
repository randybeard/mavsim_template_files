"""
mavDynamics 
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state
    
mavsim_python
    - Beard & McLain, PUP, 2012
    - Update history:  
        2/16/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np

# load message types
from message_types.msg_state import msg_state
from message_types.msg_sensors import msg_sensors

import parameters.aerosonde_parameters as MAV
import parameters.sensor_parameters as SENSOR
from tools.rotations import Quaternion2Rotation, Quaternion2Euler

class mav_dynamics:
    def __init__(self, Ts):
        self._forces = np.array([[0.], [0.], [0.]])
        # initialize the sensors message
        self.sensors = msg_sensors()
        # random walk parameters for GPS
        self._gps_eta_n = 0.
        self._gps_eta_e = 0.
        self._gps_eta_h = 0.
        # timer so that gps only updates every ts_gps seconds
        self._t_gps = 999.  # large value ensures gps updates at initial time.


    ###################################
    # public functions
    def update_state(self, delta, wind):

    def update_sensors(self):
        "Return value of sensors on MAV: gyros, accels, static_pressure, dynamic_pressure, GPS"
        self.sensors.gyro_x =
        self.sensors.gyro_y =
        self.sensors.gyro_z =
        self.sensors.accel_x =
        self.sensors.accel_y =
        self.sensors.accel_z =
        self.sensors.static_pressure =
        self.sensors.diff_pressure =
        if self._t_gps >= SENSOR.ts_gps:
            self._gps_eta_n =
            self._gps_eta_e =
            self._gps_eta_h =
            self.sensors.gps_n =
            self.sensors.gps_e =
            self.sensors.gps_h =
            self.sensors.gps_Vg =
            self.sensors.gps_course =
            self._t_gps = 0.
        else:
            self._t_gps += self._ts_simulation

    ###################################
    # private functions
    def _derivatives(self, state, forces_moments):

    def _update_velocity_data(self, wind=np.zeros((6,1))):

    def _forces_moments(self, delta):

        self._forces[0] = fx
        self._forces[1] = fy
        self._forces[2] = fz
        return np.array([[fx, fy, fz, Mx, My, Mz]]).T

    def _motor_thrust_torque(self, Va, delta_t):
        return T_p, Q_p


    def _update_true_state(self):
