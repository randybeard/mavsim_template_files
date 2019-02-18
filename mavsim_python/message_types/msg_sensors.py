"""
msg_sensors
    - messages type for output of sensors
    
part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        2/16/2019 - RWB
"""

class msg_sensors:
    def __init__(self):
        self.gyro_x = 0  # gyro_x
        self.gyro_y = 0  # gyro_y
        self.gyro_z = 0  # gyro_z
        self.accel_x = 0  # accel_x
        self.accel_y = 0  # accel_y
        self.accel_z = 0  # accel_z
        self.static_pressure = 0  # static pressure
        self.diff_pressure = 0  # differential pressure
        self.gps_n = 0  # gps north
        self.gps_e = 0  # gps east
        self.gps_h = 0  # gps altitude
        self.gps_Vg = 0  # gps ground speed
        self.gps_course = 0  # gps course angle