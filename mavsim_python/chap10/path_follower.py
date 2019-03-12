import numpy as np
from math import sin, cos, atan, atan2
import sys

sys.path.append('..')
from message_types.msg_autopilot import msg_autopilot

class path_follower:
    def __init__(self):
        self.chi_inf =   # approach angle for large distance from straight-line path
        self.k_path =   # proportional gain for straight-line path following
        self.k_orbit =   # proportional gain for orbit following
        self.gravity = 9.8
        self.autopilot_commands = msg_autopilot()  # message sent to autopilot

    def update(self, path, state):
        if path.flag=='line':
            self._follow_straight_line(path, state)
        elif path.flag=='orbit':
            self._follow_orbit(path, state)
        return self.autopilot_commands

    def _follow_straight_line(self, path, state):
        self.autopilot_commands.airspeed_command =
        self.autopilot_commands.course_command =
        self.autopilot_commands.altitude_command =
        self.autopilot_commands.phi_feedforward =

    def _follow_orbit(self, path, state):
        self.autopilot_commands.airspeed_command =
        self.autopilot_commands.course_command =
        self.autopilot_commands.altitude_command =
        self.autopilot_commands.phi_feedforward =

    def _wrap(self, chi_c, chi):
        while chi_c-chi > np.pi:
            chi_c = chi_c - 2.0 * np.pi
        while chi_c-chi < -np.pi:
            chi_c = chi_c + 2.0 * np.pi
        return chi_c

