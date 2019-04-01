import numpy as np
import sys
sys.path.append('..')
from chap11.dubins_parameters import dubins_parameters
from message_types.msg_path import msg_path

class path_manager:
    def __init__(self):
        # message sent to path follower
        self.path = msg_path()
        # pointers to previous, current, and next waypoints
        self.ptr_previous = 0
        self.ptr_current = 1
        self.ptr_next = 2
        # flag that request new waypoints from path planner
        self.flag_need_new_waypoints = True
        self.num_waypoints = 0
        self.halfspace_n = np.inf * np.ones((3,1))
        self.halfspace_r = np.inf * np.ones((3,1))
        # state of the manager state machine
        self.manager_state = 1
        # dubins path parameters
        self.dubins_path = dubins_parameters()

    def update(self, waypoints, radius, state):
        def update(self, waypoints, radius, state):
            # this flag is set for one time step to signal a redraw in the viewer
            if self.path.flag_path_changed == True:
                self.path.flag_path_changed = False
            if waypoints.num_waypoints == 0:
                waypoints.flag_manager_requests_waypoints = True
            else:
                if waypoints.type == 'straight_line':
                    self.line_manager(waypoints, state)
                elif waypoints.type == 'fillet':
                    self.fillet_manager(waypoints, radius, state)
                elif waypoints.type == 'dubins':
                    self.dubins_manager(waypoints, radius, state)
                else:
                    print('Error in Path Manager: Undefined waypoint type.')
            return self.path

    def line_manager(self, waypoints, state):

    def fillet_manager(self, waypoints, radius, state):

    def dubins_manager(self, waypoints, radius, state):

    def initialize_pointers(self):

    def increment_pointers(self):

    def inHalfSpace(self, pos):
        if (pos-self.halfspace_r).T @ self.halfspace_n >= 0:
            return True
        else:
            return False

