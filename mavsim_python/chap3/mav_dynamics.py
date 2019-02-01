"""
mav_dynamics
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state

part of mavsimPy
    - Beard & McLain, PUP, 2012
    - Update history:
        12/17/2018 - RWB
        1/14/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np

# load message types
from message_types.msg_state import msg_state

import parameters.aerosonde_parameters as MAV
from tools.tools import Quaternion2Euler

from IPython.core.debugger import Pdb

class mav_dynamics:
    def __init__(self, Ts):
        self.ts_simulation = Ts
        # set initial states based on parameter file
        self.reset_state()
        self.msg_true_state = msg_state()

    ###################################
    # public functions

    def reset_state(self):
        # _state is the 13x1 internal state of the aircraft that is being propagated:
        # _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
        self._state = np.array([[MAV.pn0],  # (0)
                               [MAV.pe0],   # (1)
                               [MAV.pd0],   # (2)
                               [MAV.u0],    # (3)
                               [MAV.v0],    # (4)
                               [MAV.w0],    # (5)
                               [MAV.e0],    # (6)
                               [MAV.e1],    # (7)
                               [MAV.e2],    # (8)
                               [MAV.e3],    # (9)
                               [MAV.p0],    # (10)
                               [MAV.q0],    # (11)
                               [MAV.r0]])   # (12)


    def update_state(self, forces_moments):
        '''

            Integrate the differential equations defining dynamics.
            Inputs are the forces and moments on the aircraft.
            Ts is the time step between function calls.
        '''

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self.ts_simulation
        k1 = self._derivatives(self._state, forces_moments)
        k2 = self._derivatives(self._state + time_step/2.*k1, forces_moments)
        k3 = self._derivatives(self._state + time_step/2.*k2, forces_moments)
        k4 = self._derivatives(self._state + time_step*k3, forces_moments)
        self._state += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)

        # normalize the quaternion
        e0 = self._state.item(6)
        e1 = self._state.item(7)
        e2 = self._state.item(8)
        e3 = self._state.item(9)
        normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        self._state[6][0] = self._state.item(6)/normE
        self._state[7][0] = self._state.item(7)/normE
        self._state[8][0] = self._state.item(8)/normE
        self._state[9][0] = self._state.item(9)/normE

        # update the message class for the true state
        self._update_msg_true_state()

    ###################################
    # private functions
    def _derivatives(self, state, forces_moments):
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
        # extract the states
        pn = state.item(0)
        pe = state.item(1)
        pd = state.item(2)
        u = state.item(3)
        v = state.item(4)
        w = state.item(5)
        e0 = state.item(6)
        e1 = state.item(7)
        e2 = state.item(8)
        e3 = state.item(9)
        p = state.item(10)
        q = state.item(11)
        r = state.item(12)
        #   extract forces/moments
        fx = forces_moments.item(0)
        fy = forces_moments.item(1)
        fz = forces_moments.item(2)
        l = forces_moments.item(3)
        m = forces_moments.item(4)
        n = forces_moments.item(5)

        # position kinematics
        mat_pos = np.array([[e1**2+e0**2-e2**2-e3**2, 2*(e1*e2-e3*e0), 2*(e1*e3+e2*e0)],
                      [2*(e1*e2+e3*e0), e2**2+e0**2-e1**2-e3**2, 2*(e2*e3-e1*e0)],
                      [2*(e1*e3-e2*e0), 2*(e2*e3+e1*e0), e3**2+e0**2-e1**2-e2**2]])
        pn_dot, pe_dot, pd_dot = mat_pos @ np.array([u, v, w])

        # position dynamics
        vec_pos = np.array([r*v - q*w, p*w - r*u, q*u - p*v])
        u_dot, v_dot, w_dot = vec_pos + 1/MAV.mass * np.array([fx, fy, fz])

        # rotational kinematics
        mat_rot = np.array([[0, -p, -q, -r],
                            [p, 0, r, -q],
                            [q, -r, 0, p],
                            [r, q, -p, 0]])
        e0_dot, e1_dot, e2_dot, e3_dot = 0.5*mat_rot @ np.array([e0,e1,e2,e3])

        # rotatonal dynamics
        G = MAV.gamma
        G1 = MAV.gamma1
        G2 = MAV.gamma2
        G3 = MAV.gamma3
        G4 = MAV.gamma4
        G5 = MAV.gamma5
        G6 = MAV.gamma6
        G7 = MAV.gamma7
        G8 = MAV.gamma8

        vec_rot = np.array([G1*p*q - G2*q*r, G5*p*r - G6*(p**2-r**2), G7*p*q - G1*q*r])
        vec_rot2 = np.array([G3*l + G4*n, m/MAV.Jy, G4*l + G8*n])

        p_dot, q_dot, r_dot = vec_rot + vec_rot2

        # collect the derivative of the states
        x_dot = np.array([[pn_dot, pe_dot, pd_dot, u_dot, v_dot, w_dot,
                           e0_dot, e1_dot, e2_dot, e3_dot, p_dot, q_dot, r_dot]]).T
        # Pdb().set_trace()
        return x_dot

    def _update_msg_true_state(self):
        # update the true state message:
        phi, theta, psi = Quaternion2Euler(self._state[6:10])
        self.msg_true_state.pn = self._state.item(0)
        self.msg_true_state.pe = self._state.item(1)
        self.msg_true_state.h = -self._state.item(2)
        self.msg_true_state.phi = phi
        self.msg_true_state.theta = theta
        self.msg_true_state.psi = psi
        self.msg_true_state.p = self._state.item(10)
        self.msg_true_state.q = self._state.item(11)
        self.msg_true_state.r = self._state.item(12)
