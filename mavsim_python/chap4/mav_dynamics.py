"""
    mav_dynamics
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state

"""
import sys
sys.path.append('..')
import numpy as np

# load message types
from message_types.msg_state import msg_state

import parameters.aerosonde_parameters as MAV
from tools.tools import Quaternion2Rotation, Quaternion2Euler

from IPython.core.debugger import Pdb

class mav_dynamics:
    def __init__(self, Ts):
        self._ts_simulation = Ts
        # set initial states based on parameter file
        # _state is the 13x1 internal state of the aircraft that is being propagated:
        # _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
        # We will also need a variety of other elements that are functions of the _state and the wind.
        # self.true_state is a 19x1 vector that is estimated and used by the autopilot to control the aircraft:
        # true_state = [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
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

        self.R_vb = Quaternion2Rotation(self._state[6:10,0])  # Rotation matrix from vehicle to body
        # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.zeros(3)  # wind in NED frame in meters/sec
        # store forces to avoid recalculation in the sensors function
        self._update_velocity_data()
        self._forces = np.zeros(3)
        self._Va = MAV.u0
        self._alpha = 0
        self._beta = 0
        # initialize true_state message
        self.msg_true_state = msg_state()


    ###################################
    # public functions
    def update_state(self, delta, wind):
        '''
            Integrate the differential equations defining dynamics, update sensors
            delta = (delta_a, delta_e, delta_r, delta_t) are the control inputs
            wind is the wind vector in inertial coordinates
            Ts is the time step between function calls.
        '''

        self._wind = wind[:3]

        # get forces and moments acting on rigid bod
        forces_moments = self._forces_moments(delta)

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts_simulation
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

        self.R_vb = Quaternion2Rotation(self._state[6:10,0])

        # update the airspeed, angle of attack, and side slip angles using new state
        self._update_velocity_data(wind)

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
        pn_dot, pe_dot, pd_dot = self.R_vb @ np.array([u, v, w])

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
        x_dot = np.array([pn_dot, pe_dot, pd_dot, u_dot, v_dot, w_dot,
                          e0_dot, e1_dot, e2_dot, e3_dot, p_dot, q_dot, r_dot])
        return x_dot

    def _update_velocity_data(self, wind=np.zeros(6)):

        # compute airspeed
        # Pdb().set_trace()
        V_wb = self.R_vb @ wind[:3] + wind[3:]
        V_ab = self._state[3:6] - V_wb
        self._Va = np.sqrt(np.sum(V_ab**2))

        # compute angle of attack
        self._alpha = np.arctan2(V_ab[2],V_ab[0])

        # compute sideslip angle
        self._beta = np.arcsin(V_ab[1]/self._Va)

    def _forces_moments(self, delta):
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :param delta: np.matrix(delta_a, delta_e, delta_r, delta_t)
        :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """
        delta_a = delta[0]
        delta_e = delta[1]
        delta_r = delta[2]
        delta_t = delta[3]

        # gravity
        fg = self.R_vb @ np.array([0,0, MAV.mass * 9.8])

        # propeller thrust and torque
        V_in = MAV.V_max * delta_t
        a = MAV.rho * MAV.D_prop**5 * MAV.C_Q0 / (2*np.pi)**2
        b = MAV.rho * MAV.D_prop**4 * MAV.C_Q1 * self._Va / (2*np.pi) + MAV.KQ*MAV.K_V/MAV.R_motor
        c = MAV.rho * MAV.D_prop**3 * MAV.C_Q2 * self._Va**2 - MAV.KQ * V_in / MAV.R_motor + MAV.KQ*MAV.i0
        Omega_op = (-b + np.sqrt(b**2 - 4*a*c)) / (2*a)

        J_op = 2 * np.pi * self._Va / (Omega_op * MAV.D_prop)
        C_T = MAV.C_T2 * J_op**2 + MAV.C_T1 * J_op + MAV.C_T0
        C_Q = MAV.C_Q2 * J_op**2 + MAV.C_Q1 * J_op + MAV.C_Q0
        n = Omega_op / (2 * np.pi)
        fp_x += MAV.rho * n**2 * MAV.D_prop**4 * C_T
        Mp_x += -MAV.rho * n**2 * MAV.D_prop**5 * C_Q
        fp = np.array([fp_x,0,0])  # force from propeller
        Mp = np.array([Mp_x,0,0])  # torque from propeller

        ## Aerodynamic forces/moments
        # Longitudinal
        Flift = 0.5*MAV.rho*self._Va**2 * MAV.S_wing * (MAV.C_L_0 + MAV.C_L_alpha*self._alpha)
        Fdrag = 0.5*MAV.rho*self._Va**2 * MAV.S_wing * (MAV.C_D_0 + MAV.C_D_alpha*self._alpha)
        m = 0.5*MAV.rho*self._Va**2 * MAV.S_wing * MAV.c * (MAV.C_m_0 + MAV.C_m_alpha*self._alpha)

        # Lateral
        fa_y = 0.5*MAV.rho*self._Va**2 * MAV.S_wing * (MAV.C_Y_0 + MAV.C_Y_beta*self._beta)
        l = 0.5*MAV.rho*self._Va**2 * MAV.S_wing * MAV.b * (MAV.C_ell_0 + MAV.C_ell_beta*self._beta)
        n = 0.5*MAV.rho*self._Va**2 * MAV.S_wing * MAV.b * (MAV.C_n_0 + MAV.C_n_beta*self._beta)

        # Combining into force/moment arrays
        sa = np.sin(self._alpha)
        ca = np.cos(self._alpha)
        [fa_x, fa_z] = np.array([[ca, -sa], [sa, ca]]) @ np.array([-Fdrag, -Flift])
        fa = np.array([fa_x, fa_y, fa_z])
        Ma = np.array([l, m, n])

        # Summing forces and moments
        [fx, fy, fz] = fg + fa + fp
        [Mx, My, Mz] = Ma + Mp
        self._forces[0] = fx
        self._forces[1] = fy
        self._forces[2] = fz
        return np.array([fx, fy, fz, Mx, My, Mz])

    def _update_msg_true_state(self):
        # update the class structure for the true state:
        #   [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        phi, theta, psi = Quaternion2Euler(self._state[6:10,0])
        self.msg_true_state.pn = self._state.item(0)
        self.msg_true_state.pe = self._state.item(1)
        self.msg_true_state.h = -self._state.item(2)
        self.msg_true_state.Va = self._Va
        self.msg_true_state.alpha = self._alpha
        self.msg_true_state.beta = self._beta
        self.msg_true_state.phi = phi
        self.msg_true_state.theta = theta
        self.msg_true_state.psi = psi
        self.msg_true_state.Vg = np.sqrt(np.sum(self._state[3:6]))
        self.msg_true_state.gamma = 0
        self.msg_true_state.chi = 0
        self.msg_true_state.p = self._state.item(10)
        self.msg_true_state.q = self._state.item(11)
        self.msg_true_state.r = self._state.item(12)
        self.msg_true_state.wn = self._wind.item(0)
        self.msg_true_state.we = self._wind.item(1)
