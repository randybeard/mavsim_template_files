"""
    mav_dynamics
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state

"""
import sys
sys.path.append('..')
import numpy as np
from math import asin, exp

# load message types
from message_types.msg_state import msg_state

import parameters.aerosonde_parameters as MAV
from tools.tools import Quaternion2Rotation, Quaternion2Euler

from IPython.core.debugger import Pdb
np.set_printoptions(linewidth=100)

class mav_dynamics:
    def __init__(self, Ts):
        self._ts_simulation = Ts
        # set initial states based on parameter file
        # _state is the 13x1 internal state of the aircraft that is being propagated:
        # _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
        # We will also need a variety of other elements that are functions of the _state and the wind.
        # self.true_state is a 19x1 vector that is estimated and used by the autopilot to control the aircraft:
        # true_state = [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        self._state = np.array([MAV.pn0,   # (0)
                                MAV.pe0,   # (1)
                                MAV.pd0,   # (2)
                                MAV.u0,    # (3)
                                MAV.v0,    # (4)
                                MAV.w0,    # (5)
                                MAV.e0,    # (6)
                                MAV.e1,    # (7)
                                MAV.e2,    # (8)
                                MAV.e3,    # (9)
                                MAV.p0,    # (10)
                                MAV.q0,    # (11)
                                MAV.r0])   # (12)

        self.R_vb = Quaternion2Rotation(self._state[6:10])  # Rotation matrix from vehicle to body
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
        # Pdb().set_trace()

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
        e0 = self._state[6]
        e1 = self._state[7]
        e2 = self._state[8]
        e3 = self._state[9]
        normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        self._state[6] = self._state[6]/normE
        self._state[7] = self._state[7]/normE
        self._state[8] = self._state[8]/normE
        self._state[9] = self._state[9]/normE

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
        pn = state[0]
        pe = state[1]
        pd = state[2]
        u  = state[3]
        v  = state[4]
        w  = state[5]
        e0 = state[6]
        e1 = state[7]
        e2 = state[8]
        e3 = state[9]
        p  = state[10]
        q  = state[11]
        r  = state[12]
        #   extract forces/moments
        fx = forces_moments[0]
        fy = forces_moments[1]
        fz = forces_moments[2]
        l  = forces_moments[3]
        m  = forces_moments[4]
        n  = forces_moments[5]

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
        self.R_vb = Quaternion2Rotation(self._state[6:10])

        # compute airspeed
        V_wb = self.R_vb @ wind[:3] + wind[3:]
        V_ab = self._state[3:6] - V_wb
        self._Va = np.linalg.norm(V_ab)

        # compute angle of attack
        self._alpha = np.arctan2(V_ab[2],V_ab[0])

        # compute sideslip angle
        self._beta = asin(V_ab[1]/self._Va)

    def _forces_moments(self, delta):
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :param delta: np.matrix(delta_a, delta_e, delta_r, delta_t)
        :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """
        de = delta[0]
        dt = delta[1]
        da = delta[2]
        dr = delta[3]
        delta_t = delta[3]

        # gravity
        fg = self.R_vb @ np.array([0,0, MAV.mass * MAV.gravity])  #### CHECK

        # propeller thrust and torque
        rho = MAV.rho
        D = MAV.D_prop
        Va = self._Va

        V_in = MAV.V_max * delta_t
        a = rho * D**5 * MAV.C_Q0 / (2*np.pi)**2
        b = rho * D**4 * MAV.C_Q1 * Va / (2*np.pi) + MAV.KQ**2/MAV.R_motor
        c = rho * D**3 * MAV.C_Q2 * Va**2 - (MAV.KQ * V_in) / MAV.R_motor + MAV.KQ*MAV.i0
        Omega_op = (-b + np.sqrt(b**2 - 4*a*c)) / (2*a)

        J_op = 2 * np.pi * Va / (Omega_op * D)
        C_T = MAV.C_T2 * J_op**2 + MAV.C_T1 * J_op + MAV.C_T0
        C_Q = MAV.C_Q2 * J_op**2 + MAV.C_Q1 * J_op + MAV.C_Q0
        n = Omega_op / (2 * np.pi)
        fp_x = rho * n**2 * D**4 * C_T
        Mp_x = rho * n**2 * D**5 * C_Q
        fp = np.array([fp_x,0,0])  # force from propeller
        Mp = np.array([Mp_x,0,0])  # torque from propeller

        ## Aerodynamic forces/moments
        # Longitudinal
        M = MAV.M
        alpha = self._alpha
        alpha0 = MAV.alpha0
        rho = MAV.rho
        Va = self._Va
        S = MAV.S_wing
        q = self._state[11]
        c = MAV.c

        sigma_alpha = (1 + exp(-M * (alpha - alpha0)) + exp(M * (alpha + alpha0))) /\
                      ((1 + exp(-M * (alpha - alpha0)))*(1 + exp(M * (alpha + alpha0))))
        CL_alpha = (1 - sigma_alpha) * (MAV.C_L_0 + MAV.C_L_alpha * alpha) + \
                    sigma_alpha * (2 * np.sign(alpha) * (np.sin(alpha)**2) * np.cos(alpha))
        F_lift = 0.5 * rho * (Va**2) * S * (CL_alpha + MAV.C_L_q * (c / (2 * Va)) * q \
                 + MAV.C_L_delta_e * de)
        CD_alpha = MAV.C_D_p + ((MAV.C_L_0 + MAV.C_L_alpha * alpha)**2) / (np.pi * MAV.e * MAV.AR)
        F_drag = 0.5 * rho * (Va**2) * S * (CD_alpha + MAV.C_D_q * (c / (2 * Va)) * q \
                 + MAV.C_D_delta_e * de)
        m = 0.5 * rho * (Va**2) * S * c * (MAV.C_m_0 + MAV.C_m_alpha * alpha + \
            MAV.C_m_q * (c / (2 * Va)) * q + MAV.C_m_delta_e * de)

        # Lateral
        b = MAV.b
        Va = self._Va
        beta = self.msg_true_state.beta
        p = self.msg_true_state.p
        r = self.msg_true_state.r
        rho = MAV.rho
        S = MAV.S_wing


        # fa_y = 0.5*MAV.rho*self._Va**2 * MAV.S_wing * (MAV.C_Y_0 + MAV.C_Y_beta*self._beta)
        # l = 0.5*MAV.rho*self._Va**2 * MAV.S_wing * MAV.b * (MAV.C_ell_0 + MAV.C_ell_beta*self._beta)
        # n = 0.5*MAV.rho*self._Va**2 * MAV.S_wing * MAV.b * (MAV.C_n_0 + MAV.C_n_beta*self._beta)

        # Calculating fy
        fa_y = 1/2.0 * rho * (Va**2) * S * (MAV.C_Y_0 + MAV.C_Y_beta * beta + MAV.C_Y_p * (b / (2*Va)) * p +\
             MAV.C_Y_r * (b / (2 * Va)) * r + MAV.C_Y_delta_a * da + MAV.C_Y_delta_r * dr)

        # Calculating l
        l = 1/2.0 * rho * (Va**2) * S * b * (MAV.C_ell_0 + MAV.C_ell_beta * beta + MAV.C_ell_p * (b/(2*Va)) * p +\
            MAV.C_ell_r * (b/(2*Va)) * r + MAV.C_ell_delta_a * da + MAV.C_ell_delta_r * dr)

        # Calculating n
        n = 1/2.0 * rho * (Va**2) * S * b * (MAV.C_n_0 + MAV.C_n_beta * beta + MAV.C_n_p * (b/(2*Va)) * p +\
            MAV.C_n_r * (b/(2*Va)) * r + MAV.C_n_delta_a * da + MAV.C_n_delta_r * dr)

        # Combining into force/moment arrays
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        [fa_x, fa_z] = np.array([[ca, -sa], [sa, ca]]) @ np.array([-F_drag, -F_lift])
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
        phi, theta, psi = Quaternion2Euler(self._state[6:10])
        self.msg_true_state.pn = self._state[0]
        self.msg_true_state.pe = self._state[1]
        self.msg_true_state.h = -self._state[2]
        self.msg_true_state.Va = self._Va
        self.msg_true_state.alpha = self._alpha
        self.msg_true_state.beta = self._beta
        self.msg_true_state.phi = phi
        self.msg_true_state.theta = theta
        self.msg_true_state.psi = psi
        self.msg_true_state.Vg = np.linalg.norm(self._state[3:6])
        self.msg_true_state.gamma = np.arctan2(-self._state[5], self._state[3])
        self.msg_true_state.chi = np.arctan2(self._state[4], self._state[3]) + psi
        self.msg_true_state.p = self._state[10]
        self.msg_true_state.q = self._state[11]
        self.msg_true_state.r = self._state[12]
        self.msg_true_state.wn = self._wind[0]
        self.msg_true_state.we = self._wind[1]
