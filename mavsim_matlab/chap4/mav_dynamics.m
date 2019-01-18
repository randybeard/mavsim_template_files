% mav dynamics - implement rigid body dynamics for mav
%
% mavMatSim 
%     - Beard & McLain, PUP, 2012
%     - Update history:  
%         1/18/2019 - RWB
classdef mav_dynamics < handle
   %--------------------------------
    properties
        ts_simulation
        state
        Va
        alpha
        beta
        wind
        msg_true_state
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = mav_dynamics(Ts, MAV)
            self.ts_simulation = Ts; % time step between function calls
            self.state = [MAV.pn0; MAV.pe0; MAV.pd0; MAV.u0; MAV.v0; MAV.w0;...
                MAV.e0; MAV.e1; MAV.e2; MAV.e3; MAV.p0; MAV.q0; MAV.r0];
            self.Va =
            self.alpha = 
            self.beta = 
            self.wind = 
            addpath('../message_types'); self.msg_true_state = msg_state();
        end
        %---------------------------
        function self=update_state(self, delta, wind, MAV)
            %
            % Integrate the differential equations defining dynamics
            % forces_moments are the forces and moments on the MAV.
            % 
            
            % get forces and moments acting on rigid body
            forces_moments = self.forces_moments(delta, MAV);
            
            % Integrate ODE using Runge-Kutta RK4 algorithm
            k1 = self.derivatives(self.state, forces_moments, MAV);
            k2 = self.derivatives(self.state + self.ts_simulation/2*k1, forces_moments, MAV);
            k3 = self.derivatives(self.state + self.ts_simulation/2*k2, forces_moments, MAV);
            k4 = self.derivatives(self.state + self.ts_simulation*k3, forces_moments, MAV);
            self.state = self.state + self.ts_simulation/6 * (k1 + 2*k2 + 2*k3 + k4);
            
            % normalize the quaternion
            self.state(7:10) = self.state(7:10)/norm(self.state(7:10));
            
            % update the airspeed, angle of attack, and side slip angles
            self.update_velocity_data(wind);
            
            % update the message class for the true state
            self.update_msg_true_state();
        end
        %----------------------------
        function xdot = derivatives(self, state, forces_moments, MAV)
                
            % collect all the derivaties of the states
            xdot = [pn_dot; pe_dot; pd_dot; u_dot; v_dot; w_dot;...
                    e0_dot; e1_dot; e2_dot; e3_dot; p_dot; q_dot; r_dot];
        end
        %----------------------------
        function self=update_velocity_data(self, wind)
            self.wind = 
            self.Va = 
            self.alpha = 
            self.beta = 
        end
        %----------------------------
        function out=forces_moments(self, delta, MAV)
    
            % output total force and torque
            out = [Force'; Torque'];
        end
        %----------------------------
        function self=update_msg_true_state(self)
            [phi, theta, psi] = Quaternion2Euler(self.state(7:10));
            self.msg_true_state.pn = self.state(1);  % pn
            self.msg_true_state.pe = self.state(2);  % pd
            self.msg_true_state.h = -self.state(3);  % h
            self.msg_true_state.phi = phi; % phi
            self.msg_true_state.theta = theta; % theta
            self.msg_true_state.psi = psi; % psi
            self.msg_true_state.p = self.state(11); % p
            self.msg_true_state.q = self.state(12); % q
            self.msg_true_state.r = self.state(13); % r
            self.msg_true_state.Va = self.Va;
            self.msg_true_state.alpha = self.alpha;
            self.msg_true_state.beta = self.beta;
            self.msg_true_state.Vg = 
            self.msg_true_state.chi = 
            self.msg_true_state.gamma = 
            self.msg_true_state.wn = self.wind(1);
            self.msg_true_state.we = self.wind(2);
        end
    end
end