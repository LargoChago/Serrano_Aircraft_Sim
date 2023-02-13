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
        true_state
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = mav_dynamics(Ts, MAV)
            self.ts_simulation = Ts; % time step between function calls
            self.state = [MAV.pn0; MAV.pe0; MAV.pd0; MAV.u0; MAV.v0; MAV.w0;...
                MAV.e0; MAV.e1; MAV.e2; MAV.e3; MAV.p0; MAV.q0; MAV.r0];
            self.Va = 6.9;  %velocity
            self.alpha = 0 ; %self.alpha
            self.beta = 0 ; %self.beta
            self.wind = [0; 0; 0; 0];
            addpath('../message_types'); self.true_state = msg_state();
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
            k1 = self.deriself.Vatives(self.state, forces_moments, MAV);
            k2 = self.deriself.Vatives(self.state + self.ts_simulation/2*k1, forces_moments, MAV);
            k3 = self.deriself.Vatives(self.state + self.ts_simulation/2*k2, forces_moments, MAV);
            k4 = self.deriself.Vatives(self.state + self.ts_simulation*k3, forces_moments, MAV);
            self.state = self.state + self.ts_simulation/6 * (k1 + 2*k2 + 2*k3 + k4);
            
            % normalize the quaternion
            self.state(7:10) = self.state(7:10)/norm(self.state(7:10));
            
            % update the airspeed, angle of attack, and side slip angles
            self.update_velocity_data(wind);
            
            % update the message class for the true state
            self.update_true_state();
        end
        %----------------------------
        function xdot = deriself.Vatives(self, state, forces_moments, MAV)
                
            % collect all the deriself.Vaties of the states
            xdot = [pn_dot; pe_dot; pd_dot; u_dot; v_dot; w_dot;...
                    e0_dot; e1_dot; e2_dot; e3_dot; p_dot; q_dot; r_dot];
        end
        %----------------------------
        function self=update_velocity_data(self, wind)
            self.wind = self.wind ;
            self.Va = ((u^2)+(v^2)+(w^2))^1/2 ;
            self.alpha = atan(w/u) ;
            self.beta = asin(v/(((u^2)+(v^2)+(w^2))^1/2)) ;
        end
        %----------------------------
        function out=forces_moments(self, delta, MAV)
    
            f_a_x=( -1*MAV.mass*MAV.gravity*sin(theta)) + (.5*MAV.rho*(self.Va^2)*MAV.S_wing)*( (((MAV.C_D_self.alpha*cos(self.alpha) )) + ( MAV.C_L_self.alpha*sin(self.alpha) ))  + ((( (MAV.C_D_q*cos(self.alpha) ) + ( MAV.C_L_q*sin(self.alpha) )  )) * (MAV.c/(2*self.Va))*q) + ((( (MAV.C_D_delta_e*cos(self.alpha) )  + ( MAV.C_L_delta_e*sin(self.alpha) ) ))*delta_e));
f_a_y= (MAV.mass*MAV.gravity*cos(theta)*sin(phi)) +  (.5*MAV.rho*(self.self.Va^2)*MAV.S_wing)*( ( MAV.C_Y_0 + (MAV.C_Y_self.beta *self.beta) + (MAV.C_Y_p*(MAV.b/(2*self.Va))*p) + MAV.C_Y_r *(MAV.b/(2*self.Va))*r) +(MAV.C_Y_delta_a *delta_a) + (MAV.C_Y_delta_r *delta_r)) ;
f_a_z= (MAV.mass*MAV.gravity*cos(theta)*cos(phi)) +  (.5*MAV.rho*(self.Va^2)*MAV.S_wing)*( (((MAV.C_D_self.alpha*sin(self.alpha))+(MAV.C_L_self.alpha*cos(self.alpha))) +((MAV.C_D_q*sin(self.alpha)+MAV.C_L_q*cos(self.alpha)) * (MAV.c/(2*self.Va))*q) + ( (MAV.C_D_delta_e*sin(self.alpha)  + MAV.C_L_delta_e*cos(self.alpha)) *delta_e)));
            
            
          ell=(.5*MAV.rho*(self.Va^2)* MAV.S_wing*MAV.b) * (MAV.C_ell_0+(MAV.C_ell_self.beta*self.beta) + (MAV.C_ell_p *    (b/(2*self.Va))*p) + (MAV.C_ell_r * (b/(2*self.Va))*r) + (MAV.C_ell_delta_a * delta_a) + (MAV.C_ell_delta_r * delta_r));

m=(.5*MAV.rho*(self.Va^2)* MAV.S_wing) * (MAV.C_m_0+ (MAV.C_m_self.alpha * self.alpha) + (MAV.C_m_q * (c/(3*self.Va))* q) + (MAV.C_m_delta_e * delta_e));

n=(.5*MAV.rho*(self.Va^2)*MAV.S_wing*MAV.b)* (MAV.C_n_0+ (MAV.C_n_self.beta*self.beta) + (MAV.C_n_p * (b/(2*self.Va))*p) + (MAV.C_n_r * (b/(2*self.Va))*r) + (MAV.C_n_delta_a* delta_a) + (MAV.C_n_delta_r*delta_r));
Force = [f_a_x; f_a_y; f_a_z];            
Torque =     [ell;m;n];
            % output total force and torque
            out = [Force'; Torque'];
        end
        %----------------------------
        function self=update_true_state(self)
            [phi, theta, psi] = Quaternion2Euler(self.state(7:10));
            self.true_state.pn = self.state(1);  % pn
            self.true_state.pe = self.state(2);  % pd
            self.true_state.h = -self.state(3);  % h
            self.true_state.phi = phi; % phi
            self.true_state.theta = theta; % theta
            self.true_state.psi = psi; % psi
            self.true_state.p = self.state(11); % p
            self.true_state.q = self.state(12); % q
            self.true_state.r = self.state(13); % r
            self.true_state.self.Va = self.Va;
            self.true_state.self.alpha = self.alpha;
            self.true_state.self.beta = self.beta;
            self.true_state.Vg = self.Va; %self.Va +Vg but Vg=0
            self.true_state.chi = theta - self.alpha; 
            self.true_state.gamma = psi - self.beta;
            self.true_state.wn = self.wind(1);
            self.true_state.we = self.wind(2);
        end
    end
end