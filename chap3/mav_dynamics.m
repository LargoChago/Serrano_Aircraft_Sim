% mav dynamics - implement rigid body dynamics for mav
%
% mavsimMatlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         2/16/2019 - RWB
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
        sensors
        forces
        gps_eta_n
        gps_eta_e
        gps_eta_h
        t_gps
        index
        vn
        ve
        vd
        rho
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = mav_dynamics(Ts, MAV)
            addpath('../message_types'); 
            self.true_state = msg_state();
            self.ts_simulation = Ts;
            self.state = [MAV.pn0; MAV.pe0; MAV.pd0; MAV.u0; MAV.v0; MAV.w0;...
                MAV.e0; MAV.e1; MAV.e2; MAV.e3; MAV.p0; MAV.q0; MAV.r0];
            v_b_g = [MAV.u0; MAV.v0; MAV.w0];
            v_b_a =v_b_g;
            self.Va = norm(v_b_a) + .0001;
            self.alpha = atan2(MAV.w0, MAV.u0);
            self.beta = asin(MAV.v0/self.Va);
            self.wind = [0 0 0 0 0 0]';
              addpath('../message_types'); self.true_state = msg_state();
            self.sensors = msg_sensors();
            self.gps_eta_n = 0;
            self.gps_eta_e = 0;
            self.gps_eta_h = 0;
            self.t_gps = 999;

            self.vn = zeros([1,100]);
            self.ve = zeros([1,100]);
            self.vd = zeros([1,100]);
            for t= 2:100
                self.vn(t) = exp((-1/1600)*self.vn(t-1)) + normrnd(0, 0.21);
                self.ve(t) = exp((-1/1600)*self.ve(t-1)) + normrnd(0, 0.21);
                self.vd(t) = exp((-1/1600)*self.vd(t-1)) + normrnd(0, 0.40);
            end
            self.index = 1;
        end

        %---------------------------
function self=update_state(self, delta, wind, MAV, axis)
forces_moments = self.forces_moments(delta, MAV);
if strcmp(axis, 'long')
    forces_moments(2) = 0;
    forces_moments(4) = 0;
    forces_moments(6) = 0;
elseif strcmp(axis,'lat')
    forces_moments(1) = 0;
    forces_moments(3) = 0;
    forces_moments(5) = 0;
end

   k1= self.derivatives(self.state, forces_moments, MAV);
   k2= self.derivatives(self.state + self.ts_simulation/2*k1, forces_moments, MAV);
   k3= self.derivatives(self.state + self.ts_simulation/2*k2, forces_moments, MAV);
   k4= self.derivatives(self.state + self.ts_simulation*k3, forces_moments, MAV);
   self.state = self.state + self.ts_simulation/6 * (k1+ 2*k2 + 2*k3 +k4);

   self.state(7:10) = self.state(7:10)/norm(self.state(7:10));
   
   self.update_velocity_data(wind);

   self.update_true_state();
        end
        %---------------------------
        function self=update_sensors(self, MAV, SENSOR)
   pn = self.state(1);
   pe = self.state(2);
   pd = self.state(3);
   u = self.state(4);
   v = self.state(5);
   w = self.state(6);
   e0 = self.state(7);
   e1 = self.state(8);
   e2 = self.state(9);
   e3 = self.state(10);
   p = self.state(11);
   q = self.state(12);
   r = self.state(13);
   fx = self.forces.x;
   fy = self.forces.y;
   fz = self.forces.z;
   [phi, theta, psi] = Quaternion2Euler(e0,e1,e2,e3);
            % Return value of sensors on MAV: gyros, accels, static_pressure, dynamic_pressure, GPS
            self.sensors.gyro_x = p + normrnd(0,SENSOR.gyro_sigma);
            self.sensors.gyro_y = q + normrnd(0,SENSOR.gyro_sigma);
            self.sensors.gyro_z = r + normrnd(0,SENSOR.gyro_sigma);
            self.sensors.accel_x = fx/MAV.mass + MAV.gravity*sin(theta) + normrnd(0,SENSOR.accel_sigma);
            self.sensors.accel_y = fy/MAV.mass - MAV.gravity*cos(theta)*sin(phi) + normrnd(0,SENSOR.accel_sigma);               
            self.sensors.accel_z = fz/MAV.mass - MAV.gravity*cos(theta)*cos(phi) + normrnd(0,SENSOR.accel_sigma);
     M=.0289;
     L0= -.0065;
R= 8.31;
g=9.81;
P0=101325;
T0=288.15;
hasl = 206.5;
P = P0 * (T0/(T0 + L0*hasl))^(g*M/(R*L0));
T= T0 + L0 * -pd;
rho = M*P/(R*T);
            self.sensors.static_pressure =  rho*g*(-pd) + normrnd(0,SENSOR.static_pres_sigma);
            self.sensors.diff_pressure =(1/2)*rho*(self.Va^2) + normrnd(0,SENSOR.diff_pres_sigma) ;   

            if self.t_gps >= SENSOR.ts_gps

%error n-e: 6.6
%error h: 9.2
% exp(-(1/16000))*

                self.gps_eta_n =0; 
                self.gps_eta_e =0;
                self.gps_eta_h =0;

                self.sensors.gps_n =   pn + self.vn(self.index);  
                self.sensors.gps_e =    pe + self.ve(self.index);
                self.sensors.gps_h =   pd + self.vd(self.index);
                wn = .05;
                we = .05;
                Vn= normrnd(self.Va*cos(psi), wn);
                Ve= normrnd(self.Va*sin(psi), we);
                self.sensors.gps_Vg = ((Vn^2 + Ve^2)^.5) + normrnd(0,SENSOR.gps_Vg_sigma);
                self.sensors.gps_course = atan2(Ve, Vn) + normrnd(0,SENSOR.gps_course_sigma);
                
                self.t_gps = 0;
            else
                self.t_gps = self.t_gps + self.ts_simulation;
            end
        end
        %----------------------------
        function xdot = derivatives(self, state, forces_moments, MAV)
             pn    = state(1);
            pe    = state(2);
            pd    = state(3);
            u     = state(4);
            v     = state(5);
            w     = state(6);
            e0    = state(7);
            e1    = state(8);
            e2    = state(9);
            e3    = state(10);
            p     = state(11);
            q     = state(12);
            r     = state(13);
            fx    = forces_moments(1);
            fy    = forces_moments(2);
            fz    = forces_moments(3);
            ell   = forces_moments(4);
            m     = forces_moments(5);
            n     = forces_moments(6);
        
            % position kinematics
            
            Reib = [e0^2+e1^2-e2^2-e3^2, 2*(e1*e2-e0*e3), 2*(e1*e3+e0*e2);
                    2*(e1*e2+e0*e3), e0^2-e1^2+e2^2-e3^2, 2*(e2*e3-e0*e1);
                    2*(e1*e3-e0*e2), 2*(e2*e3+e0*e1), e0^2-e1^2-e2^2+e3^2]*[u; v; w];
            pn_dot = Reib(1);
            pe_dot = Reib(2);
            pd_dot = Reib(3);

            % position dynamics
            u_dot = r*v-q*w+(1/MAV.mass)*fx;
            v_dot = p*w-r*u+(1/MAV.mass)*fy;
            w_dot = q*u-p*v+(1/MAV.mass)*fz;
            
            % rotational kinematics
            e = .5*[0, -p, -q, -r;
                    p, 0, r, -q;
                    q, -r, 0, p;
                    r, q, -p, 0]*[e0; e1; e2; e3];
                    
            e0_dot = e(1);
            e1_dot = e(2);
            e2_dot = e(3);
            e3_dot = e(4);
            
            % rotational dynamics
            g1 = MAV.Gamma1;
            g2 = MAV.Gamma2;
            g3 = MAV.Gamma3;
            g4 = MAV.Gamma4;
            g5 = MAV.Gamma5;
            g6 = MAV.Gamma6;
            g7 = MAV.Gamma7;
            g8 = MAV.Gamma8;
            p_dot = (g1*p*q-g2*q*r)+(g3*ell+g4*n);
            q_dot = (g5*p*r-g6*(p^2-r^2))+((1/MAV.Jy)*m);
            r_dot = (g7*p*q-g1*q*r)+(g4*ell+g8*n);
                
            % collect all the derivaties of the states
            xdot = [pn_dot; pe_dot; pd_dot; u_dot; v_dot; w_dot;...
                    e0_dot; e1_dot; e2_dot; e3_dot; p_dot; q_dot; r_dot];
        end
        %----------------------------
        function self=update_velocity_data(self, wind)
             v_b_g = self.state(4:6);
            v_b_a = v_b_g - wind(1:3) - wind(4:6);
            u     = self.state(4);
            v     = self.state(5);
            w     = self.state(6);
            self.wind = wind;
            self.Va = norm(v_b_a);  
            self.alpha = atan2(w, u);  
            self.beta = asin(v/self.Va);
        end
        %----------------------------
        function out=forces_moments(self, delta, MAV)
            pn    = self.state(1);
            pe    = self.state(2);
            pd    = self.state(3);
            u     = self.state(4);
            v     = self.state(5);
            w     = self.state(6);
            e0    = self.state(7);
            e1    = self.state(8);
            e2    = self.state(9);
            e3    = self.state(10);
            p     = self.state(11);
            q     = self.state(12);
            r     = self.state(13);
            delta_e = delta(1);
            delta_t = delta(2);
            delta_a = delta(3);
            delta_r = delta(4);
            
            my_alpha = self.alpha;
            [phi, theta, psi] = Quaternion2Euler(e0, e1, e2, e3);
            
            self.forces.x_grav = -MAV.mass*MAV.gravity*sin(theta);  
            self.forces.y_grav = MAV.mass*MAV.gravity*cos(theta)*sin(phi);  
            self.forces.z_grav = MAV.mass*MAV.gravity*cos(theta)*cos(phi);
           
            q_bar = .5 * MAV.rho * self.Va^2;
            
            sigmoid = (1+exp(-MAV.M*(my_alpha-MAV.alpha0)) + exp(MAV.M*(my_alpha+MAV.alpha0)))/((1+exp(-MAV.M*(my_alpha-MAV.alpha0)))*(1+exp(MAV.M*(my_alpha+MAV.alpha0))));
            flatplate = 2*sign(my_alpha)*sin(my_alpha)^2*cos(my_alpha);
            CL_alpha_func = (1-sigmoid)*(MAV.C_L_0 + MAV.C_L_alpha * my_alpha) + sigmoid*flatplate;
            AR = MAV.b^2/MAV.S_wing;
            CD_alpha_func = MAV.C_D_p + (MAV.C_L_0 + MAV.C_L_alpha * my_alpha)^2/(pi * MAV.e * AR);
%             f_lift = q_bar*MAV.S_wing*(CL_alpha_func + MAV.C_L_q*MAV.c*q/(2*self.Va)+ MAV.C_L_delta_e*delta_e); 
%             f_drag = q_bar*MAV.S_wing*(CD_alpha_func + MAV.C_D_q*MAV.c*q/(2*self.Va)+ MAV.C_D_delta_e*delta_e);
            self.forces.lift = q_bar*MAV.S_wing*(CL_alpha_func + MAV.C_L_q*MAV.c*q/(2*self.Va)+ MAV.C_L_delta_e*delta_e); 
            self.forces.drag = q_bar*MAV.S_wing*(CD_alpha_func + MAV.C_D_q*MAV.c*q/(2*self.Va)+ MAV.C_D_delta_e*delta_e);
            rot_mat = [cos(my_alpha) -sin(my_alpha); sin(my_alpha) cos(my_alpha)] * [-self.forces.drag; -self.forces.lift];
            self.forces.x_aero = rot_mat(1);
            self.forces.z_aero = rot_mat(2); 
            
            self.forces.y_aero = q_bar*MAV.S_wing*MAV.b*(MAV.C_Y_beta * self.beta + MAV.C_Y_delta_a * delta_a + MAV.C_Y_delta_r * delta_r);

%             Vin = MAV.V_max * delta_t;
%             A = MAV.C_Q0 * MAV.rho * MAV.D_prop^5/(2*pi)^2;
%             B = MAV.C_Q1 * MAV.rho * MAV.D_prop^4 * self.Va/(2*pi) + MAV.K_V * MAV.KQ/MAV.R_motor;
%             C = MAV.rho * MAV.D_prop^3 * MAV.C_Q2 * self.Va^2 - MAV.KQ*Vin/MAV.R_motor + MAV.KQ*MAV.i0;
%             Omega_p = (-B + sqrt(B^2 - 4*A*C))/(2*A);
%             
%             self.forces.thrust = (MAV.rho*MAV.D_prop^4*MAV.C_T0/(4*pi^2))*Omega_p^2 + (MAV.rho*MAV.D_prop^3*MAV.C_T1*self.Va/(2*pi))*Omega_p + ...
%                             (MAV.rho*MAV.D_prop^2 * MAV.C_T2 * self.Va^2);
            k_motor = 30;
            self.forces.thrust = .5 * MAV.rho * MAV.C_prop* MAV.S_prop * ((k_motor*delta_t)^2 - self.Va^2);
 
            self.forces.x = self.forces.x_grav + self.forces.x_aero + self.forces.thrust;
            self.forces.y = self.forces.y_grav + self.forces.y_aero;
            self.forces.z = self.forces.z_grav + self.forces.z_aero;
            
%             ell_prop = (MAV.rho*MAV.D_prop^5*MAV.C_Q0/(4*pi^2))*Omega_p^2 + (MAV.rho*MAV.D_prop^4*MAV.C_Q1*self.Va/(2*pi))*Omega_p + ...
%                             (MAV.rho*MAV.D_prop^3 * MAV.C_Q2 * self.Va^2);

            ell_prop = -.5*(delta_t)^2;
            ell_aero = q_bar*MAV.S_wing*MAV.b*(MAV.C_ell_0 + MAV.C_ell_beta * self.beta + MAV.C_ell_p * MAV.b/(2*self.Va) * p + ...
                        MAV.C_ell_r * MAV.b/(2*self.Va) * r + MAV.C_ell_delta_a * delta_a + MAV.C_ell_delta_r * delta_r);
            ell = ell_aero+ell_prop;  % moment about the x axis
            m = q_bar*MAV.S_wing*MAV.c*(MAV.C_m_0 + MAV.C_m_alpha * my_alpha + MAV.C_m_q*MAV.c*q/(2*self.Va)+ MAV.C_m_delta_e*delta_e);  % moment about the y axis
            n = q_bar*MAV.S_wing*MAV.b*(MAV.C_n_0 + MAV.C_n_beta * self.beta + MAV.C_n_p * MAV.b/(2*self.Va) * p + ...
                        MAV.C_n_r * MAV.b/(2*self.Va) * r + MAV.C_n_delta_a * delta_a + MAV.C_n_delta_r * delta_r);% moment about the z axis
            
            Force = [self.forces.x, self.forces.y, self.forces.z];
            Torque = [ell, m, n];
            % output total force and torque
            out = [Force'; Torque'];
        end
        %----------------------------
        function self=update_true_state(self)
             u     = self.state(4);
            v     = self.state(5);
            w     = self.state(6);
            e0    = self.state(7);
            e1    = self.state(8);
            e2    = self.state(9);
            e3    = self.state(10);
            Reib = [e0^2+e1^2-e2^2-e3^2, 2*(e1*e2-e0*e3), 2*(e1*e3+e0*e2);
                    2*(e1*e2+e0*e3), e0^2-e1^2+e2^2-e3^2, 2*(e2*e3-e0*e1);
                    2*(e1*e3-e0*e2), 2*(e2*e3+e0*e1), e0^2-e1^2-e2^2+e3^2]*[u; v; w];
            pn_dot = Reib(1);
            pe_dot = Reib(2);
            pd_dot = Reib(3);
            [phi, theta, psi] = Quaternion2Euler(self.state(7), self.state(8), self.state(9), self.state(10));
            self.true_state.pn = self.state(1);  % pn
            self.true_state.pe = self.state(2);  % pd
            self.true_state.h = -self.state(3);  % h
            self.true_state.phi = phi; % phi
            self.true_state.theta = theta; % theta
            self.true_state.psi = psi; % psi
            self.true_state.p = self.state(11); % p
            self.true_state.q = self.state(12); % q
            self.true_state.r = self.state(13); % r
            self.true_state.Va = self.Va;
            self.true_state.alpha = self.alpha;
            self.true_state.beta = self.beta;
            self.true_state.Vg = norm(self.state(4:6));
            self.true_state.chi = atan2(pe_dot, pn_dot);
            self.true_state.gamma = asin(-pd_dot/self.true_state.Vg);
            self.true_state.wn = self.wind(1);
            self.true_state.we = self.wind(2);
            function set_theta(self, theta)
            %Dont change the velocities just change the orientation and
            %keep the velocities constant.
            e = Euler2Quaternion(self.true_state.phi, theta, self.true_state.psi);
            self.state(7) = e(1);
            self.state(8) = e(2);
            self.state(9) = e(3);
            self.state(10) = e(4);
            self.update_true_state();
        end
        function set_longitudinal(self, Va, gamma, alpha)
            theta = gamma + alpha;
            e = Euler2Quaternion(0.0, theta, 0.0);
            self.state(7) = e(1);
            self.state(8) = e(2);
            self.state(9) = e(3);
            self.state(10) = e(4);

            %update velocity components
            u = cos(alpha)*Va;
            w = sin(alpha)*Va;
            self.state(4) = u;
            self.state(6) = w;
            self.update_velocity_data([0;0;0;0;0;0]);
            self.update_true_state();
        end
        function set_gamma(self, gamma)
        end
        end
    end
end