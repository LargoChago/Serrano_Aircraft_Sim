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
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = mav_dynamics(Ts, MAV)
            addpath('../message_types'); 
            self.forces = [0; 0; 0];
            self.sensors = msg_sensors();
            self.gps_eta_n = 0;
            self.gps_eta_e = 0;
            self.gps_eta_h = 0;
            self.t_gps = 999;
        end
        %---------------------------
        function self=update_state(self, delta, wind, MAV)
        end
        %---------------------------
        function self=update_sensors(self, MAV, SENSOR)
            % Return value of sensors on MAV: gyros, accels, static_pressure, dynamic_pressure, GPS
            self.sensors.gyro_x = p + normrnd(0,SENSOR.gyro_sigma^2);
            self.sensors.gyro_y = q + normrnd(0,SENSOR.gyro_sigma^2);
            self.sensors.gyro_z = r + normrnd(0,SENSOR.gyro_sigma^2);
            self.sensors.accel_x = udot + q*w - r*v + MAV.gravity*sin(theta) + normrnd(0,SENSOR.accel_sigma^2);
            self.sensors.accel_y = vdot + r*u - p*w - MAV.gravity*cos(theta)*sin(phi) + normrnd(0,SENSOR.accel_sigma^2);               
            self.sensors.accel_z = wdot + p*v - q*u - MAV.gravity*cos(theta)*cos(phi) + normrnd(0,SENSOR.accel_sigma^2);
         % qbar= (1/2)*MAV.rho*(Va^2);
          % betadiff = .02;
          %  101325 * (288.15 / (288.15 + (.0065* altitude)) )^((9.81*.0289)/(8.31*.0065));
            self.sensors.static_pressure =  (101325 * (288.15 / (288.15 + (.0065* altitude)) )^((9.81432*.0289)/(8.31*.0065)))  + normrnd(0,SENSOR.static_pres_sigma^2);
            self.sensors.diff_pressure =(1/2)*MAV.rho*(self.Va^2) + normrnd(0,SENSOR.diff_pres_sigma^2) ;   

            if self.t_gps >= SENSOR.ts_gps

%error n-e: 6.6
%error h: 9.2
% exp(-(1/16000))*

                self.gps_eta_n =exp(-(1/16000))*v(t-1) + normrnd(0,SENSOR.gps_n_sigma^2); %is this the v[N] term?
                self.gps_eta_e =exp(-(1/16000))*v(t-1) + normrnd(0,SENSOR.gps_e_sigma^2);
                self.gps_eta_h =exp(-(1/16000))*v(t-1) + normrnd(0,SENSOR.gps_h_sigma^2);

                self.sensors.gps_n =    Pn(t) + self.gps_eta_n;  
                self.sensors.gps_e =    Pe(t) + self.gps_eta_e;
                self.sensors.gps_h =   -Pd(t) + self.gps_eta_h;

                self.sensors.gps_Vg = ((((self.Va*cos(phi) + wn)^2) + ((self.Va*sin(phi) + we)^2))^.5) + normrnd(0,SENSOR.gps_Vg_sigma^2);
                self.sensors.gps_course = atan2(self.Va*sin(phi) + we, self.Va*cos(phi) + wn) + normrnd(0,SENSOR.gps_course_sigma^2);
                
                self.t_gps = 0;
            else
                self.t_gps = self.t_gps + self.ts_simulation;
            end
        end
        %----------------------------
        function xdot = derivatives(self, state, forces_moments, MAV)
        end
        %----------------------------
        function self=update_velocity_data(self, wind)
        end
        %----------------------------
        function out=forces_moments(self, delta, MAV)
            
            self.forces = Force;
            out = [Force'; Torque'];
        end
        %----------------------------
        function self=update_true_state(self)
        end
    end
end