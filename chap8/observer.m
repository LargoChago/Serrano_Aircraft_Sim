% observer for mavsim_matlab
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         3/2/2019 - RWB
classdef observer < handle
   %--------------------------------
    properties
        estimated_state
        lpf_gyro_x
        lpf_gyro_y
        lpf_gyro_z
        lpf_accel_x
        lpf_accel_y
        lpf_accel_z
        lpf_static
        lpf_diff
        attitude_ekf
        position_ekf
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = observer(ts_control)
            addpath('../message_types'); 
            self.estimated_state = msg_state();
            addpath('../chap8')
            self.lpf_gyro_x = alpha_filter(0.5);
            self.lpf_gyro_y = alpha_filter(0.5);
            self.lpf_gyro_z = alpha_filter(0.5);
            self.lpf_accel_x = alpha_filter(0.5);
            self.lpf_accel_y = alpha_filter(0.5);
            self.lpf_accel_z = alpha_filter(0.5);
            self.lpf_static = alpha_filter(0.5);
            self.lpf_diff = alpha_filter(0.5);
            self.attitude_ekf = ekf_attitude();
            self.position_ekf = ekf_position();
            % load AP: control gains/parameters
%             run('../parameters/control_parameters') 

% 
        end
        %------methods-----------
        function estimated_state = update(self, measurements, MAV)
            % estimates for p, q, r are low pass filter of gyro minus bias estimate
            self.estimated_state.p = self.lpf_gyro_x.update(measurements.gyro_x) ;
            self.estimated_state.q = self.lpf_gyro_y.update(measurements.gyro_y) ;
            self.estimated_state.r = self.lpf_gyro_z.update(measurements.gyro_z) ;

            % invert sensor model to get altitude and airspeed
            self.estimated_state.h = self.lpf_static.update(measurements.static_pressure)/(MAV.rho*MAV.gravity) ;
            self.estimated_state.Va = ((2/MAV.rho)*self.lpf_diff.update(measurements.diff_pressure))^.5;

            % estimate phi and theta with simple ekf
            self.estimated_state = self.attitude_ekf.update(self.estimated_state, measurements);

            % estimate pn, pe, Vg, chi, wn, we, psi
            self.estimated_state = self.position_ekf.update(self.estimated_state, measurements);

            % not estimating these
            self.estimated_state.alpha = self.estimated_state.theta;
            self.estimated_state.beta = 0.0;
            self.estimated_state.bx = 0.0;
            self.estimated_state.by = 0.0;
            self.estimated_state.bz = 0.0;
            
            % return the estimated state
            estimated_state = self.estimated_state;
        end
    end
end