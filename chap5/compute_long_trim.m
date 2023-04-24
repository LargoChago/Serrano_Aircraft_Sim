classdef compute_long_trim < handle
    %contains the setup to trim the longitudinal axis

    properties
        Va
        gamma
        mav
        MAV
    end

    methods
        function self = compute_long_trim(Va, gamma, mav, MAV)
            %Setup trim state and mav properties
            self.Va = Va;
            self.gamma = gamma;
            self.mav = mav;
            self.MAV = MAV;
        end

        function norm_long_forces = compute_long_forces(self, x)
            %compute longitudinal forces with an input vector
            % x = [alpha, delta_e, delta_t] and output the weighted root sum 
            % squared of fx, fz, and m
            alpha= x(1);
            delta_e = x(2);
            delta_t = x(3);

            self.mav.set_longitudinal(self.Va, self.gamma, alpha);
            f_and_m = self.mav.forces_moments([delta_e, delta_t, 0,0],self.MAV);
            fx= f_and_m(1);
            fz= f_and_m(3);
            m= f_and_m(5);

            norm_long_forces = norm([fx,fz,m]);
        end
    end
end