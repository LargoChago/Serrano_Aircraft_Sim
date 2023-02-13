% wind Simulation - simulates steady state and wind gusts
%
% mavMatSim 
%     - Beard & McLain, PUP, 2012
%     - Update history:  
%         12/27/2018 - RWB
classdef wind_simulation < handle
   %--------------------------------
    properties
        steady_state
        A
        B
        C
        gust_state
        gust_
        Ts
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = wind_simulation(Ts)
            self.steady_state = [0; 0; 0; 0];
            self.A = [0 0 ; 0 0];
            self.B = [0 ;0];
            self.C = [0 0];
            self.gust_state = 0;
            self.gust_ = [0; 0; 0];
            self.Ts = Ts;
        end
        %---------------------------
        function wind=update(self)
            wind = [self.steady_state; self.gust_];
        end
        %----------------------------
        function self = gust(self)
            w = randn;
            self.gust_state = self.gust_state + self.Ts*(self.A*self.gust_state + self.B*w);
            self.gust_ = self.C*self.gust_state;
        end
    end
end