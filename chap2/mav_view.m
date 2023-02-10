classdef mav_view < handle
    %
    %    Create spacecraft animation
    %
    %--------------------------------
    properties
        body_handle
    	Vertices
    	Faces
    	facecolors
        plot_initialized
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = mav_view
            self.body_handle = [];
            [self.Vertices, self.Faces, self.facecolors] = self.define_mav();
            self.plot_initialized = 0;           
        end
        %---------------------------
        function self=update(self, state)
            if self.plot_initialized==0
                figure(1); clf;
                self=self.drawBody(state.pn, state.pe, -state.h,...
                                   state.phi, state.theta, state.psi);
                title('Spacecraft')
                xlabel('East')
                ylabel('North')
                zlabel('-Down')
                view(32,47)  % set the vieew angle for figure
                axis([-10,10,-10,10,-10,10]);
                hold on
                grid on
                self.plot_initialized = 1;
            else
                self=self.drawBody(state.pn, state.pe, -state.h,... 
                                   state.phi, state.theta, state.psi);

            end
        end
        %---------------------------
        function self = drawBody(self, pn, pe, pd, phi, theta, psi)
            Vertices = self.rotate(self.Vertices, phi, theta, psi);   % rotate rigid body  
            Vertices = self.translate(Vertices, pn, pe, pd);     % translate after rotation
            % transform vertices from NED to ENU (for matlab rendering)
            R = [...
                0, 1, 0;...
                1, 0, 0;...
                0, 0, -1;...
                ];
            Vertices = R*Vertices;
            if isempty(self.body_handle)
                self.body_handle = patch('Vertices', Vertices', 'Faces', self.Faces,...
                                             'FaceVertexCData',self.facecolors,...
                                             'FaceColor','flat');
            else
                set(self.body_handle,'Vertices',Vertices','Faces',self.Faces);
                drawnow
            end
        end 
        %---------------------------
        function pts=rotate(self, pts, phi, theta, psi)
            % define rotation matrix (right handed)
            R_roll = [...
                        1, 0, 0;...
                        0, cos(phi), sin(phi);...
                        0, -sin(phi), cos(phi)];
            R_pitch = [...
                        cos(theta), 0, -sin(theta);...
                        0, 1, 0;...
                        sin(theta), 0, cos(theta)];
            R_yaw = [...
                        cos(psi), sin(psi), 0;...
                        -sin(psi), cos(psi), 0;...
                        0, 0, 1];
            R = R_roll*R_pitch*R_yaw;   % inertial to body
            R = R';  % body to inertial
            % rotate vertices
            pts = R*pts;
        end
        %---------------------------
        % translate vertices by pn, pe, pd
        function pts = translate(self, pts, pn, pe, pd)
            pts = pts + repmat([pn;pe;pd],1,size(pts,2));
        end
        %---------------------------
        function [V, F, colors] = define_mav(self)
            % Define the vertices (physical location of vertices)
            V = [...
-2.5 0 -5; %point 1
-1.75 -.25 -5.25; %point 2
-1.75 .25 -5.25; %point 3
-1.75 .25 -4.75; %point 4
-1.75 -.25 -4.75; %point 5
3 0 -5; %point 6
-.75 -2.5 -5; %point 7
.75 -2.5 -5; %point 8
.75 2.5 -5; %point 9
-.75 2.5 -5; %point 10
2.25 -1.25 -5; %point 11
3 -1.25 -5; %point 12
3 1.25 -5; %point 13
2.25 1.25 -5; %point 14
2.25 0 -5; %point 15
3 0 -5.75; %point 16

            ]';

            % define faces as a list of vertices numbered above
            F = [...
7 8 9 ; %front wing
7 9 10; %front wing
11 12 13; %back wing
11 13 14; %back wing
6 15 16; %tail wing
1 2 3;
1 3 4;
1 4 5;
1 2 5; %nose
2 3 6;
3 4 6;
4 5 6;
2 5 6; %body
                    ];

            % define colors for each face    
            myred = [1, 0, 0];
            mygreen = [0, 1, 0];
            myblue = [0, 0, 1];
            myyellow = [1, 1, 0];
            mycyan = [0, 1, 1];

            colors = [...
                myyellow;...
                myyellow;... % front wing
                myblue;...   
                myblue;...   % backwing
                mycyan;...   % tail wing
                myred;...  
                myred;...  
                myred;...  
                myred;...  %nose
                mygreen;...  
                mygreen;... 
                mygreen;... 
                mygreen;... %body
                ];
            
        end
    end
end