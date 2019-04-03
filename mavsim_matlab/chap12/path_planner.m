% path planner for mavsim_matlab
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Last updated:  
%         4/2/2019 - RWB
classdef path_planner < handle
   %--------------------------------
    properties
        waypoints
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = path_planner
            % waypoints definition
            addpath('../message_types'); 
            self.waypoints = msg_waypoints(); 
        end
        %------methods-----------
        function waypoints = update(self, map, state)
            % this flag is set for one time step to signal a redraw in the
            % viewer
            %planner_flag = 1;  % return simple waypoint path
            planner_flag = 2;  % return dubins waypoint path
            %planner_flag = 3;  % plan path through city using straight-line RRT
            %planner_flag = 4;  % plan path through city using dubins RRT
            if planner_flag == 1
                self.waypoints.type = 'fillet';
                self.waypoints.num_waypoints = 4;
                Va = 25;
                self.waypoints.ned(:, 1:self.waypoints.num_waypoints) = ...
                    [0,   0,   -100;...
                     1000, 0,   -100;...
                     0,   1000, -100;...
                     1000, 1000, -100]';
                self.waypoints.airspeed(1:self.waypoints.num_waypoints) = ...
                    [Va, Va, Va, Va];
            elseif planner_flag == 2
                self.waypoints.type = 'dubins';
                self.waypoints.num_waypoints = 4;
                Va = 25;
                self.waypoints.ned(:, 1:self.waypoints.num_waypoints) = ...
                    [0,   0,   -100;...
                     1000, 0,   -100;...
                     0,   1000, -100;...
                     1000, 1000, -100]';
                self.waypoints.airspeed(1:self.waypoints.num_waypoints) = ...
                    [Va, Va, Va, Va];
                self.waypoints.course = ...
                    [0, 45*pi/180, 45*pi/180, -135*pi/180];
            elseif planner_flag == 3
%                                % current configuration
%               wpp_start = [pn, pe, -h, chi, PLAN.Va0];
%               % desired end waypoint
%               if norm([pn; pe; -h]-[map.width; map.width; -h])<map.width/2
%                   wpp_end = [0, 0, -h, chi, PLAN.Va0];
%               else
%                   wpp_end = [map.width, map.width, -h, chi, PLAN.Va0];
%               end
%               waypoints = planRRT(wpp_start, wpp_end, map);
%               num_waypoints = size(waypoints,1);
%               wpp = [];
%               for i=1:num_waypoints
%                   wpp = [...
%                             wpp;...
%                             waypoints(i,1), waypoints(i,2), waypoints(i,3), waypoints(i,4), PLAN.Va0;...
%                         ];
%               end
            elseif planner_flag == 4
%                                % current configuration
%               wpp_start = [pn, pe, -h, chi, PLAN.Va0];
%               % desired end waypoint
%               if norm([pn; pe; -h]-[map.width; map.width; -h])<map.width/2
%                   wpp_end = [0, 0, -h, 0, PLAN.Va0];
%               else
%                   wpp_end = [map.width, map.width, -h, 0, PLAN.Va0];
%               end
%               waypoints = planRRTDubins(wpp_start, wpp_end, PLAN.R_min, map);
%               num_waypoints = size(waypoints,1);
%               wpp = [];
%               for i=1:num_waypoints
%                   wpp = [...
%                             wpp;...
%                             waypoints(i,1), waypoints(i,2), waypoints(i,3), waypoints(i,4), PLAN.Va0;...
%                         ];
%               end
% 
            else
                disp('Error in Path Planner: Undefined planner type.')
            end
            % return the planned waypoints
            waypoints = self.waypoints;
        end
    end
end