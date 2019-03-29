% msg_waypoints
%   - message type for path planning
%
% mavsim_matlab 
%     - Beard & McLain, PUP, 2012
%     - Update history:  
%         3/20/2019 - RWB
classdef msg_waypoints
   %--------------------------------
    properties
        flag_waypoints_changed
        flag_manager_requests_waypoints
        type
        max_waypoints
        num_waypoints
        ned
        airspeed
        course
        cost
        parent_idx
        flag_connect_to_goal
    end
    %--------------------------------
    methods
        %------constructor-----------
        function self = msg_waypoints()
            % the first two flags are used for interacting with the path
            % planner
            %
            % flag to indicate waypoints recently changed (set by planner)
            self.flag_waypoints_changed = 1;  
            % flag to indicate that the waypoint manager needs new
            % waypoints (set by manager)
            self.flag_manager_requests_waypoints = 1;
            
            % type of waypoint following:
            %   - straight line following
            %   - fillets between straight lines
            %   - follow dubins paths
            self.type = 'straight_line';
            %self.type = 'fillet';
            %self.type = 'dubins';
            % maximum number of waypoints.  This is used to pre-allocate
            %   memory to improve efficiency
            self.max_waypoints = 100;
            % current number of valid waypoints in memory
            self.num_waypoints = 0;
            % [n, e, d] - coordinates of waypoints
            self.ned = inf*ones(3, self.max_waypoints);
            % the airspeed that is commanded along the waypoints
            self.airspeed = inf*ones(1, self.max_waypoints);
            % the desired course at each waypoint (used only for Dubins
            % paths)
            self.course = inf*ones(1, self.max_waypoints);
            
            % these last three variables are used by the path planner
            % running cost at each node
            self.cost = inf*ones(1, self.max_waypoints);
            % index of the parent to the node
            self.parent_idx = inf*ones(1, self.max_waypoints);   
            % can this node connect to the goal?  1==connected, 0==not
            % connected
            self.flag_connect_to_goal = inf*ones(1, self.max_waypoints);  
        end
    end
end