function [waypoints] = get_waypoint_samples(scenario)
%GET_X_F16_SAMPLES Summary of this function goes here
%   Detailed explanation goes here

if nargin == 0
    scenario = 'default';
end

% Some default values
e_pos = -500;
n_pos = 1000;
alt = 500;
        
switch scenario
    case 'default'
        waypoints = [e_pos, n_pos, alt];
    case 'two_waypoints'
        waypoints = [e_pos, n_pos, alt;
          e_pos, n_pos + 500, alt + 100;  
        ];
    otherwise
        error('AEROBENCH:tests:helpers', ...
            'Waypoint scenario not defined for: %s', scenario);
end
