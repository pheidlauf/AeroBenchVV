function [ initialState, x_f16_0, waypoints, t_end, GCAS_starts_on] = getInitialConditions(scenario)
%Returns an F-16 initialState given a scenario specification

%% Common settings between all scenarios
t_end = 150;

powg = 9;                   % Power
% Default alpha & beta
alphag = deg2rad(2.1215);   % Trim Angle of Attack (rad)
betag = 0;                  % Side slip angle (rad)

% Select target waypoints
e_pt = 1000;
n_pt = 3000;
h_pt = 4000;

GCAS_starts_on = 0;

waypoints = [...
    e_pt, n_pt, h_pt;
    e_pt+2000, n_pt+ 5000, h_pt-100;
    e_pt-2000, n_pt+15000, h_pt-250;
    e_pt- 500, n_pt+25000, h_pt;
    ];

switch scenario
    case 'GCAS'
        GCAS_starts_on = 1;
        t_end = 3.51;
        altg = 1000;
        Vtg = 540;
        phig = -pi/8;          % Roll angle from wings level (rad)
        thetag = (-pi/2)*0.3;       % Pitch angle from nose level (rad)
        psig = 0;               % Yaw angle from North (rad)
        
        waypoints = [...
            0, 20000, 1500;
            ];
    case 'GCAS_inverted'
        GCAS_starts_on = 1;
        altg = 1000;
        Vtg = 540;
        phig = -pi * 0.9;          % Roll angle from wings level (rad)
        thetag = (-pi/2)*0.01;       % Pitch angle from nose level (rad)
        psig = 0;               % Yaw angle from North (rad)
        
        waypoints = [...
            0, 20000, 1500;
            ];    
        
        t_end = 5;
    case 'waypoint'
        altg = 3800;
        Vtg = 540;
        phig = 0;                   % Roll angle from wings level (rad)
        thetag = 0;                 % Pitch angle from nose level (rad)
        psig = pi/8;                % Yaw angle from North (rad)  
        
        t_end = 70;
    case 'waypoint_trigger_GCAS'
        altg = 1500;
        Vtg = 540;
        phig = 0;                   % Roll angle from wings level (rad)
        thetag = 0;                 % Pitch angle from nose level (rad)
        psig = pi/4;                % Yaw angle from North (rad)
        % Select target waypoints
        e_pt = 3000;
        n_pt = 3000;
        h_pt = 2000;
        
        waypoints = [...
            e_pt, n_pt, h_pt;
            e_pt+2000, n_pt+ 5000, h_pt-500;
            e_pt+1000, n_pt+10000, h_pt-750;
            e_pt- 500, n_pt+15000, h_pt-1250;
            ];

    case 'u_turn'
        altg = 1500;
        Vtg = 540;
        phig = 0;                   % Roll angle from wings level (rad)
        thetag = 0;                 % Pitch angle from nose level (rad)
        psig = 0;                % Yaw angle from North (rad)
        
        % Select target waypoints        
        waypoints = [...
            -5000, -7500, altg;
            -15000, -7500, altg;
            -20000, 0, altg+500;
            ];

    otherwise
        error('AEROBENCH:utils:getInitiailConditions', ...
            'Scenario not defined for: %s', scenario);
end

%% Build Initial Condition Vectors
% state = [VT, alpha, beta, phi, theta, psi, P, Q, R, pn, pe, h, pow]
initialState = [Vtg alphag betag phig thetag psig 0 0 0 0 0 altg powg]';
x_f16_0 = [initialState; 0; 0; 0]; % Append integrator states

end
