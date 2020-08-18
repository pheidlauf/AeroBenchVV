function [ initialState,x_f16_0,waypoints] = getInitialConditions(scenario)
%Returns an F-16 initialState given a scenario specification

%% Common settings between all scenarios
powg = 9;                   % Power
% Default alpha & beta
alphag = deg2rad(2.1215);   % Trim Angle of Attack (rad)
betag = 0;                  % Side slip angle (rad)

% Select target waypoints
e_pt = 3000;
n_pt = 3000;
h_pt = 4000;

waypoints = [...
    e_pt, n_pt, h_pt;
    e_pt+2000, n_pt+ 5000, h_pt-500;
    e_pt+1000, n_pt+10000, h_pt-750;
    e_pt- 500, n_pt+15000, h_pt-1250;
    ];

switch scenario
    case 'GCAS'
        altg = 1200;
        Vtg = 540;
        phig = -pi/3;          % Roll angle from wings level (rad)
        thetag = (-pi/2)*0.3;       % Pitch angle from nose level (rad)
        psig = 0;               % Yaw angle from North (rad)
        
        waypoints = [...
            0, 20000, 1500;
            ];
    case 'GCAS_inverted'
        altg = 1100;
        Vtg = 540;
        phig = -pi;          % Roll angle from wings level (rad)
        thetag = (-pi/2)*0.01;       % Pitch angle from nose level (rad)
        psig = 0;               % Yaw angle from North (rad)
        
        waypoints = [...
            0, 20000, 1500;
            ];    
    case 'default'
        altg = 4000;
        Vtg = 540;
        phig = (pi/2)*0.5;          % Roll angle from wings level (rad)
        thetag = (-pi/2)*0.1;       % Pitch angle from nose level (rad)
        psig = pi/4;                % Yaw angle from North (rad)
    case 'waypoint'
        altg = 3500;
        Vtg = 540;
        phig = 0;                   % Roll angle from wings level (rad)
        thetag = 0;                 % Pitch angle from nose level (rad)
        psig = pi/4;                % Yaw angle from North (rad)
    case 'optimizer'
        altg = 1000;
        Vtg = 540;
        phig = 0;                   % Roll angle from wings level (rad)
        thetag = 0;                 % Pitch angle from nose level (rad)
        psig = 0;                   % Yaw angle from North (rad)    
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
            -20000, -2000, altg+1000;
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
