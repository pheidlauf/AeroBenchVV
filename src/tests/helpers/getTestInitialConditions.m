function [x_f16, xequil, uequil, K_lqr, F16_model, lin_f16, ...
    flightLimits, ctrlLimits, autopilot] = ...
    getTestInitialConditions()
%GETSIMINITIALCONDITIONS Returns initial conditions needed to run a sim

printOn = false;

%% Set Initial Conditions
powg = 9;                   % Power

% Default alpha & beta
alphag = deg2rad(2.1215);   % Trim Angle of Attack (rad)
betag = 0;                  % Side slip angle (rad)

% Initial Attitude (for simpleGCAS)
altg = 4000;
Vtg = 540;                  % Pass at Vtg = 540;    Fail at Vtg = 550;
phig = (pi/2)*0.5;          % Roll angle from wings level (rad)
thetag = (-pi/2)*0.8;       % Pitch angle from nose level (rad)
psig = -pi/4;               % Yaw angle from North (rad)     

% Initial rates (to make Nz, Ny_r look interesting)
Pg = 0.02;
Qg = -0.5;
Rg = -0.07;

%% Set Flight & Ctrl Limits (for pass-fail conditions)
[flightLimits,ctrlLimits,autopilot] = getDefaultSettings();
ctrlLimits.ThrottleMax = 0.7;   % Limit to Mil power (no afterburner)
autopilot.simpleGCAS = true;    % Run GCAS simulation

%% Build Initial Condition Vectors
% state = [VT, alpha, beta, phi, theta, psi, P, Q, R, pn, pe, h, pow]
initialState = [Vtg alphag betag phig thetag psig Pg Qg Rg 0 0 altg powg];

%% Hardcoded equilibrium points
xequil = [502 0.0389 0 0 0.0389 0 0 0 0 0 0 1000 9.0567]';
uequil = [0.1395 -0.7496 0 0]';

%% Hard coded LQR gain matrix
K_long = [-156.8802  -31.0370  -38.7298];

% Optimally tuned (experimental)
K_lat = [37.84483    -25.40956     -6.82876   -332.88343    -17.15997;...
    -23.91233      5.69968    -21.63431     64.49490    -88.36203];

% Combine decoupled controllers
K_lqr = blkdiag(K_long,K_lat);

F16_model='morelli';

lin_f16 = getLinF16(xequil,uequil,printOn);

% ODE initial conditions
x_f16 = [initialState'; 0; 0; 0];
end

