%% SimConfig
% This script is called by the initFcn callback of AeroBenchSim
%
%   x_f16 states
%       x_f16(1)  = air speed, VT                            (ft/s)
%       x_f16(2)  = angle of attack, alpha                   (rad)
%       x_f16(3)  = angle of sideslip, beta                  (rad)
%       x_f16(4)  = roll angle, phi                          (rad)
%       x_f16(5)  = pitch angle, theta                       (rad)
%       x_f16(6)  = yaw angle, psi                           (rad)
%       x_f16(7)  = roll rate, P                             (rad/s)
%       x_f16(8)  = pitch rate, Q                            (rad/s)
%       x_f16(9)  = yaw rate, R                              (rad/s)
%       x_f16(10) = northward horizontal displacement, pn    (ft)
%       x_f16(11) = eastward horizontal displacement, pe     (ft)
%       x_f16(12) = altitude, h                              (ft)
%       x_f16(13) = engine thrust dynamics lag state, pow    (lbs)
%       ----------------------------------------------------------
%       x_f16(14) = Integral of Nz error, int_e_Nz           (g's)
%       x_f16(15) = Integral of Ps_error, int_e_ps           (rad/sec)
%       x_f16(16) = Integral of Ny_error, int_e_Ny_r         (g's)
%
%   Nonlinear f16 controls:
%       u(1) = throttle command     (0 to 1)
%       u(2) = elevator command     (deg)
%       u(3) = aileron command      (deg)
%       u(4) = rudder command       (deg)
%
%   LQR Controls:
%       u(1) = throttle command     (0 to 1)
%       u(2) = elevator command     (deg)
%       u(3) = aileron command      (deg)
%       u(4) = rudder command       (deg)
%       ------------------------------------
%       u(5) = Nz commanded         (g's)
%       u(6) = roll rate commanded  (deg/s)
%       u(7) = Ny+r commanded       (g's)  

%%%%%%%%%%%%
% to init sim and graph, run: 
% SimConfig; sim('AeroBenchSim_2019a'); graphOutput;

% Note: sim_path is set by the initFcn of AeroBenchSim
aerobench_path = addAeroBenchPaths(false);

%% INPUTS HERE:

%% Toggles
warnOn = false;

%% Set Initial Conditions
scenario = 'u_turn';
[initialState, x_f16_0, waypoints, t_end] = getInitialConditions(scenario);

%% Set Flight & Ctrl Limits (for pass-fail conditions)
% [flightLimits,ctrlLimits,~] = getDefaultSettings();
% ctrlLimits.ThrottleMax = 0.7;   % Limit to Mil power (no afterburner)

% Using F16 class
flightLimits = F16.get_default_flight_limits();
ctrlLimits = F16.get_default_ctrl_srfc_limits();
ctrlLimits.ThrottleMax = 0.7;   % Limit to Mil power (no afterburner)
simF16 = F16(flightLimits, ctrlLimits);

%% Equivalent to RunF16Sim.m
% Hard-coded equilibrium conditions at LQR design point.
[xequil,uequil] = getDefaultEquilibrium();

% Combine decoupled controllers
K_lqr = getLqrControlGains('old');

%% Set GCAS configuration
GCAS_config = GCAS.get_default_GCAS_config();

%% Set Waypoint Following configuration
WF_config = WaypointFollower.get_default_WF_config();

%% GENERIC SIM CONFIG
SIM_config.auto_stop = true;    % Causes sim to end after waypoint nav
SIM_config.auto_stop_delay = 5; % Seconds
SIM_config.t_sample = 1/30;