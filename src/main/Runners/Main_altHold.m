% Configures the settings for, runs, and saves an F-16 simulation.
% Within this script, the user inputs all initial conditions and autopilot
% instructions for a given simulation scenario. This is the only script
% that must be run to complete a simulation and save the results.
%
% Copyright: GNU General Public License 2017
% <a href="https://github.com/pheidlauf/AeroBenchVV">AeroBenchVV</a>
%
% See also: RUNF16SIM, GETDEFAULTSETTINGS, RENDERANIMATION, RENDERIMAGE,

%% Add all needed paths (if not manually added)
close all; clear; clc;
aerobench_root_data = what('AeroBench');
addpath(fullfile(aerobench_root_data.path,'src','main','utils'));
addAeroBenchPaths(false);

%% Set Initial Conditions
powg = 9;                   % Power

% Default alpha & beta
alphag = deg2rad(0);        % Angle of Attack (rad)
betag = 0;                  % Side slip angle (rad)

% Initial Attitude (for simpleGCAS)
altg = 550;
Vtg = 540;                  % Airspeed (ft/s)
phig = 0;                   % Roll angle from wings level (rad)
thetag = alphag;            % Pitch angle from nose level (rad)
psig = -pi/4;               % Yaw angle from North (rad)     
t_vec = 0:0.01:15;          % Time vector for simulation output

%% Set Flight & Ctrl Limits (for pass-fail conditions)
[flightLimits,ctrlLimits,autopilot] = getDefaultSettings()
autopilot.altitudeHold = true;      % Run GCAS simulation
autopilot.altitude = 500;           % Desired altitude (ft msl)

%% Build Initial Condition Vectors
% state = [VT, alpha, beta, phi, theta, psi, P, Q, R, pn, pe, h, pow]
initialState = [Vtg alphag betag phig thetag psig 0 0 0 0 0 altg powg];
orient = 4;             % Orientation for trim
analysisOn = true;
printOn = true;
plotOn = true;

%% Select Desired F-16 Plant
% Polynomial
[output, passFail] = RunF16Sim(initialState, t_vec, orient, 'morelli',...
    flightLimits, ctrlLimits, autopilot, analysisOn, printOn, plotOn);

%% Save results
% Save output to workspace
data_output = fullfile(pwd,'src/results/SimResults.mat');
image_output = fullfile(pwd,'src/results/output_picture');
animation_output = fullfile(pwd,'src/results/output_animation');

% Generate outputs 
save(data_output,'output','passFail');
renderImage(data_output, image_output);
% renderAnimation(data_output, animation_output);

%% End Run
close all
disp('Script Complete')
