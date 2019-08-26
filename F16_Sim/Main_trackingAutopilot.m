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
addpath(genpath('F16_Model'));
addpath(genpath('utils'));
addpath(genpath('Runner'));
addpath(genpath('Autopilot'));
addpath(genpath('FlightController'));
addpath(genpath('../Visualizers'));

%% Set Initial Conditions
close all; clear; clc;
powg = 9;                   % Power

% Default alpha & beta
alphag = deg2rad(0);        % Angle of Attack (rad)
betag = 0;                  % Side slip angle (rad)

% Initial Altitude/Attitude
altg = 550;                 % Altitude (ft msl)
Vtg = 540;                  % Airspeed (ft/s)
phig = deg2rad(0);          % Roll angle from wings level (rad)
thetag = deg2rad(0);        % Pitch angle from nose level (rad)
psig = pi/4;                % Yaw angle from North (rad)     
t_vec = 0:0.01:30;          % Time vector for simulation output


%% Set Flight & Ctrl Limits (for pass-fail conditions)
[flightLimits,ctrlLimits,autopilot] = getDefaultSettings();
autopilot.mode = 'tracking';        % Use tracking autopilot (EXPERIMENTAL)
autopilot.airspeed = 550;           % ft/s
autopilot.heading = 0;              % North
autopilot.climbRate = 25;           % ft/s

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
data_output = fullfile(pwd,'../Results/SimResults.mat');
image_output = fullfile(pwd,'../Results/output_picture');
animation_output = fullfile(pwd,'../Results/output_animation.mp4');

% Generate outputs 
save(data_output,'output','passFail');
renderImage(data_output, image_output);
renderAnimation(data_output, animation_output);

