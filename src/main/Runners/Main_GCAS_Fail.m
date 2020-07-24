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
alphag = deg2rad(2.1215);   % Trim Angle of Attack (rad)
betag = 0;                  % Side slip angle (rad)

% Initial Attitude (for simpleGCAS)
altg = 3500;
Vtg = 560;                  % Pass at Vtg = 540;    Fail at Vtg = 550?
phig = (pi/2)*0.5;          % Roll angle from wings level (rad)
thetag = (-pi/2)*0.8;       % Pitch angle from nose level (rad)
psig = -pi/4;               % Yaw angle from North (rad)     
t_vec = 0:0.01:15;          % Time vector for simulation output

%% Set Flight & Ctrl Limits (for pass-fail conditions)
[flightLimits,ctrlLimits,autopilot] = getDefaultSettings()
ctrlLimits.ThrottleMax = 0.7;       % Limit to Mil power (no afterburner)
autopilot.simpleGCAS = true;     % Run GCAS simulation

%% Build Initial Condition Vectors
% state = [VT, alpha, beta, phi, theta, psi, P, Q, R, pn, pe, h, pow]
initialState = [Vtg alphag betag phig thetag psig 0 0 0 0 0 altg powg];
orient = 4;             % Orientation for trim
analysisOn = true;
printOn = true;
plotOn = true;

%% Select Desired F-16 Plant
% Table Lookup
% [output, passFail] = RunF16Sim(initialState, t_vec, orient, 'stevens',...
%     flightLimits, ctrlLimits, autopilot, analysisOn, printOn, plotOn);

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
