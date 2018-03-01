% Configures the settings for, runs, and saves an F-16 simulation.
% Within this script, the user inputs all initial conditions and autopilot
% instructions for a given simulation scenario. This is the only function
% that must be to complete a simulation and save the results. The scripts
% MakeAnimation and MakePicture can then be run and operate from the
% results in the workspace.
%
% Note: This version of MAIN is a barebones sample of what is required to
% run a single simulation. Results are printed to console, but not plotted.
%
% <a href="https://github.com/pheidlauf/AeroBenchVV">AeroBenchVV</a>
% Copyright: GNU General Public License 2017
%
% See also: RUNF16SIM, GETDEFAULTSETTINGS, MAKEANIMATION, MAKEPICTURE,

%% Set Initial Conditions
close all; clear; clc;
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
save('SimResults.mat','output','passFail');
disp('Script Complete');