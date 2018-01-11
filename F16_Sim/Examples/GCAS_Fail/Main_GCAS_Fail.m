% Configures the settings for, runs, and saves an F-16 simulation.
% Within this script, the user inputs all initial conditions and autopilot
% instructions for a given simulation scenario. This is the only function
% that must be to complete a simulation and save the results. The scripts
% MakeAnimation and MakePicture can then be run and operate from the
% results in the workspace.
%
% Copyright: GNU General Public License 2017
% <a href="https://github.com/pheidlauf/AeroBenchVV">AeroBenchVV</a>
%
% See also: RUNF16SIM, GETDEFAULTSETTINGS, MAKEANIMATION, MAKEPICTURE,

%% Set Initial Conditions
close all; clear; clc;
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

% Initial Attitude (for turnToHeading)
% altg = 500;
% Vtg = 502;
% phig = 0;
% thetag = alphag;
% psig = deg2rad(-135);       % Yaw angle;
% t_vec = 0:0.01:30; 

% Initial Attitude (for steadyLevelFlightHold)
% phig = deg2rad(10);                 % Roll angle;
% thetag = alphag - deg2rad(10);      % Pitch angle;
% psig = deg2rad(-15);                % Yaw angle;
% t_vec = 0:0.01:15;    


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
save('SimResults.mat','output','passFail');

% Generate Renderings using a modified version of flypath3d
MakePicture;
MakeAnimation;