function [ flightLimits, ctrlLimits, autopilot ] = getDefaultSettings(  )
%Returns structs with default settings needed to run the F16 simulations
%
%   Function Calls:
%       [ flightLimits, ctrlLimits, autopilot ] = getDefaultSettings()
%   Inputs:
%       N/A
%
%   Outputs:
%       flightLimits    - struct of flight limits used to determine
%       manevuer failure or success
%       ctrlLimits      - struct of control limits for the F-16 actuators
%       autopilot       - struct of autopilot settings that determines
%       which autonomous maneuvers are to be performed
%
%   Comments:
%   This function creates and returns the necessary structs to call
%   RUNF16SIM. Some variables should not be modified and are commented as
%   such in the function. This function can easily be modified (apart from
%   hard-coded model limits) to meet your specific default needs.
%
% <a href="https://github.com/pheidlauf/AeroBenchVV">AeroBenchVV</a>
% Copyright: GNU General Public License 2017
%
% See also: GETAUTOPILOTCOMMANDS, CONTROLLEDF16, RUNF16SIM

%% Set Flight Limits (for pass-fail conditions)
flightLimits = struct([]);
flightLimits(1).altitudeMin = 0;        % ft AGL
flightLimits.altitudeMax = 10000;       % ft AGL
flightLimits.maneuverTime = 15;         % seconds
flightLimits.NzMax = 9;                 % G's
flightLimits.NzMin = -2;                % G's
flightLimits.psMaxAccelDeg = 500;       % deg/s/s

% Note: Alpha, Beta, Vt are hard-coded limits. DO NOT CHANGE
flightLimits.vMin = 300;                % ft/s
flightLimits.vMax = 900;                % ft/s
flightLimits.alphaMinDeg = -10;         % deg
flightLimits.alphaMaxDeg = 45;          % deg
flightLimits.betaMaxDeg = 30;           % deg

%% Set Control Limits
ctrlLimits = struct([]);
ctrlLimits(1).ThrottleMax = 1;          % Afterburner on for throttle > 0.7
ctrlLimits.ThrottleMin = 0; 
ctrlLimits.ElevatorMaxDeg = 25;
ctrlLimits.ElevatorMinDeg = -25;
ctrlLimits.AileronMaxDeg = 21.5;
ctrlLimits.AileronMinDeg = -21.5;
ctrlLimits.RudderMaxDeg = 30;
ctrlLimits.RudderMinDeg = -30;
ctrlLimits.MaxBankDeg = 60;             % For turning maneuvers
ctrlLimits.NzMax = 6;
ctrlLimits.NzMin = -1;

%% Set Initial Autopilot Modes
autopilot = struct([]);
autopilot(1).title = 'Default Simulation';
autopilot.mode = 'default';         % Set to 'tracking' for improvement
autopilot.basicSpeedControl     = true;     % Proportional control on airspeed
autopilot.steadyLevelFlightHold = false;    % PD control on pitch & roll
autopilot.levelTurnControl      = false;    % Pulls g's to stay level when banked
autopilot.simpleGCAS            = false;    % "Roll & Pull" to level flight
autopilot.turnToHeading         = false;    % Bank, G, unbank to heading
autopilot.timeTriggeredControl  = false;    % Program your own maneuvers f(t)
autopilot.altitudeHold          = false;    % Proportional altitude control
autopilot.altitude              = 500;      % ft msl 
autopilot.trackHeading          = false;    % Track a heading (P-D?)
autopilot.heading               = 0;        % Track to north on default
autopilot.airspeed              = 500;      % ft/s

end