function [u_ref,t_maneuver] = getTrackingAutopilotCmds(t, x_f16, xequil,...
    uequil, flightLimits, ctrlLimits, autopilot, resetTrue)
% Given the time, f16 state, and trim condition, returns desired Nz, ps, 
% Ny+r, and throttle settings.
%
%   Function Call:
%       [u_ref] = getAutopilotCommands(t, x_f16, xequil, uequil, ...
%           flightLimits, ctrlLimits, autopilot, resetTrue)
%
%   Inputs:
%       t       - time of step (sec)
%       x_f16   - State vector (f16 state) (16x1)
%       xequil  - trim conditions (13x1)
%       uequil  - equilibrium control (4x1)
%       flightLimits - struct of flight limits
%       ctrlLimits - struct of control limits
%       autopilot - struct of autopilot settings:
%           simpleGCAS              - "Roll & Pull" to level flight
%           basicSpeedControl       - Proportional control on airspeed
%           steadyLevelFlightHold   - PD control on pitch & roll
%           levelTurnControl        - Pulls g's to stay level when banked
%           turnToHeading          - Bank, G, unbank to heading
%           timeTriggeredControl    - Program your own maneuvers f(t)
%       resetTrue  - OPTIONAL: If 1, reset persistent variables          
%
%   Outputs:
%       u_ref(1)    = Down force, Nz_ref                    (g's)
%       u_ref(2)    = Stability roll rate, ps_ref           (rad/sec)
%       u_ref(3)    = Side force, Ny_r_ref                  (g's)
%       u_ref(4)    = Throttle Setting                      (ft/sec)
%       t_maneuver(1) =     Maneuver start time             (sec)
%       t_maneuver(2) =     Maneuver end time               (sec)
%       t_maneuver(3:n) =   Checkpoint time                 (sec)
%
%   x_f16 States:
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
%       x_f16(14) = Integral of Nz error, Nz_e_i             (g's)
%       x_f16(15) = Integral of Ps_error, Ps_e_i             (rad/sec)
%       x_f16(16) = Integral of Ny_error, Ny_e_i             (g's)
%
%   Notes:
%       This function is where you program the autopilot. Commanded
%       Nz, ps, Ny+r, and throttle settings can be hard coded, time
%       triggered, or state triggered. 
% 
%       Persistent variables are used to track any states that are not
%       included in the F-16 state vector. These are needed for
%       integration, tracking time flags, maneuver states, etc. If you
%       modify this function to add your own, make sure to declare the
%       persistent variables and included them in the "reset" block.
%
% <a href="https://github.com/pheidlauf/AeroBenchVV">AeroBenchVV</a>
% Copyright: GNU General Public License 2017
%
% See also: GETDEFAULTSETTINGS, CONTROLLEDF16, PERSISTENT

%% Initialize Flags, Controls, Vectors
% Run top level function if called without arguments
if(nargin==0)
    Main;
    return;
end

% If "resetTrue" not designated, set to false
if(nargin < 8)
    resetTrue = 0;
end

% Declare persistent flags and variables
persistent man_start man_complete man_check_1 man_check_2;

% Reset persistent flags if (re)starting simulation
if(isempty(man_start) || t <= 0 || resetTrue)
    % Reset maneuver time flags if first func call or needed.
    man_start = -1;
    man_check_1 = -1;
    man_check_2 = -1;
    man_complete = -1;
end

% Zero default commands
Nz = 0;
ps = 0;
Ny_r = 0;
throttle = 0;

% Initialize t_maneuver vector to zeros
t_maneuver = zeros(1,2);

%% General algorithm for tracking:
% TODO: Implement fancier solution
man_start = 1;

if(t<1)
    % Do nothing
else
    Nz_alt = trackAltitude(x_f16,autopilot.altitude);
    Nz_roll = getNzForLevelTurnOL(x_f16);
    Nz = Nz_alt + Nz_roll;
    psi_cmd = getPsiToTrackHeading(x_f16,autopilot.heading);
    ps = trackRollAngle(x_f16,psi_cmd);
    throttle = trackAirspeed(x_f16,autopilot.airspeed);
end

%% Function Definitions
    function [Nz] = trackAltitude(x_f16, h_cmd)
        % Given state & des altitude, calculate PD control using Nz
        
        % Pull out important variables for ease of use
        Vt = x_f16(1);              % Airpeed       (ft/sec)
        alpha = x_f16(2);           % AoA           (rad)
        beta = x_f16(3);            % Sideslip      (rad)
        phi = x_f16(4);             % Roll anle     (rad)
        theta = x_f16(5);           % Pitch angle   (rad)
        h = x_f16(12);              % Altitude      (feet)
        
        % Proportional-Derivative Control
        h_error = h_cmd - h;
        sinGamma = (cos(alpha)*sin(theta)- ...
            sin(alpha)*cos(theta)*cos(phi))*cos(beta) - ...
            (cos(theta)*sin(phi))*sin(beta);
        h_dot = Vt*sinGamma;    % Calculated, not differentiated
        
        % Gains for Control
        k_alt = 0.005;
        k_h_dot = 0.02;
        
        % Calculate Nz command
        Nz = k_alt*h_error - k_h_dot*h_dot;
    end

    function [Nz] = getNzForLevelTurnOL(x_f16)
        % Pull g's to maintain altitude during bank based on trig
        
        % Calculate theta
        phi = x_f16(4);
        if(phi <= pi/2 && phi > -pi/2) % If cos(phi) ~= 0, basically
            Nz = 1/cos(phi) - 1; % Keeps plane at altitude
        else
            Nz = 0;
            % TODO: Put a warning here?
        end
        
    end

    function [ps] = trackRollAngle(x_f16,phi_cmd)
        % PD control on roll angle using stability roll rate
        
        % Pull out important variables for ease of use
        phi = x_f16(4);             % Roll angle    (rad)
        p = x_f16(7);               % Roll rate     (rad/sec)
        
        % Set Proportional-Derivative Control Gains
        K_prop = 0.5;
        K_der = K_prop*0.5;
        
        % Calculate PD control
        ps = (phi_cmd-phi)*K_prop - p*K_der;
    end

    function [throttle] = trackAirspeed(x_f16,Vt_cmd)
        % Proportional control on airspeed using throttle
        K_vt = 0.25;
        throttle = K_vt*(Vt_cmd - x_f16(1));
    end

    function [phi_cmd] = getPsiToTrackHeading(x_f16,psi_cmd)
        % PD Control on heading angle using phi_cmd as control
        
        % Pull out important variables for ease of use
        psi = x_f16(6);             % Heading angle (rad) zero is north
        r = x_f16(9);               % Heading rate  (rad/s)
        
        % Set Proportional-Derivative Control Gains
        K_prop = 4;
%         K_der = K_prop*0.5;
        K_der = 0; % Proportional only for now
        
        % TODO: Implement PID or PI control
        
        % Calculate PD control
        phi_cmd = (psi_cmd - psi)*K_prop - r*K_der;
        
        % Bound to acceptable bank angles:
        maxBankRad = deg2rad(60);
        phi_cmd = min(max(phi_cmd,-maxBankRad),maxBankRad);
    end

%% Check for maneuver completion
% TODO: Put in checks for this.
% man_start = 0;
if(man_complete < 0)
    if(abs(autopilot.airspeed - x_f16(1)) < 5)
        if(abs(autopilot.heading - x_f16(6)) < 1)
            if(abs(autopilot.altitude - x_f16(12)) < 10)
                man_complete = t;
            end
        end
    end
end
% man_complete = man_start + 1; % Just make it pass for now

%% Combine/condition controls
% Set t_maneuver states
t_maneuver(1) = man_start;
t_maneuver(2) = man_complete;
    
% Create reference vector
u_ref = [Nz; ps; Ny_r; throttle];

end