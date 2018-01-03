function [u_ref,t_maneuver] = getAutopilotCommands(t, x_f16, xequil,...
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
persistent turnDirection;

% Reset persistent flags if (re)starting simulation
if(isempty(man_start) || t <= 0 || resetTrue)
    % Reset maneuver time flags if first func call or needed.
    man_start = -1;
    man_check_1 = -1;
    man_check_2 = -1;
    man_complete = -1;
    turnDirection = 0;
end

% Zero default commands
Nz = 0;
ps = 0;
Ny_r = 0;
throttle = 0;

% Initialize t_maneuver vector to zeros
t_maneuver = zeros(1,2);

%% Implementation of a Simple Ground Collision Avoidance System
% Concept:
%   Roll until wings level (in the shortest direction)
%   When abs(roll rate) < threshold, pull X g's until pitch angle > X deg
if(autopilot.simpleGCAS)
    % Choose threshold values:
    eps_phi = deg2rad(5);   % Max roll angle magnitude before pulling g's
    eps_p = deg2rad(1);     % Max roll rate magnitude before pulling g's
    path_goal = deg2rad(0); % Final desired path angle
    Nz_des = 5;             % Desired maneuver g's
    
    % Pull out important variables for ease of use
    phi = x_f16(4);             % Roll angle    (rad)
    p = x_f16(7);               % Roll rate     (rad/sec)
    theta = x_f16(5);           % Pitch angle   (rad)
    alpha = x_f16(2);           % AoA           (rad)
    % Note: pathAngle = theta - alpha
    
    % Set Proportional-Derivative Control Gains
	K_prop = 4;
    K_der = K_prop*0.3;
    
    % Maneuver starts at t == 0 (boring)
    man_start = 2; 
    
    if(t<man_start)
        % Do nothing
    elseif(man_check_1 < 0)
        % Determine which angle is "level" (0, 180, 360, 720, etc)
        radsFromWingsLevel = round(phi/pi);
        % Until wings are "level" & roll rate is small
        if(abs(phi-pi*radsFromWingsLevel) < eps_phi && abs(p) < eps_p)       
            man_check_1 = t;
        else
            % PD Control until phi == pi*radsFromWingsLevel
            ps = -(phi - pi*radsFromWingsLevel)*K_prop - p*K_der;
        end
    elseif(man_complete < 0)      
        radsFromNoseLevel = round((theta-alpha)/(2*pi));
        if((theta-alpha) - 2*pi*radsFromNoseLevel > path_goal)
            man_complete = t;
        else
            Nz = Nz_des;
        end
    else
        % What to do once recovery is complete
        Nz = 0;
        ps = 0;
        autopilot.steadyLevelFlightHold = true;
    end
    
    % Set t_maneuver states
    t_maneuver(1) = man_start;
    t_maneuver(2) = man_complete;
    t_maneuver(3) = man_check_1;    
end

%% Implementation of a large-scale yaw to heading maneuver
% Note: This is not a heading tracker.

if(autopilot.turnToHeading)
    % Enable speed control
    autopilot.basicSpeedControl       = true;
    % Choose threshold values:
    eps_psi = deg2rad(5);       % Max yaw angle at completion
    eps_r = deg2rad(0.1);       % Max yaw rate magnitude at completion
    eps_phi = deg2rad(1);       % Max roll angle at completion
    eps_p = deg2rad(0.1);       % Max roll rate at completion
    
    psi_goal = deg2rad(0);      % Final desired heading
    
    % Pull out important variables for ease of use
    alpha = x_f16(2);           % AoA           (rad)
    phi = x_f16(4);             % Roll angle    (rad)
    p = x_f16(7);               % Roll rate     (rad/sec)
    psi = x_f16(6);             % Heading angle (rad) zero is north
    r = x_f16(9);               % Heading rate  (rad/s)
    
    % Maneuver starts at t == 0 (boring)
    man_start = 2; 
    maxBankAngle = deg2rad(ctrlLimits.MaxBankDeg);
    autopilot.levelTurnControl = true;
    
    % Set a flag to determine the direction of the turn
    if(turnDirection==0)
        turnDirection = sign(psi_goal - psi);
        % Negative = 
    end
    
    if(t<man_start)
        % Do nothing
        autopilot.steadyLevelFlightHold = true;
    elseif(man_check_1 < 0)
        autopilot.steadyLevelFlightHold = false;
        % phi_goal has not been passed yet
        if(sign(psi_goal - psi) == turnDirection )
            % Set desired turn bank angle
            phi_goal = maxBankAngle*turnDirection;
            % Set Proportional-Derivative Control Gains
            K_prop = 4;
            K_der = K_prop*0.5;
            % PD control for roll rate
            ps = (phi_goal-phi)*K_prop - p*K_der;
        else
            man_check_1 = t;
        end 
    elseif(man_complete < 0)
        % Wings are not level yet
        if(abs(phi) > eps_phi || abs(p) > eps_p)
            K_prop = 4;
            K_der = K_prop*0.5;
            % PD control for roll rate
            ps = -phi*K_prop - p*K_der;
        else
            man_complete = t;
        end
    else
        % What to do once recovery is complete
        autopilot.steadyLevelFlightHold = true;
    end
    
    t_maneuver(1) = man_start;
    t_maneuver(2) = man_complete;
    t_maneuver(3) = man_check_1; 
end


%% Time-Based Commands
% Note: This is just a sample of time-scheduled commands
if(autopilot.timeTriggeredControl)
    maxBank = 60; % degrees 
    if(t<5)
        ps = deg2rad(maxBank/5);
    elseif(t<10)
        ps = deg2rad(-maxBank/5);
    end
end

%% Basic "Smart" Controls

% Proportional control on airspeed
if(autopilot.basicSpeedControl)
    K_vt = 0.25;
    throttle = -K_vt*(x_f16(1) - xequil(1));
end

% Parallel PD control on pitch and roll modes to stay level
if(autopilot.steadyLevelFlightHold)    
    % NOTE: This is not a recovery maneuver
    
    % Pull out important variables for ease of use
    phi = x_f16(4);             % Roll angle    (rad)
    p = x_f16(7);               % Roll rate     (rad/sec)
    theta = x_f16(5);           % Pitch angle   (rad)
    q = x_f16(8);               % Pitch rate    (rad/sec)
    alpha = x_f16(2);           % AoA           (rad)
    
    % Set Proportional-Derivative control gains for roll
	K_prop = 1;
    K_der = K_prop*0.3;
    
    % Determine which angle is "level" (0, 180, 360, 720, etc)
    radsFromWingsLevel = round(phi/pi);
    % PD Control on phi using roll rate 
    ps = -(phi-pi*radsFromWingsLevel)*K_prop - p*K_der;
    
    % Set Proportional-Derivative control gains for pitch
	K_prop2 = 2;
    K_der2 = K_prop*0.3;
    
    % Determine "which" angle is level (0, 360, 720, etc)
    radsFromNoseLevel = round((theta-alpha)/pi);
    % PD Control on theta using Nz
    Nz = -(theta - alpha - pi*radsFromNoseLevel)*K_prop2 - p*K_der2;

    % Note: Could implement PD control on psi using Ny_r (for small errors)
    % Set Proportional-Derivative control gains for pitch
    % 	K_prop3 = 5;
    %     K_der3 = K_prop*0.3;
    % PD Control on psi using Ny_r
    % Ny_r = (psi_goal-psi)*K_prop3 - r*K_der3;
end

% Pull g's to maintain altitude during bank based on geometry/trig
if(autopilot.levelTurnControl)
    % Calculate theta
    phi = x_f16(4);
    
    % Note: This calculation only works for {-pi/2 < phi < pi/2}
    if(phi <= pi/2 && phi > -pi/2)
        Nz = 1/cos(phi) - 1; % Keeps plane at altitude
    else
        Nz = ctrlLimits.NzMax;
        fprintf('At time = %d, phi = %d deg\n',t,rad2deg(phi));
        warning('Singularity at pi/2 < theta < -pi/2. Nz_ref = 0');
    end      
end


%% Emforce G-limits and set maneuver states as needed.
if(Nz > ctrlLimits.NzMax)
    fprintf('Time = %d\n',t);
    fprintf('Nz   = %d g''s\n',Nz);
    fprintf('phi  = %d deg\n',rad2deg(x_f16(4)));
    Nz = ctrlLimits.NzMax;
    warning('Nz_ref exceeds NzMax. Constraining...');   
    fprintf('Nz_trimmed   = %d g''s\n',Nz);
    disp('Note: ODE45 may re-calculate this time step');
end

if(Nz < ctrlLimits.NzMin)
    fprintf('Time = %d\n',t);
    fprintf('Nz   = %d g''s\n',Nz);
    fprintf('phi  = %d deg\n',rad2deg(x_f16(4)));
    Nz = ctrlLimits.NzMin;
    warning('Nz_ref below NzMin. Constraining...');
    fprintf('Nz_trimmed   = %d g''s\n',Nz);
    disp('Note: ODE45 may re-calculate this time step');
end

% Set t_maneuver states
t_maneuver(1) = man_start;
t_maneuver(2) = man_complete;
    
% Create reference vector
u_ref = [Nz; ps; Ny_r; throttle];

end