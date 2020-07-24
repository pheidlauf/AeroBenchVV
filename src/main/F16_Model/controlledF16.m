function [ xd, u, Nz, ps, Ny_r ] = controlledF16( t, x_f16,... 
    xequil, uequil, K_lqr, F16_model, lin_f16, flightLimits, ctrlLimits,...
    autopilot)
% CONTROLLEDF16 Returns the LQR-controlled F-16 state derivatives and more
%   Function Call:
%       [ xd, u, Nz, ps, Ny_r ] = CONTROLLEDF16( t, x_f16,... 
%           xequil, uequil, K_lqr, F16_model, lin_f16, ...
%           flightLimits, ctrlLimits, autopilot)
%
%   Inputs:
%       t       - time of step
%       x_f16   - F16 state vector (13+3 x 1) in rads, rads/sec, etc.
%       xequil  - trim states (13x1) in rads
%       uequil  - trim control (4x1) in degs
%       K_lqr   - linear quadratic regulator gain matrix (3x8)
%       F16_model - Optional string defining which function to use to
%           calculate state derivatives: {'morelli','stevens','linear'}.
%           Defaults to 'morelli'.
%       lin_f16 - Linear state space model of F-16 (req. only for 'linear')
%       ctrlLimits - struct of control limits
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
%
%   OUTPUT:
%       xd (16x1)   - units in [rad, rad/s, ft/s, ft, g's, etc...]
%
%
% <a href="https://github.com/pheidlauf/AeroBenchVV">AeroBenchVV</a>
% Copyright: GNU General Public License 2017
%
% See also: RUNF16SIM, GETAUTOPILOTCOMMANDS, BUILDLATERALLQRCTRL,
% BUILDLONGITUDINALLQRCTRL, ODE45
                  
% Get Reference Control Vector (commanded Nz, ps, Ny+r, throttle)
if(strcmp(autopilot.mode,'tracking'))
    u_ref = getTrackingAutopilotCmds(t, x_f16, xequil, uequil, ...
        flightLimits, ctrlLimits, autopilot, 0); % in g's & rads/sec
else
    u_ref = getAutopilotCommands(t, x_f16, xequil, uequil, ...
        flightLimits, ctrlLimits, autopilot, 0); % in g's & rads/sec
end

[u_deg, x_ctrl] = getInnerLoopControl(x_f16, u_ref, xequil, uequil, K_lqr);          

%% Limit controls to saturation limits (except for 'linear')
if(~strcmp(F16_model,'linear') && ~strcmp(F16_model,'fullLinear'))
    u_deg = applyCtrlSatLimits(u_deg, ctrlLimits); 
end

%% Generate xd using user-defined method:

%   Note: Control vector (u) for subF16 is in units of degrees
if(strcmp(F16_model,'stevens'))
    % Uses lookup tables / interpolation to get state derivatives
    [xd, Nz, Ny] = subf16_stevens(x_f16(1:13), u_deg);
elseif(strcmp(F16_model,'morelli'))
    % Uses polynomial equations to get state derivatives
    [xd, Nz, Ny] = subf16_morelli(x_f16(1:13), u_deg); 
elseif(strcmp(F16_model,'linear'))
    % Uses linear approximations to get state derivatives
    xd = lin_f16.a*x_f16(1:13)      + lin_f16.b*u_deg;
    % Uses polynomial equations to get down & side forces
    [~, Nz, Ny] = subf16_morelli(x_f16(1:13), u_deg); 
elseif(strcmp(F16_model,'fullLinear'))
    warning('fullLinear not yet implemented correctly');
    % Uses linear approximations to get state derivatives
    xd = lin_f16.a*x_f16(1:13)      + lin_f16.b*u_deg;
    
    % Uses linear approximations to get down & side forces
    % TODO: Account for shift from CG to pilot??
    Nz = lin_f16.c(1,:)*x_f16(1:13) + lin_f16.d(1,:)*u_deg;
    Ny = lin_f16.c(6,:)*x_f16(1:13) + lin_f16.d(6,:)*u_deg;
else
   error('Unsupported F16_model requested') 
end

% Calculate (Nonlinear) stability axis roll rate:
if(strcmp(F16_model,'linear') || strcmp(F16_model,'fullLinear'))
    % Linear (Approx):          ps = p            +     r*alpha
    ps = x_ctrl(5)*x_ctrl(1);
else
    % Nonlinear (Actual):       ps = p*cos(alpha) +     r*sin(alpha)
    ps = x_ctrl(5)*cos(x_ctrl(1)) + x_ctrl(6)*sin(x_ctrl(1)); 
end

% Calculate (side force + yaw rate) term
Ny_r = Ny + x_ctrl(6);

% Add tracked error states to xd for integration
xd(14:16) = [Nz - u_ref(1); ps - u_ref(2); Ny_r - u_ref(3)]; 

% Build full control signal (in rad) including reference signals.
u = zeros(7,1);
u(1:4,1) = u_deg2u_rad(u_deg);
u(5:7,1) = u_ref(1:3);

end
