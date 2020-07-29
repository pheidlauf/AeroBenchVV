function [u_deg, x_ctrl] = getInnerLoopControl( ...
    x_f16, u_ref, xequil, uequil, K_lqr)
%GETINNERLOOPCONTROL Given F16 state, equilibrium, controller gains, and
%reference input, calculates the inner loop control signals.
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

% Calculate perturbation from trim state 
x_delta = x_f16 - [xequil; 0; 0; 0]; % in rads

% Reorder states to match controller design
%       [alpha, q, int_e_Nz, beta, p, r, int_e_ps, int_e_Ny_r]
x_ctrl = x_delta([2 8 14 3 7 9 15 16]);

% Initialize control vectors
u_deg = zeros(4,1);     % throt, ele, ail, rud

% Set throttle as directed from output of getOuterLoopCtrl(...)
u_deg(1) = u_ref(4);

% Calculate actuator control using LQR gains
u_deg(2:4) = -K_lqr*x_ctrl; % Full Control

% Add in equilibrium control
u_deg(1:4) = u_deg(1:4) + uequil;

end
