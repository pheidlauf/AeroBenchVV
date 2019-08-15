% Determines the gains for a F-16 Linear Quadratic Regulator 
% for the Stevens & Lewis mathematical model of the F-16 aircraft at a
% specified trim point. The lateral and longitudinal modes of the F-16 are
% COMBINED. This script builds the COMBINED controller based on specified
% weights on state error and control effort and trim point.
%
% <a href="https://github.com/pheidlauf/AeroBenchVV">AeroBenchVV</a>
% Copyright: GNU General Public License 2017
%
% See also: CONTROLLEDF16

%% Initialize Starting Variables
close all; clear; clc;
addpath(genpath('F16_Model')); % Add necessary sub folder to path

disp('------------------------------------------------------------');
disp('COMBINED F-16 Controller for stability p tracking');
disp('------------------------------------------------------------');
disp('MANUAL INPUTS:');

% SET THESE VALUES MANUALLY
altg = 1000;    % Altitude guess (ft msl)
Vtg = 502;      % Velocity guess (ft/sec)
phig = 0;       % Roll angle from horizontal guess (deg)
thetag = 0;     % Pitch angle guess (deg)
% Note: If a gain-scheduled controller is desired, the above values would
% be varied for each desired trim point.

xguess = [Vtg 0 0 phig thetag 0 0 0 0 0 0 altg 0];

% u = [throttle elevator aileron rudder]
uguess = [.2 0 0 0];

% Orientation for Linearization
% 1:    Wings Level (gamma = 0)
% 2:    Wings Level (gamma <> 0)
% 3:    Constant Altitude Turn
% 4:    Steady Pull Up
orient = 2;
inputs = [xguess(1), xguess(12), 0, 0, 0];
printOn = true;

if(printOn)
    disp('trimmerFun Inputs:');
    printmat(inputs,'inputs',[],'Vt h gamma psidot thetadot')
    fprintf('Orientation Used: ');
    switch orient
        case 1
            disp('Wings Level (gamma = 0)');
        case 2
            disp('Wings Level (gamme <> 0)');
        case 3
            disp('Constant Altitude Turn');
        case 4
            disp('Steady Pull Up');
        otherwise
            error('orient invalid');
    end
    printmat(xguess,'State Guess',[],...
        'Vt alpha beta phi theta psi p q r pn pe alt pow');
    printmat(uguess,'Control Guess',[],...
        'throttle elevator aileron rudder');
end

%% Build Linearized Model
% Get Equilibrium Values
[xequil,uequil] = trimmerFun(xguess, uguess, orient, inputs, printOn);

if(printOn)
    printmat(xequil,'State Equilibrium',[],...
        'Vt alpha beta phi theta psi p q r pn pe alt pow');
    printmat(uequil,'Control Equilibrium',[],...
        'throttle elevator aileron rudder');
end

% Get Linearized Model
lin_f16 = getLinF16(xequil,uequil,printOn);

%% Decouple Linearized F-16 Model: Isolate States & Actuators
% States:   alpha q, beta, p, r

%   x_f16 states
%       x_f16(1)  = air speed, VT                           (ft/s)
%       x_f16(2)  = angle of attack, alpha                  (rad)
%       x_f16(3)  = angle of sideslip, beta                 (rad)
%       x_f16(4)  = roll angle, phi                         (rad)
%       x_f16(5)  = pitch angle, theta                      (rad)
%       x_f16(6)  = yaw angle, psi                          (rad)
%       x_f16(7)  = roll rate, P                            (rad/s)
%       x_f16(8)  = pitch rate, Q                           (rad/s)
%       x_f16(9)  = yaw rate, R                             (rad/s)
%       x_f16(10) = northward horizontal displacement, pn   (ft)
%       x_f16(11) = eastward horizontal displacement, pe    (ft)
%       x_f16(12) = altitude, h                             (ft)
%       x_f16(13) = engine thrust dynamics lag state, pow   (lbs)
%       ----------------------------------------------------------
%       x_f16(14) = Integral of V_error                     (ft)
%       x_f16(15) = Integral of P_error                     (rad)
%       x_f16(16) = Integral of Q_error                     (rad)
%       x_f16(17) = Integral of R_error                     (rad)
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
%       u(5) = Airspeed commanded
%       u(6) = Roll rate commanded
%       u(7) = Pitch rate commanded 
%       u(8) = Yaw rate commanded

% States:   [V, pow, alpha, beta, P, Q, R]
A_full = lin_f16.a([1 13 2 3 7 8 9], [1 13 2 3 7 8 9]); 

% Inputs:   [throttle, elevator, aileron, rudder]
B_full = lin_f16.b([1 13 2 3 7 8 9], :);     

% Outputs:  [V, P, Q, R]   
% C_full = lin_f16.c([5 7 2 8],[1 13 2 3 7 8 9]); 
% D_full = lin_f16.d([5 7 2 8],:);  % Should be all zeros?


% Outputs:  [pow alpha beta V P Q R]
C_full = [lin_f16.a(13,[1 13 2 3 7 8 9]);           % pow
        lin_f16.c([3 9], [1 13 2 3 7 8 9]);         % alpha beta
        lin_f16.c([5 7 2 8],[1 13 2 3 7 8 9])];     % V P Q R
D_full = [lin_f16.b(13,:);                          % pow
    lin_f16.d([3 9],:);                             % alpha beta
    lin_f16.d([5 7 2 8],:)];                        % V P Q R

% Build A & B matrices for integral tracking
Atilde = [A_full zeros(7,4);            % V; pow; alpha; beta; P; Q; R;
        [C_full(4:7,:) zeros(4,4)]];             % V_i; P_i; Q_i; R_i
Btilde = [B_full; 
        D_full(4:7,:)];

%% Select State & Control Weights & Penalties
% Set LQR weights
% Q: Penalty on State Error
q_V = 0;
q_pow = 0;
q_alpha = 0;
q_beta = 0;
q_P = 0;
q_Q = 0;
q_R = 0;
q_V_i = 1;
q_P_i = 100;
q_Q_i = 100;
q_R_i = 1000;
Q = diag([q_V q_pow q_alpha q_beta q_P q_Q q_R q_V_i q_P_i q_Q_i q_R_i]);

% R: Penalty on Control Effort
r_throttle = 1;
r_elevator = 1;
r_aileron = 1;
r_rudder = 1;

R = diag([r_throttle r_elevator r_aileron r_rudder]);
   
%% Calculate Lateral Short Period LQR Gains
N = 0;
K_full = lqr(Atilde,Btilde,Q,R,N);

% K:                V pow alpha beta P Q R int_e_V int_e_P int_e_Q int_e_R 
%      throttle:
%      elevator:
%       aileron:
%        rudder:
% printmat(K_full,'LQR Gains','throttle elevator aileron rudder',...
%         'V pow alpha beta P Q R int_e_V int_e_P int_e_Q int_e_R');

%% Build State Space Models of Lateral F-16    
% Uncontrolled SS model of combined dynamics
ss_full=ss(A_full,B_full,...
    [C_full; zeros(4,7)],[D_full; eye(4)]);                 
ss_full.u={'throttle','elevator','aileron','rudder'};
ss_full.StateName = {'V','pow','alpha','beta','P','Q','R'};
ss_full.y={'V','pow','alpha','beta','P','Q','R',...
    'throt_out','ele_out','ail_out','rud_out'};   

% Proportional Compensator (using beta, r)
ss_Kpx=ss(zeros(4,4),zeros(4,7), ...
    zeros(4,4), K_full(:,1:7));          % Pass through proportional gains
ss_Kpx.u={'V', 'pow', 'alpha', 'beta', 'P', 'Q', 'R'}; % Inputs
ss_Kpx.y='Kp_x';                    % Outputs:  throt, ele, ail, rud cmds

controls=sumblk('%u = -Kp_x - Ki_x',{'throttle','elevator',...
    'aileron','rudder'});

% Integral Compensator (using ps)
ss_Kint_e=ss(zeros(4),eye(4),K_full(:,8:11),0);   % Integral control on e_ps, beta
ss_Kint_e.u={'e_V','e_P','e_Q','e_R'};            % Inputs:
ss_Kint_e.StateName = {'int_e_V','int_e_P','int_e_Q','int_e_R'};
ss_Kint_e.y='Ki_x';         % Outputs:  Integral controls

% Negative error between p_cmd and stability p
error=sumblk('%error = -%cmd + %x',...
    {'e_V','e_P','e_Q','e_R'},...
    {'V_cmd','P_cmd','Q_cmd','R_cmd'},...
    {'V','P','Q','R'});  

sys_cl_full=connect(ss_full,ss_Kpx,controls,ss_Kint_e,error,...
    {'V_cmd','P_cmd','Q_cmd','R_cmd'},...
    {'V','P','Q','R','throt_out','ele_out','ail_out','rud_out'});
sys_cl_full

OLTF=connect(ss_full,ss_Kpx,controls,-ss_Kint_e,...
    {'e_V','e_P','e_Q','e_R'},...
    {'V','P','Q','R','throt_out','ele_out','ail_out','rud_out'});

%% Compare Margins, Damping Ratios, and Pole Frequencies
figure(1);
margin(OLTF(1,1))

figure(2);
margin(OLTF(2,2))

figure(3);
margin(OLTF(3,3))

figure(4);
margin(OLTF(4,4))

% Output results to .mat file
save('combinedCtrlData.mat','K_full','sys_cl_full','xequil','uequil');

% View Damping Ratio & Frequency of Poles
damp(eig(sys_cl_full.a))
[Wn,zeta] = damp(eig(sys_cl_full.a));
% Note: Wn is the natural frequency (aim for 3 rad/s)
%       zeta is the damping ratio   (aim for 0.707)

printmat(K_full,'LQR Gains','throttle elevator aileron rudder',...
        'V pow alpha beta P Q R int_e_V int_e_P int_e_Q int_e_R');