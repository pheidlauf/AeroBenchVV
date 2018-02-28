% Determines the gains for a decoupled F-16 Linear Quadratic Regulator 
% for the Stevens & Lewis mathematical model of the F-16 aircraft at a
% specified trim point. The lateral and longitudinal modes of the F-16 are
% decoupled. This script builds the lateral controller based on specified
% weights on state error and control effort and trim point.
%
% <a href="https://github.com/pheidlauf/AeroBenchVV">AeroBenchVV</a>
% Copyright: GNU General Public License 2017
%
% See also: BUILDLATERALLQRCTRL, CONTROLLEDF16

%% Initialize Starting Variables
close all; clear; clc;
addpath(genpath('F16_Model')); % Add necessary sub folder to path

disp('------------------------------------------------------------');
disp('Lateral F-16 Controller for stability p tracking');
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

%% Decouple Linearized F-16 Model: Isolate Lateral States & Actuators
% States:   beta, p, r
A_lat = lin_f16.a([3 7 9], [3 7 9]); 

% Inputs:   aileron, rudder
B_lat = lin_f16.b([3 7 9], [3 4]);     

% Outputs:  beta, p, r, ps, Ny+r   
C_lat = [lin_f16.c([9 7 8],[3 7 9]);
    (lin_f16.c(7,[3 7 9]) + lin_f16.c(8,[3 7 9])*xequil(2));
    lin_f16.c(6,[3 7 9]) + [0 0 1]];  
D_lat = [zeros(4,2); 
    lin_f16.d(6,[3 4])];

Atilde = [A_lat zeros(3,2);           % beta; p; r;
        [C_lat(4:5,:) zeros(2)]];     % ps; Ny+r
Btilde = [B_lat; D_lat(4:5,:)];

%% Select State & Control Weights & Penalties
% Set LQR weights
% Q: Penalty on State Error
q_beta = 0;
q_p = 0;
q_r = 0;
q_ps_i = 1200;
q_Ny_r_i = 3000;
Q = diag([q_beta q_p q_r q_ps_i q_Ny_r_i]);

% R: Penalty on Control Effort
r_aileron = 1;
r_rudder = 1;
R = diag([r_aileron r_rudder]);
   
%% Calculate Lateral Short Period LQR Gains
N = 0;
K_lat = lqr(Atilde,Btilde,Q,R,N);

% K:                beta   p    r    ps_i    Ny
%       aileron:
%        rudder:
% printmat(K_lat,'LQR Gains','aileron rudder',...
%         'beta p r int_e_ps int_e_Ny_r');

%% Build State Space Models of Lateral F-16    
% Uncontrolled SS model of lateral directional dynamics
ss_lat=ss(A_lat,B_lat,...
    [C_lat; zeros(2,3)],[D_lat; eye(2)]);                 
ss_lat.u={'aileron','rudder'};              % Controls: aileron, rudder
ss_lat.StateName = {'beta','p','r'};
ss_lat.y={'beta','p','r','ps','Ny_r','ail_out',...
    'rud_out'};   % Outputs: beta, p, r, ps, ail_cmd, rud_cmd

% Proportional Compensator (using beta, r)
ss_Kpx=ss(zeros(2,2),zeros(2,3), ...
    zeros(2,2), K_lat(:,1:3));          % Pass through proportional gains
ss_Kpx.u={'beta', 'p', 'r'}; % Inputs:   beta, r (bpr)
ss_Kpx.y='Kp_x';                    % Outputs:  Controls for ail, rud

controls=sumblk('%u = -Kp_x - Ki_x',{'aileron','rudder'});

% Integral Compensator (using ps)
ss_Kint_e=ss(zeros(2),eye(2),K_lat(:,end-1:end),0);   % Integral control on eps, beta
ss_Kint_e.u={'e_ps','e_Ny_r'};         % Inputs:   2x1 p beta
ss_Kint_e.StateName = {'int_e_ps','int_e_Ny_r'};
ss_Kint_e.y='Ki_x';         % Outputs:  Integral controls

% Negative error between p_cmd and stability p
error=sumblk('%error = -%cmd + %x',...
    {'e_ps','e_Ny_r'},...
    {'ps_cmd','Ny_r_cmd'},...
    {'ps','Ny_r'});  

sys_cl_lat=connect(ss_lat,ss_Kpx,controls,ss_Kint_e,error,...
    {'ps_cmd','Ny_r_cmd'},...
    {'beta','p','r','ps','Ny_r','e_ps','e_Ny_r','ail_out',...
    'rud_out'});
sys_cl_lat

OLTF=connect(ss_lat,ss_Kpx,controls,-ss_Kint_e,...
    {'e_ps','e_Ny_r'},{'beta','p','r','ps','Ny_r','ail_out',...
    'rud_out'});

%% Compare Margins, Damping Ratios, and Pole Frequencies
figure(1);
margin(OLTF(4,1))

figure(2);
margin(OLTF(5,2))

% Output results to .mat file
save('lateralCtrlData.mat','K_lat','sys_cl_lat','xequil','uequil');

% View Damping Ratio & Frequency of Poles
damp(eig(sys_cl_lat.a))
[Wn,zeta] = damp(eig(sys_cl_lat.a));
% Note: Wn is the natural frequency (aim for 3 rad/s)
%       zeta is the damping ratio   (aim for 0.707)

% opt = stepDataOptions('StepAmplitude',1);
% figure(3);
% step(sys_cl_lat(:,:),10,opt)
printmat(K_lat,'LQR Gains','aileron rudder',...
        'beta p r int_e_ps int_e_Ny_r');

%% Citations
% Stevens, Brian L., Frank L. Lewis, and Eric N. Johnson. 
% Aircraft control and simulation: dynamics, controls design, and 
% autonomous systems. John Wiley & Sons, 2015.
%
% Liebst, Bradley S.,
% AFIT 
