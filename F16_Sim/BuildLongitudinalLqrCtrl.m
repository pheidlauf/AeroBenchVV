% Determines the gains for a decoupled F-16 Linear Quadratic Regulator 
% for the Stevens & Lewis mathematical model of the F-16 aircraft at a
% specified trim point. The lateral and longitudinal modes of the F-16 are
% decoupled. This script builds the longitudinal controller based on 
% specified weights on state error and control effort and trim point.
%
% <a href="https://github.com/pheidlauf/AeroBenchVV">AeroBenchVV</a>
% Copyright: GNU General Public License 2017
%
% See also: BUILDLATERALLQRCTRL, CONTROLLEDF16

%% Initialize Starting Variables
close all; clear; clc;
addpath(genpath('F16_Model')); % Add necessary sub folder to path

disp('------------------------------------------------------------');
disp('Longitudinal F-16 Controller for Nz tracking');
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
orient = 4;
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
[xequil,uequil] = trimmerFun(xguess,uguess,orient,inputs,printOn);

if(printOn)
    printmat(xequil,'State Equilibrium (ft, rads, etc.)',[],...
        'Vt alpha beta phi theta psi p q r pn pe alt pow');
    printmat(uequil,'Control Equilibrium (% & degs)',[],...
        'throttle elevator aileron rudder');
end

% Get Linearized Model
lin_f16 = getLinF16(xequil,uequil,printOn);

%% Decouple Linearized F-16 Model: Isolate Longitudinal States & Actuators
A_long = lin_f16.a([2 8],[2 8]);        % States:   alpha, q
B_long = lin_f16.b([2 8],2);            % Inputs:   elevator
C_long = lin_f16.c([3 2 1],[2 8]);      % Outputs:  alpha; q; Nz
D_long = lin_f16.d([3 2 1],2);          % Inputs:   elevator

Atilde = [A_long zeros(2,1);            % States:   alpha; q;
    C_long(3,:) 0];                     %           Nz
Btilde = [B_long;
    D_long(3,:)];                       % Inputs:   elevator


%% Select State & Control Weights & Penalties
% Set LQR weights
% Q: Penalty on State Error
q_alpha = 1000;
q_q = 0;
q_Nz = 1500;

% R: Penalty on Control Effort
r_elevator = 1;

%% Calculate Longitudinal Short Period LQR Gains
K_long = lqr(Atilde,Btilde,diag([q_alpha q_q q_Nz]), r_elevator);
printmat(K_long,'LQR Gains','elevator',...
    'alpha q int_e_Nz');

%% Build State Space Models of Longitudinal F-16    
% My ss_asp (with elevator commanded as an output state)
ss_plant=ss(A_long,B_long,...
    [C_long; 0 0],[D_long; deg2rad(1)]);       % Uncontrolled SS model
ss_plant.StateName={'alpha','q'};
ss_plant.u='elevator';                      % Inputs:   elevator
ss_plant.y={'alpha','q','Nz','ele_out'};    % Outputs:  alpha, q, Nz, ele_out

ss_Kx=ss(0,[0 0],0,K_long(1:2));     % Pass through gains for alpha, q
ss_Kx.u={'alpha','q'};          % Inputs:   alpha, q
ss_Kx.y='Kx';                   % Outputs:  Elevator control from alpha, q

de_cmd=sumblk('elevator=-Kx-Kint_e_Nz');

% Integral control on Nz error
ss_Kint_Nz=ss(0,1,K_long(3),0);
ss_Kint_Nz.u='e_Nz';                 % Inputs:   Nz error
ss_Kint_Nz.y={'Kint_e_Nz'};          % Outputs:  K_i*int_e_Nz
ss_Kint_Nz.StateName = 'int_e_Nz';

error=sumblk('e_Nz=-Nz_cmd+Nz');

sys_cl_long=connect(ss_plant,ss_Kx,de_cmd,ss_Kint_Nz,error,...
    'Nz_cmd',{'alpha','q','Nz','e_Nz','ele_out'});

OLTF = connect(ss_plant,ss_Kx,de_cmd,-ss_Kint_Nz,...
    'e_Nz',{'alpha','q','Nz','ele_out'});
sys_cl_long

%% Compare Margins, Damping Ratios, and Pole Frequencies
figure(1);
margin(OLTF(3,1))

% View Damping Ratio & Frequency of Poles
damp(eig(sys_cl_long.a))
% [Wn,zeta] = damp(eig(sys_cl_long.a))
% Note: Wn is the natural frequency (aim for 3 rad/s)
%       zeta is the damping ratio   (aim for 0.707)

figure(2);
% step(sys_cl_sp)
opt = stepDataOptions('StepAmplitude',1);
step(sys_cl_long,3,opt)

save('longitudinalCtrlData.mat','K_long','sys_cl_long','xequil','uequil');
printmat(K_long,'LQR Gains','elevator',...
    'alpha q int_e_Nz');

%% Citations
% Stevens, Brian L., Frank L. Lewis, and Eric N. Johnson.
% Aircraft control and simulation: dynamics, controls design, and
% autonomous systems. John Wiley & Sons, 2015.
%
% Liebst, Bradley S.,
% AFIT