function [ f, Q, R, K_long ] = f16LongCtrlCostFunc( x, lin_f16 )
%pendLqrCostFun Cost function for Lqr controller performance

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
q_alpha = exp(x(1));
q_q = exp(x(2));
q_Nz = exp(x(3));
Q = diag([q_alpha q_q q_Nz]);

% R: Penalty on Control Effort
r_elevator = exp(x(4));
R = r_elevator;

% Calculate Lateral Short Period LQR Gains
K_long = lqr(Atilde, Btilde, Q, R, 0);

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

OLTF = connect(ss_plant,ss_Kx,de_cmd,-ss_Kint_Nz,...
    'e_Nz',{'alpha','q','Nz','ele_out'});

%% Frequency Response Based Cost Function:
[Gm_Nz,Pm_Nz,Wgm_Nz,Wpm_Nz] = margin(minreal(OLTF(3,1),sqrt(eps),false));

% We want desired OLTF gain crossover frequencies
f = (Wpm_Nz - 3)^2;

end
