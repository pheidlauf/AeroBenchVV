function [ f, Q, R, K_lat ] = f16LatCtrlCostFunc( x, lin_f16, xequil )
%pendLqrCostFun Cost function for Lqr controller performance

% Decouple Linearized F-16 Model: Isolate Lateral States & Actuators

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

% Build in integral tracking on ps, Ny+r
Atilde = [A_lat zeros(3,2);           % beta; p; r;
        [C_lat(4:5,:) zeros(2)]];     % ps; Ny+r
Btilde = [B_lat; D_lat(4:5,:)];

%% Select State & Control Weights & Penalties
% Q: Penalty on State Error
q_beta = exp(x(1));
q_p = exp(x(2));
q_r = exp(x(3));
q_ps_i = exp(x(4));
q_Ny_r_i = exp(x(5));
Q = diag([q_beta q_p q_r q_ps_i q_Ny_r_i]);

% R: Penalty on Control Effort
r_aileron = exp(x(6));
r_rudder = exp(x(7));
R = diag([r_aileron r_rudder]);

% Calculate Lateral Short Period LQR Gains
K_lat = lqr(Atilde, Btilde, Q, R, 0);

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

OLTF=connect(ss_lat,ss_Kpx,controls,-ss_Kint_e,...
    {'e_ps','e_Ny_r'},{'beta','p','r','ps','Ny_r','ail_out',...
    'rud_out'});

%% Frequency Response Based Cost Function:
[Gm_ps,Pm_ps,Wgm_ps,Wpm_ps] = margin(minreal(OLTF(4,1),sqrt(eps),false));
[Gm_Ny,Pm_Ny,Wgm_Ny,Wpm_Ny] = margin(minreal(OLTF(5,2),sqrt(eps),false));

% We want desired OLTF gain crossover frequencies
f = (Wpm_ps-10)^2 + (Wpm_Ny - 3)^2;

end
