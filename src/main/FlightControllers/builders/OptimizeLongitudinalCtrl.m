close all; clearvars; clc;

[xequil, uequil] = getDefaultEquilibrium();
lin_f16 = getLinF16(xequil, uequil, false);


x_0 = [1, 1, 1, 1, 1];

% Declare anonymous function for additional inputs
costFunc = @(x)f16LongCtrlCostFunc(x, lin_f16);

options = optimset('PlotFcns',@optimplotfval);
% options = optimset('Display','iter');

x_sol = fminsearch(costFunc, x_0, options);

disp("Optimal F-16 Longitudinal Control")
[ ~, Q, R, K_long ] = f16LongCtrlCostFunc(x_sol, lin_f16);

printmat(K_long,'LQR Gains','elevator',...
    'alpha q int_e_Nz');

% Output results to .mat file
output_dir = fullfile('FlightControllers','controlGains');
save(fullfile(output_dir,'optimizedCtrlGains.mat'),'K_long','-append');