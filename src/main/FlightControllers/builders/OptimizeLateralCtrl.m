close all; clearvars; clc;

[xequil, uequil] = getDefaultEquilibrium();
lin_f16 = getLinF16(xequil, uequil, false);


x_0 = [1, 1, 1, 1, 1, 1, 1];

% Declare anonymous function for additional inputs
costFunc = @(x)f16LatCtrlCostFunc(x, lin_f16, xequil);

options = optimset('PlotFcns',@optimplotfval);
% options = optimset('Display','iter');

x_sol = fminsearch(costFunc, x_0, options);

disp("Optimal F-16 Lateral Control")
[ ~, Q, R, K_lat ] = f16LatCtrlCostFunc( x_sol, lin_f16, xequil );

printmat(K_lat,'LQR Gains','aileron rudder',...
        'beta p r int_e_ps int_e_Ny_r');
    
% Output results to .mat file
output_dir = fullfile('FlightControllers','controlGains');
save(fullfile(output_dir,'optimizedCtrlGains.mat'),'K_lat','-append');
