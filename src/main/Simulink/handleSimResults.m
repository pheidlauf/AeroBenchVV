% This runs when simulink stops

disp('Simulink Run Complete');
disp('Begin Post Processing');

%% Save results
% Save output to workspace
data_output = fullfile(aerobench_path,'results/SimulinkResults.mat');
image_output = fullfile(aerobench_path,'results/output_picture');
animation_output = fullfile(aerobench_path,'results/output_animation');
hud_output = fullfile(aerobench_path,'results/HUD_animation');

% Generate outputs 
save(data_output,'x_f16_out','t_out');

%% Create plots


% save mp4 / avi animation
fprintf("saving mp4 / avi animation in results folder\n")
renderSimpleAnimation(animation_output, t_out, transpose(x_f16_out), ...
    flightLimits);

% save image
fprintf("saving image in results folder\n")
renderSimpleImage(image_output, t_out, transpose(x_f16_out), ...
    flightLimits, waypoints, WF_config)

plotAircraftStates(t_out, x_f16_out, u_il_out, u_ol_out, u_ol_ref_out);

plotStateflowModes(t_out, autopilot_mode_out, GCAS_mode_out, ...
    WF_mode_out, WF_iter_out, waypoints)

plotWaypointCmds(t_out, x_f16_out, psi_cmd_out, ...
    WF_mode_out, WF_iter_out, waypoints)

% save 3d animation
%fprintf("saving 3d HUD animiation in results folder\n")
%render3dHudAnimation(hud_output, t_out, x_f16_out, u_ol_out, ...
%    waypoints, WF_config, GCAS_config)

disp('Post Processing Complete');
