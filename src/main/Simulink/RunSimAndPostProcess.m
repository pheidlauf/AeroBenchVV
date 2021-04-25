%% This script does everything you need to set up, run, and view a sim.
close all; clearvars; clc;

% Run SimConfig to set required workspace variables
SimConfig;

% Update Simulink diagram to account for code changes
sys = 'AeroBenchSim_2019a';
load_system(sys)
set_param(sys, 'SimulationCommand', 'update')
    
try
    sim(strcat(sys,'.slx'))
catch exception
    exception
end

% Plot results
%handleSimResults;
graphOutput