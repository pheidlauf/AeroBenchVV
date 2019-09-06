%% SimConfig
% This script is called by the initFcn callback of AeroBenchSim
close all; clear; clc;

% Add required dirs to MATLAB Path
addpath(genpath('F16_Model'));
addpath(genpath('utils'));




[ flightLimits, ctrlLimits, autopilot ] = getDefaultSettings();