%
% PACKAGE_SETUP (package_setup.m)
%
% flypath3d package check & configuration file
% Last updated: 2016-08-09
%
% SYNTAX:
%
%    package_setup
%

clear all;
close all;
clc;

% --- flypath.m ---
filename = strcat(cd,'/flypath.m'); 
if exist(filename,'file') ~= 2
   warning('file flypath.m not found!'); 
end;

% --- new_object.m ---
filename = strcat(cd,'/new_object.m'); 
if exist(filename,'file') ~= 2
   warning('file new_object.m not found!');
end;

% --- model_import.m ---
filename = strcat(cd,'/model_import.m'); 
if exist(filename,'file') ~= 2
   warning('file model_import.m not found!');
end;

% --- model_show.m ---
filename = strcat(cd,'/model_show.m'); 
if exist(filename,'file') ~= 2
   warning('file model_show.m not found!');
end;

% --- update path list ---
addpath(cd);
newpath = strcat(cd,'/models');
addpath(newpath);