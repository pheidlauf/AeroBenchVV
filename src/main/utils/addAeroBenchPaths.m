function aerobench_path = addAeroBenchPaths(printOn)
%addAeroBenchPath Finds and adds AeroBenchPaths

%% Set args if not provided
if nargin<1
    printOn = true;
elseif nargin>1
    error('Incorrect args provided')
end

%% Find AeroBench root path
aerobench_path = getAeroBenchPath();
src_path = fullfile(aerobench_path,'src');

% Define AeroBench subfolders to add
pathsToAdd = {aerobench_path;
    src_path;
    fullfile(src_path,'main');
    fullfile(src_path,'main','utils');
    %fullfile(src_path,'main','Runners');
    fullfile(src_path,'main','FlightControllers');
    fullfile(src_path,'main','Autopilot');
    fullfile(src_path,'main','viewers');
    fullfile(src_path,'main','F16_Model');
    fullfile(src_path,'main','F16_Model','NonlinearModel');
    fullfile(src_path,'main','Simulink');
    fullfile(src_path,'tests','helpers');
    %fullfile(src_path,'tests','resources');
    %fullfile('results');
    };
    
%% Add all found paths
for i=1:size(pathsToAdd,1)
    addpath(cell2mat(pathsToAdd(i,:)))
end

if(printOn)
	fprintf('\nAdded AeroBench paths:\n');
    disp(listAddedPaths('AeroBench'))
end

end
