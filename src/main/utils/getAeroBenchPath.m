function [aerobench_path] = getAeroBenchPath()
%GETAEROBENCHPATH Returns the filesystem path of AeroBench

if ~exist('./Autopilot', 'dir')
    error('Autopilot folder not found. Did you set src/main to be the current working directory?');
end

aerobench_path = '../..';

end
