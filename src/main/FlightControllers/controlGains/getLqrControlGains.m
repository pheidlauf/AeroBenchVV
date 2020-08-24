function [K_lqr, K_lat, K_long] = getLqrControlGains(gainSet)

if nargin == 0
    disp('Default gainSet used: optimized')
    gainSet = 'optimized';
end

switch gainSet
    case 'optimized'
        filename = 'optimizedCtrlGains.mat';
    case 'manuallyTuned'
        filename = 'manualCtrlGains.mat';
    case 'old'
        filename = 'oldCtrlGains.mat';
    otherwise
        disp('Requested LQR Ctrl Gains not defined')
        error('AEROBENCH:getLqrControlGains:badArgs', ...
            'Requested gainSet: %s not available', gainSet)
end

gain_dir = fullfile('FlightControllers','controlGains');

load(fullfile(gain_dir,filename), 'K_lat', 'K_long');

K_lqr = blkdiag(K_long, K_lat);

end
