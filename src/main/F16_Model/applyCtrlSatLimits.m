function [u_deg, isSaturated] = applyCtrlSatLimits(u_deg_in, ...
    ctrlLimits, warnOn)
%APPLYCTRLSATLIMITS Limits the control commands to ctrl saturation limits

%Declare function as extrinsic
coder.extrinsic('sprintf', 'warning');

if nargin < 3
    warnOn = true;
end

u_deg = zeros(size(u_deg_in));

% Limit throttle from 0 to 1
u_deg(1) = max(min(u_deg_in(1),ctrlLimits.throttleMax),...
    ctrlLimits.throttleMin);

% Limit elevator from -25 to 25 deg
u_deg(2) = max(min(u_deg_in(2),ctrlLimits.elevatorMax),...
    ctrlLimits.elevatorMin);

% Limit aileron from -21.5 to 21.5 deg
u_deg(3) = max(min(u_deg_in(3),ctrlLimits.aileronMax),...
    ctrlLimits.aileronMin);

% Limit rudder from -30 to 30 deg
u_deg(4) = max(min(u_deg_in(4),ctrlLimits.rudderMax),...
    ctrlLimits.rudderMin);

% If u_deg is constrained, set flag to true
isSaturated = any(u_deg_in ~= u_deg);

if (isSaturated && warnOn)
    warn_message = sprintf('u_deg_in: [%f %f %f %f]', u_deg_in);
    warning('AEROBENCH:CTRL:saturated', warn_message);
end

end
