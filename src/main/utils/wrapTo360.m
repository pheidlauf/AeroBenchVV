function [psi_deg_wrapped] = wrapTo360(psi_deg)
%WRAPTO360 Summary of this function goes here
%   Detailed explanation goes here
remainder = rem(psi_deg, 360);
psi_deg_wrapped = remainder + (remainder < 0)*360;
end
