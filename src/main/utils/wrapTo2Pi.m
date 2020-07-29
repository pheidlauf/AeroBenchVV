function [psi_rad_wrapped] = wrapTo2Pi(psi_rad)
%WRAPTO360 Summary of this function goes here
%   Detailed explanation goes here
remainder = rem(psi_rad, 2*pi);
psi_rad_wrapped = remainder + (remainder < 0)*2*pi;
end
