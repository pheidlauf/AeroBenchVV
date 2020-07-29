function [psi_rad_wrapped] = wrapToPi(psi_rad)
%WRAPTOPI Summary of this function goes here
%   Detailed explanation goes here
remainder = rem(psi_rad, 2*pi);
psi_rad_wrapped = remainder ...
    + (remainder < -pi)*2*pi ...
    - (remainder > pi)*2*pi;
end
