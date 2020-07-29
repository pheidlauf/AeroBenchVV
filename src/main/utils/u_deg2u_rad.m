function [u_rad] = u_deg2u_rad(u_deg)
%U_DEG2U_RAD Converts 4x1 u_deg to u_rad

% Convert all degree values to radians for output
u_rad(1,1) = u_deg(1);
u_rad(2:4,1) = deg2rad(u_deg(2:4));

end
