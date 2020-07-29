function rotDCM = attitude2dcm(yaw, pitch, roll)
%attitude2dcm Create direction cosine matrix from rotation angles.
%   Kerianne Hobbs 3 December 2019    
%   This is a simplified substitute for the angle2dcm function in the
%   Aerospace Blockset that just does a 3-2-1 aircraft convention.
%    
%   rotDCM = attitude2dcm(yaw, pitch, roll) calculates the direction 
%   cosine matrix, rotDCM for aircraft attitude for a given yaw, pitch, 
%   and roll set of rotation angles.
%
%    Input angles should be in radians
%  
%     Examples:
%  
%     Determine the direction cosine matrix from rotation angles:
%        yaw = 0.7854; 
%        pitch = 0.1; 
%        roll = 0;
%        rotDCM = attitude2dcm(yaw, pitch, roll)
% Note: there is a singularity when theta = +/- pi/2
phi = roll;
theta = pitch;
psi = yaw;
R3 = [1 0 0; 0 cos(phi) sin(phi);0 -sin(phi) cos(phi)];
R2 = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
R1 = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
rotDCM = R3*R2*R1;