function [x,u] = conf16(x,u,const)
%
% function [x,u] = conf16(x,u,const)
% purpose : apply constraints to the variable 'x'
% called from : ' clf16 ' (the f16 cost function )
% calls to : none
% author C.S. Clark 24 Jan 97
% inputs:
%   
%         x(2) = angle of attack, alpha  (rad)
%         x(3) = angle of sideslip, beta (rad)
%         x(4) = roll angle, phi  (rad)
%         x(5) = pitch angle, theta  (rad)
%         x(7) = roll rate, P  (rad/sec)
%         x(9) = yaw rate, R  (rad/sec)
%         gamma = flight path angle at which to trim ( rad )
%         coord = flag ( 1= 'yes' , 0 = 'no') to trim in coordinated turn
%                 NOT USED... SET IT EQUAL TO ZERO
%         stab  = flag ( 1= 'yes' , 0 = 'no') for stability axis roll
%                 Note: if neither flag is 'yes' then body axis roll assumed
% outputs:
%         x(4) = constrained roll angle, phi  (rad)
%         x(5) = constrained pitch angle, theta  (rad)
%         x(9) = constrained yaw rate, R  (rad/sec)
%
%

radgam = const(1);
singam = const(2);
gamm = asin(singam);
rr = const(3);
pr = const(4);
tr = const(5);
phi = const(6);
cphi = const(7);
sphi = const(8);
thetadot = const(9);
coord = const(10);
stab = const(11);
orient = const(12);

%
% Steady Level Flight
%
if orient==1
   x(4) = phi;          % Phi
   x(5) = x(2);         % Theta
   x(7) = rr;           % Roll Rate
   x(8) = pr;           % Pitch Rate
   x(9) = 0.0;          % Yaw Rate
end

%
% Steady Climb
%
if orient==2
   x(4) = phi;          % Phi
   x(5) = x(2)+radgam;  % Theta
   x(7) = rr;           % Roll Rate
   x(8) = pr;           % Pitch Rate
   x(9) = 0.0;          % Yaw Rate
end

%
% orient=3 implies coordinated turn
%
if orient==3
x(7)=-tr*sin(x(5));           % Roll Rate
x(8)=tr*cos(x(5))*sin(x(4));  % Pitch Rate
x(9)=tr*cos(x(5))*cos(x(4));  % Yaw Rate
end

%
% Pitch Pull Up
%
if orient==4
   x(5) = x(2);    % Theta = alpha
   x(4) = phi;          % Phi
   x(7) = rr;           % Roll Rate
   x(8) = thetadot;     % Pitch Rate
   x(9) = 0.0;          % Yaw Rate   
end


x(13)=tgear(u(1));
