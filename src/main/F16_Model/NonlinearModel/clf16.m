function [r,x,u]= clf16(s,x,u,const)
%
% function [r,x,u]= clf16(s,x,u,const)
% purpose : this is the F16 cost function used in the trim program
% called from : function 'fminsa' ( simplex minimization routine )
% calls to functions: 'tgear' , 'subf16'
%  author: C. S. Clark 
%  date : 24 Jan 97
%  
%
% step 1: load the control values recieved from fminsa for relay to
%          'subf16'
% step 2: call 'tgear' to get throttle setting
% step 3: call 'subf16' ( the f16 airframe and engine ) to see what accelerations
%         those control settings produce
% step 4: call' conf16' to apply constraints to those settings
% step 5: evaluate the cost as a function of those 'xd' (accelerations) values to see
%         if the trim condition has been met.
% Comment: that cost will be evaluated by 'fminsa' to see if trim condition has been met.
% 
%         x(1) = air speed, VT    (ft/sec)
%         x(2) = angle of attack, alpha  (rad)
%         x(3) = angle of sideslip, beta (rad)
%         x(4) = roll angle, phi  (rad)
%         x(5) = pitch angle, theta  (rad)
%         x(6) = yaw angle, psi  (rad)
%         x(7) = roll rate, P  (rad/sec)
%         x(8) = pitch rate, Q  (rad/sec)
%         x(9) = yaw rate, R  (rad/sec)
%         x(10) = northward horizontal displacement, pn  (feet)
%         x(11) = eastward horizontal displacement, pe  (feet)
%         x(12) = altitude, h  (feet)
%         x(13) = engine thrust dynamics lag state, pow  
%
%         u(1) = throttle command  0.0 < u(1) < 1.0
%         u(2) = elevator command in degrees
%         u(3) = aileron command in degrees
%         u(4) = rudder command in degrees
%  Output:
%         r(scalar) = the cost value
% LET THE CODE BEGIN !!!!!!!!   



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

if length(s)==3
u(1)=s(1);
u(2)=s(2);
x(2)=s(3);
else
u(1)=s(1);
u(2)=s(2);
u(3)=s(3);
u(4)=s(4);
x(2)=s(5);
x(4)=s(6);
x(5)=s(7);
end

%
% Get the current power and constraints
%
x(13) = tgear(u(1));
[x,u]=conf16(x,u,const);
xd=subf16(x,u);

%
% Steady Level flight
%
if orient == 1
   r=100.0*(xd(1)^2 + xd(2)^2 + xd(3)^2 + xd(7)^2 + xd(8)^2 + xd(9)^2);
end

%
% Steady Climb
%
if orient == 2
   r= 500.0*(xd(12)-x(1)*sin(gamm))^2 + xd(1)^2 + 100.0*(xd(2)^2 + xd(3)^2) + 10.0*( xd(7)^2 +...
      xd(8)^2 + xd(9)^2);
end



%
% Coord Turn
%
if orient == 3
   r=xd(1)*xd(1) + 100.0*( xd(2)*xd(2) + xd(3)*xd(3) + xd(12)*xd(12)) + 10.0*( xd(7)*xd(7) +...
     xd(8)*xd(8)+xd(9)*xd(9)) + 500.0*(xd(6)-tr)^2;
end

%
% Pitch Pull Up
%


if orient == 4
   r= 500.0*(xd(5)-thetadot)^2 + xd(1)^2 + 100.0*(xd(2)^2 + xd(3)^2) + 10.0*( xd(7)^2 +...
      xd(8)^2 + xd(9)^2);
end

%
% Scale r if it is less than 1
%
if r < 1.0
  r = r^0.5;
end


