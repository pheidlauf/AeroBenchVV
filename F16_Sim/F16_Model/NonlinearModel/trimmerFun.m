function [Xequil,Uequil]=trimmerFun(Xguess,Uguess,orient,inputs,printOn)
% [Xequil,Uequil]=trimmerFun(Xguess,Uguess,orient,inputs,printOn)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Program: trimmer
%	by: Mech 628 Incredible Group 1
%	    Ise, Shearer, Clark
%   Revised by: Peter Heidlauf, AFRL/RQQA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  This program numerically calculates the equilibrium state and control vectors of an F-16 model given 
%  certain parameters.  Inputs include initial guesses for the equilibrium state and input vectors.  
%  If the routine is called with no inputs the user will be prompted to key the equilibrium initial 
%  guesses in by hand.
%  The user will be prompted to pick one of the following A/C orientation options
%  and provide the desired altitude, airspeed, gamma, turn rate, pitch rate,etc. :
%
%     1. Wings Level (gamma = 0)
%     2. Wings Level (gamma <> 0)
%     3. Steady Constant Altitude Turn
%     4. Steady Pull Up
% 
%  The user will also be prompted for the number of iterations to be used in the numerical
%  minimization search.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  states:                                              controls:
%	x1 = Vt		x4 = phi	x7 = p	  x10 = pn			u1 = throttle
%	x2 = alpha	x5 = theta	x8 = q	  x11 = pe			u2 = elevator
%	x3 = beta	x6 = psi    x9 = r	  x12 = alt		    u3 = aileron
%                                     x13 = pow         u4 = rudder
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Script/Function calls:
%	getinput
%	subf16
%       clf16
%       conf16
%       fminsa
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





global ay az

format long

if(nargin>=2)
    x=Xguess;
    u=Uguess;
else
    % If called with insufficient args
    warning('Defaulting to zero states for initial trim guess');
    x=zeros(13,1);
    u=zeros(4,1);
end

if(nargin<5)
    printOn = 0;
end


if(printOn);
    disp('------------------------------------------------------------');
    disp('Running trimmerFun.m');
end

%        gamma singam rr  pr   tr  phi cphi sphi thetadot coord stab  orient
const = [0.0    0.0   0.0 0.0  0.0 0.0 1.0  0.0   0.0      0.0   0.0  1];

rtod = 57.29577951;

% orient = menu('Choose an A/C Orientation','Wings Level (gamma = 0)',...
% 'Wings Level (gamma <> 0)','Steady Turn','Steady Pull Up');
const(12) = orient;




ndof = 6;







%% BEGIN HEIDLAUF EDITS

% inputs: [Vt, h, gamm, psidot, thetadot]
if orient == 1
%    x(1) = input('Velocity Vector (ft/s) (VT): ');
%    x(12) = input('Altitude (ft) (h): ');
    x(1) = inputs(1);
    x(12) = inputs(2);
end



if orient == 2
%    x(1) = input('Velocity Vector (ft/s) (VT): ');
%    x(12) = input('Altitude (ft) (h): ');
%    gamm = input('Gamma (deg): ');
    x(1) = inputs(1);
    x(12) = inputs(2);
    gamm = inputs(3);
    const(1) = gamm/rtod;
    const(2) = sin(const(1));
end



if orient == 3
%    x(1) = input('Velocity Vector (ft/s) (VT): ');
%    x(12) = input('Altitude (ft) (h): ');
%    psidot = input('Turn Rate (deg/s) (Psi dot): ');
    x(1) = inputs(1);
    x(12) = inputs(2);
    psidot = inputs(4);
    const(5) = psidot/rtod;
end



if orient == 4
%    x(1) = input('Velocity Vector (ft/s) (VT): ');
%    x(12) = input('Altitude (ft) (h): ');
%    thetadot = input('Pitch Rate (deg/s) (Theta dot): ');
    x(1) = inputs(1);
    x(12) = inputs(2);
    thetadot = inputs(5);
    const(9) = thetadot/rtod;
end
%% END HEIDLAUF EDITS

%
% Set up the initial guess for the state and control vectors
%

if nargin<2
    disp(' ')
    disp('Next Input The Initial Guess For The Equilibrium State And Control Vectors')
    disp('Remember To Match The Altitude and Air Speed You Just Keyed In:')
    disp(' ')
    getinput
end

%

yesno = 1;
clear s

if orient == 3
    s(1)=u(1);
    s(2)=u(2);
    s(3)=u(3);
    s(4)=u(4);
    s(5)=x(2);
    s(6)=x(4);
    s(7)=x(5);
else
    s(1)=u(1);
    s(2)=u(2);
    s(3)=x(2);
end


counter = 0;
while yesno == 1
    counter = counter + 1;
    options = [0 1.0E-9 1.0E-9 0 0 0 0 0 0 0 0 0 0 1000];
	
%    ot = input('Required# of iterations (def. = 1000): ');   
%    if(isempty(ot))
%        options(14) = 1000;
%    else
%        options(14) = ot
%    end
    options(14) = 1000;

    [s,options,x,u,fcost,lcost] = fminsa('clf16',s,options,[],x,u,const);
    [amach,qbar]=adc(x(1),x(12));
    
    if(printOn)
        fprintf('\n')
        if ndof > 3
            fprintf('Throttle (percent):            %g\n', u(1))
            fprintf('Elevator (deg):                %g\n', u(2))
            fprintf('Ailerons (deg):                %g\n', u(3))
            fprintf('Rudder (deg):                  %g\n', u(4))
            fprintf('Angle of Attack (deg):         %g\n', rtod*x(2))
            fprintf('Sideslip Angle (deg):          %g\n', rtod*x(3))
            fprintf('Pitch Angle (deg):             %g\n', rtod*x(5))
            fprintf('Bank Angle (deg):              %g\n', rtod*x(4))
            fprintf('Normal Acceleration (g):       %g\n', az/32.2)
            fprintf('Lateral Acceleration (g):      %g\n', ay/32.2)
            fprintf('Dynamic Pressure (psf):        %g\n', qbar)
            fprintf('Mach Number:                   %g\n', amach)
        else
            fprintf('Throttle (percent):		%g\n', u(1))
            fprintf('Elevator (deg):			%g\n', u(2))
            fprintf('Alpha (deg):               %g\n', x(2)*rtod)
            fprintf('Pitch Angle (deg):         %g\n', x(5)*rtod)
            fprintf('Normal Acceleration (g):	%g\n', az/32.2)
            fprintf('Dynamic Pressure (psf):        %g\n', qbar)
            fprintf('Mach Number:               %g\n', amach)
        end

        fprintf('\n')
        fprintf('Initial Cost Function:         %g\n', fcost)
        fprintf('Final Cost Function:           %g\n', lcost)
    end

%     yesno = menu('More Iterations?','Yes','No');

    if(lcost > 1e-7)
        if(counter < 100)
            yesno = 1;
        else
            yesno = 2;
            warning('trimmerFun stopped by function call limit');
        end
    else
        yesno = 2; 
    end
end
Xequil=x;
Uequil=u;