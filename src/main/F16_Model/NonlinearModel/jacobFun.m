function [a,b,c,d]=jacobFunc(Xequil,Uequil,printOn)
%  [A,B,C,D]=jacobFun(Xequil,Uequil)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Program : jacobFunc
%	by: Mech 628 Incredible Group 2
%	Capts Chapa & St. Germain
%   Revised by: Peter Heidlauf, AFRL/RQQA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  This program numerically calculates the linearized A, B, C, & D matrices 
%  of an F-16 model given certain parameters where:
%       
%       x = A x  +  B u
%       y = C x  +  D u
%
%  Inputs include the equilibrium state vector and control vector.  If the 
%  routine is called with no inputs the user will be prompted to key the 
%  equilibrium values in by hand. Output may be shown coupled 
%  longitudinal/lateral or separate.  If convergence is not reached for 
%  any value, the user is prompted for an estimate.
%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  states:                                              controls:
%	x1 = Vt		x4 = phi	x7 = p	  x10 = pn			u1 = throttle
%	x2 = alpha	x5 = theta	x8 = q	  x11 = pe			u2 = elevator
%	x3 = beta	x6 = psi    x9 = r	  x12 = alt		    u3 = aileron
%                                     x13 = pow         u4 = rudder
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Order of the measurements described by y = Cx + Du are:
%
%  Coupled Long & Lat:  y = [ Az q alpha theta Vt Ay p r beta phi ]T
%
%  Note: angles are in degrees, angular rates are in deg/s, 
%        airspeed in ft/sec, accelerations (Az, Ay) are non-dimensional 
%        (i.e., in g's)
%
% HEIDLAUF NOTE:
%   States are in rads, outputs are in degs ?
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Order of the inputs u are:
% 
%  Coupled Long & Lat:                       u = [ thtl el ail rdr ]T
%					  					
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Script/Function calls:
%	getinput
%	subf16
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global az ay;

if nargin==3
    x=Xequil;
    u=Uequil;
elseif nargin==2
    x=Xequil;
    u=Uequil;
    printOn = 1;
else
    error('Wrong number of arguments provided')
end

if(printOn)
    disp('------------------------------------------------------------');
    disp('Running jacobFun.m');
end

xe=x;ue=u;
tol=0.0001;
xde=subf16(x,u);
aze=az;aye=ay;

%%%%%  	A matrix 	%%%%%

for i=1:13
	for j=1:13	
		x=xe;del=0.01;slope1=0;diff=.9;
		if xe(i)==0
			del=0.5;
		else
			del=del*xe(i);
		end			
 		while diff > tol		
 			x(i)=xe(i)+del;
			xd=subf16(x,u);
     			slope2=(xd(j)-xde(j))/(del);
        			diff=abs(slope1-slope2);
   			del=del*.1;
			slope1=slope2;
			if diff >1e6
				fprintf('No convergence for perturbed state X1 with state X2 : ');
				X1=i
				X2=j
				slope1=input('Enter estimate : ');
				diff=0;
			end
  		end
		AA(j,i)=slope1;
	end
end 

%%%%%  	B matrix	%%%%%

x=xe;
for i=1:4
	for j=1:13	
		u=ue;del=0.01;slope1=0;diff=1;
		if ue(i)==0
			del=0.5;
		else
			del=del*ue(i);
		end			
 		while diff > tol		
 			u(i)=ue(i)+del;
			xd=subf16(x,u);
     			slope2=(xd(j)-xde(j))/(del);
        			diff=abs(slope2-slope1);
   			del=del*.1;
			slope1=slope2;
			if diff >1e6
				fprintf('No convergence for perturbed input U with state X : ');
				U=i
				X=j
				slope1=input('Enter estimate : ');
				diff=0;
			end

  		end
		BB(j,i)=slope1;
	end
end 

%%%%%	C matrix	%%%%%

u=ue;
for i=1:13					%  az
	x=xe;del=0.01;slope1=0;diff=1;
	if xe(i)==0
		del=0.5;
	else
		del=del*xe(i);
	end			
 	while diff > tol		
 		x(i)=xe(i)+del;
		xd=subf16(x,u);
     		slope2=(az-aze)/(del);
        		diff=abs(slope2-slope1);
   		del=del*.1;
		slope1=slope2;
		if diff >1e6
			fprintf('No convergence for  az  with state X : ');
			X=i
			slope1=input('Enter estimate : ');
			diff=0;
		end
  	end
	CC(1,i)=slope1/(-32.2);	
end 

for i=1:13					%  ay
	x=xe;del=0.01;slope1=0;diff=1;
	if xe(i)==0
		del=0.5;
	else
		del=del*xe(i);
	end			
 	while diff > tol		
 		x(i)=xe(i)+del;
		xd=subf16(x,u);
     		slope2=(ay-aye)/(del);
        		diff=abs(slope2-slope1);
		del=del*.1;
		slope1=slope2;
		if diff >1e6
			fprintf('No convergence for  ay  with state X : ');
			X=i
			slope1=input('Enter estimate : ');
			diff=0;
		end
	end
	CC(6,i)=slope1/(32.2);	
end 
CC(2,:)=[0 0 0 0 0 0 0 57.3 0 0 0 0 0];
CC(3,:)=[0 57.3 0 0 0 0 0 0 0 0 0 0 0];
CC(4,:)=[0 0 0 0 57.3 0 0 0 0 0 0 0 0];
CC(5,:)=[1. 0 0 0 0 0 0 0 0 0 0 0 0];
CC(7,:)=[0 0 0 0 0 0 57.3 0 0 0 0 0 0];
CC(8,:)=[0 0 0 0 0 0 0 0 57.3 0 0 0 0];
CC(9,:)=[0 0 57.3 0 0 0 0 0 0 0 0 0 0];
CC(10,:)=[0 0 0 57.3 0 0 0 0 0 0 0 0 0];


%%%%%  	D matrix	%%%%%


x=xe;
for i=1:4					%  az
	u=ue;del=0.01;slope1=0;diff=1;
	if ue(i)==0
		del=0.5;
	else
		del=del*ue(i);
	end			
 	while diff > tol		
 		u(i)=ue(i)+del;
		xd=subf16(x,u);
     		slope2=(az-aze)/(del);
        		diff=abs(slope2-slope1);
   		del=del*.1;
		slope1=slope2;
		if diff >1e6
			fprintf('No convergence for  az  with input U : ');
			U=i
			slope1=input('Enter estimate : ');
			diff=0;
		end
  	end
	D(1,i)=slope1/(-32.2);
end 
for i=1:4					%  ay
	u=ue;del=0.01;slope1=0;diff=1;
	if ue(i)==0
		del=0.5;
	else
		del=del*ue(i);
	end			
 	while diff > tol		
 		u(i)=ue(i)+del;
		xd=subf16(x,u);
     		slope2=(ay-aye)/(del);
        		diff=abs(slope2-slope1);
   		del=del*.1;
		slope1=slope2;
		if diff >1e6
			fprintf('No convergence for  ay  with input U : ');
			U=i
			slope1=input('Enter estimate : ');
			diff=0;
		end

  	end
	D(6,i)=slope1/(32.2);
end
D(2,:)=[0 0 0 0];
D(3,:)=[0 0 0 0];
D(4,:)=[0 0 0 0];
D(5,:)=[0 0 0 0];
D(7,:)=[0 0 0 0];
D(8,:)=[0 0 0 0];
D(9,:)=[0 0 0 0];
D(10,:)=[0 0 0 0];

a=AA;
b=BB;
c=CC;
d=D;