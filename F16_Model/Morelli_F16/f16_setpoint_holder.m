function [f,throttle_act_com,elevator_act_com,aileron_act_com,rudder_act_com]=f16_setpoint_holder(t,y,desired_velocity,desired_altitude,desired_alpha,desired_theta,constants)
%This function when used with a numerical integration produces the time
%history of an f16 simulation that responds to a commanded step input in
%normal acceleration and phi while maintaining a trim setpoint specified.

%--Inputs
%    xeq is a state vector containing the trim [Vt    alpha   beta    phi theta    psi      p       q       r       pn     pe      alt      pow]
%    desired_velocity is the velocity to be maintained
%    desired_altitude is the altitude to be maintained
%    desired_alpha is the angle of attack to be maintained
%    desired_theta is the pitch angle to be maintained
%    constants contains control gains, conversion factors, and step amplitudes
%    constants.rtod-conversion from radian to degree
%    constants.ktheta-proportional gain on theta
%    constants.kv-proportional gain on velocity
%    constants.kq-proportional gain on pitch rate
%    constants.kalpha-proportional gain on angle of attack
%    constants.kgp-proportional gain on normal acceleration
%    constants.kgi-integral gain on normal acceleration
%    constants.kh-proportional gain on altitude
%    constants.kphi-proportional gain on phi
%    constants.kp-proportional gain on roll rate
%    constants.gamp-normal acceleration step input amplitude
%    constants.phiamp-phi step input amplitude
%% Setting preconditions
temp=f16eom([y(1:16);zeros(4,1)]);
azout_prev=temp(19);    %estimate of the current z acceleration
rtod=constants.rtod;
ktheta=constants.ktheta;
kv=constants.kv;
kq=constants.kq;
kalpha=constants.kalpha;
kgp=constants.kgp;
kgi=constants.kgi; 
kh=constants.kh;
kphi=constants.kphi;
kp=constants.kp;
gamp=constants.gamp;
phiamp=constants.phiamp;
%% Simulated pilot commands
throttle_pilot_com=0;
elevator_pilot_com=0;
aileron_pilot_com=0;
rudder_pilot_com=0;

%%
throttle_delta=throttle_feedback_controller(y,desired_velocity,kv);   %addition to throttle command needed to maintain step setpoint
[elevator_delta,error_g]=elevator_feedback_controller(y,desired_altitude,desired_alpha,desired_theta,kq,kh,kalpha,ktheta,kgp,kgi,gamp,azout_prev);  %addition to the elevator commmand needed to maintain step setpoint
aileron_delta=aileron_feedback_controller(y,kphi,kp,rtod,phiamp); %addition to the aileron command needed to maintain step setpoint
rudder_delta=rudder_feedback_controller(y); %addition to the rudder command needed to maintain step setpoint
  
throttle_act_com=throttle_pilot_com+throttle_delta;
elevator_act_com=elevator_pilot_com+elevator_delta;
aileron_act_com=aileron_pilot_com+aileron_delta;
rudder_act_com=rudder_pilot_com+rudder_delta;
temp2=f16eom([y(1:16);throttle_act_com;elevator_act_com;aileron_act_com;rudder_act_com]);
f=[temp2(1:16);error_g];

%% Helper functions
function yd=f16eom(y)
%
%
%  Note: This routine is similar to subf16.m, the equations of motion are the same,
%        but the input and output format is significantly different.
%
%  This routine outputs a vector, yd, for the computer model of an F-16 aircraft.
%  This is called by etc.mdl to run nonlinear F-16 simulations.  Prior to running, this m-file
%  must be edited to set the proper c.g. location (see 'xcg = ' below). The nominal value of xcg=.35
%  generally produces unstable open loop A/C dynamics.  A value of xcg<=.3 will generally produce stable
%  A/C dynamics.  If you choose to run xcg>=.35 you should definitely add a stablizing feedback 
%  controller or the plane will be difficult to control by the pilot.
%
%  The first 16 components of the input vector y is the aircraft state vector, x, where:
%
%
%         y(1) = air speed, VT    (ft/sec)
%         y(2) = angle of attack, alpha  (rad)
%         y(3) = angle of sideslip, beta (rad)
%         y(4) = roll angle, phi  (rad)
%         y(5) = pitch angle, theta  (rad)
%         y(6) = yaw angle, psi  (rad)
%         y(7) = roll rate, P  (rad/sec)
%         y(8) = pitch rate, Q  (rad/sec)
%         y(9) = yaw rate, R  (rad/sec)
%         y(10) = northward horizontal displacement, pn  (feet)
%         y(11) = eastward horizontal displacement, pe  (feet)
%         y(12) = altitude, h  (feet)
%         y(13) = engine thrust dynamics lag state, pow
%			 y(14) = elevator actuator deflection, deg
%			 y(15) = aileron actuator deflection, deg
%			 y(16) = rudder actuator deflection, deg 
%
%  The last 4 components of the input vector y are the control input commands
%
%			 y(17) = throttle command,  0 <  thtlc < 1.0
%			 y(18) = elevator command, deg
%			 y(19) = aileron command, deg
%			 y(20) = rudder command, deg
%
%
%  The first 16 components of the output vector yd is dx/dt(i.e., the aircraft
%  state vector derivative), which is the derivatives of the first 16 y vector components :
%
%
%         yd(1) = derivative of air speed, VT    (ft/sec^2)
%         yd(2) = derivative of angle of attack, alpha  (rad/sec)
%         yd(3) = derivative of angle of sideslip, beta (rad/sec)
%         yd(4) = derivative of roll angle, phi  (rad/sec)
%         yd(5) = derivative of pitch angle, theta  (rad/sec)
%         yd(6) = derivative of yaw angle, psi  (rad/sec)
%         yd(7) = derivative of roll rate, P  (rad/sec^2)
%         yd(8) = derivative of pitch rate, Q  (rad/sec^2)
%         yd(9) = derivative of yaw rate, R  (rad/sec^2)
%         yd(10) = derivative of northward horizontal displacement, pn  (feet/sec)
%         yd(11) = derivative of eastward horizontal displacement, pe  (feet/sec)
%         yd(12) = derivative of altitude, h  (feet/sec)
%         yd(13) = derivative of engine thrust dynamics lag state, pow  (1/sec)
%			 yd(14) = derivative of elevator actuator deflection, deg/sec
%			 yd(15) = derivative of aileron actuator deflection, deg/sec
%			 yd(16) = derivative of rudder actuator deflection, deg/sec
%
%
%  The last 6 components of the output vector yd are the linear and angular accelerations
%  that the pilot would sense and are the commands to be sent to the centrifuge:
%
%			 yd(17) = output lin_accel_x (ft/sec^2)
%			 yd(18) = output lin_accel_y (ft/sec^2)
%			 yd(19) = output lin_accel_z (ft/sec^2)
%			 yd(20) = output rot_accel_x (rad/sec^2)
%			 yd(21) = output rot_accel_y (rad/sec^2)
%			 yd(22) = output rot_accel_z (rad/sec^2)
%
%%
yd=zeros(22,1);
thtlc=y(17);
elc=y(18);
ailc=y(19);
rdrc=y(20);
%%

% The following is the c.g. location which can be modified (nominal is xcg=.35)

xcg=.35;

s=300;
b=30;
cbar=11.32;
rm=1.57e-3;
xcgr=.35;
he=160.0;
c1=-.770;
c2=.02755;
c3=1.055e-4;
c4=1.642e-6;
c5=.9604;
c6=1.759e-2;
c7=1.792e-5;
c8=-.7336;
c9=1.587e-5;
rtod=57.29578;
g=32.17;


vt=y(1);
alpha=y(2)*rtod;
beta=y(3)*rtod;
phi=y(4);
theta=y(5);
psi=y(6);
p=y(7);
q=y(8);
r=y(9);
alt=y(12);
[amach,qbar]=adc(vt,alt);

pow=y(13);
if(thtlc>=1.0)
    thtlc=1.0;
elseif(thtlc<0.)
    thtlc=0.;
end;
cpow=tgear(thtlc);
yd(13)=pdot(pow,cpow);
t=thrust(pow,alt,amach);

el=y(14);
sel=sign(el);
yd(14)=20.202*(elc-el);
if(abs(el)>=25 && sign(yd(14))==sel)
    yd(14)=0;
    el=sel*25;
end
if(abs(yd(14))>=60)
    yd(14)=sign(yd(14))*60;
end

ail=y(15);
sal=sign(ail);
yd(15)=20.202*(ailc-ail);
if(abs(ail)>=21.5 && sign(yd(15))==sal)
    yd(15)=0;
    ail=sal*21.5;
end
if(abs(yd(15))>=80)
    yd(15)=sign(yd(15))*80;
end

rdr=y(16);
yd(16)=20.202*(rdrc-rdr);
if(abs(rdr)>=30 && sign(yd(16))==sign(rdr))
    yd(16)=0;
    rdr=sign(rdr)*30;
end
if(abs(yd(16))>=120)
    yd(16)=sign(yd(16))*120;
end

%%
[cxt,cyt,czt,clt,cmt,cnt]=SnL_f16_aero_coeffs(alpha,beta,el,ail,rdr,p,q,r,cbar,b,vt,xcg,xcgr);
% [cxt,cyt,czt,clt,cmt,cnt]=Morellif16(alpha*pi/180,beta*pi/180,el*pi/180,ail*pi/180,rdr*pi/180,p,q,r,cbar,b,vt,xcg,xcgr);

%%

u=vt*cos(y(2))*cos(y(3));
v=vt*sin(y(3));
w=vt*sin(y(2))*cos(y(3));


%%
udot=r*v-q*w-g*sin(theta)+rm*(qbar*s*cxt+t);
vdot=p*w-r*u+g*cos(theta)*sin(phi)+rm*qbar*s*cyt;
wdot=q*u-p*v+g*cos(theta)*cos(phi)+rm*qbar*s*czt;

yd(1)=(u*udot+v*vdot+w*wdot)/vt;
yd(2)=(u*wdot-w*udot)/(u^2+w^2);
yd(3)=(vt*vdot-v*yd(1))*cos(y(3))/(u^2+w^2); 
yd(4)=p+(sin(theta)/cos(theta))*(q*sin(phi)+r*cos(phi));
yd(5)=q*cos(phi)-r*sin(phi);
yd(6)=(q*sin(phi)+r*cos(phi))/cos(theta);
yd(7)=(c2*p+c1*r+c4*he)*q+qbar*s*b*(c3*clt+c4*cnt);
yd(8)=(c5*p-c7*he)*r+c6*(r*r-p*p)+qbar*s*cbar*c7*cmt;
yd(9)=(c8*p-c2*r+c9*he)*q+qbar*s*b*(c4*clt+c9*cnt);

t1=sin(phi)*cos(psi);
t2=cos(phi)*sin(theta);
t3=sin(phi)*sin(psi);
s1=cos(theta)*cos(psi);
s2=cos(theta)*sin(psi);
s3=t1*sin(theta)-cos(phi)*sin(psi);
s4=t3*sin(theta)+cos(phi)*cos(psi);
s5=sin(phi)*cos(theta);
s6=t2*cos(psi)+t3;
s7=t2*sin(psi)-t1;
s8=cos(phi)*cos(theta);

yd(10)=u*s1+v*s3+w*s6;
yd(11)=u*s2+v*s4+w*s7;
yd(12)=u*sin(theta)-v*s5-w*s8;
if(alt<=0 && sign(yd(12))<0)   % can't fly underground
    yd(12)=0;
end
xa=15.0;                  % sets distance normal accel is in front of the c.g. (xa=15.0 at pilot)
az=rm*qbar*s*czt-xa*yd(8);           % moves normal accel in front of c.g.
ay=rm*qbar*s*cyt+xa*yd(9);           % moves side accel in front of c.g.
yd(17)=rm*(qbar*s*cxt+t);					  % output lin_accel_x (ft/sec^2)
yd(18)=ay;					  % output lin_accel_y (ft/sec^2)
yd(19)=az;					  % output lin_accel_z (ft/sec^2)
yd(20)=yd(7);				  % output rot_accel_x (rad/sec^2)
yd(21)=yd(8);				  % output rot_accel_y (rad/sec^2)
yd(22)=yd(9);				  % output rot_accel_z (rad/sec^2)

function [amach,qbar]=adc(vt,alt)
ro=2.377e-3;
tfac=1-.703e-5*alt;
t=519*tfac;
if(alt >=35000), t=390; end
rho=ro*(tfac^4.14);
amach=vt/sqrt(1.4*1716.3*t);
qbar=.5*rho*vt*vt;

function pd=pdot(p3,p1)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Script/Function calls:
%
%  rtau
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if(p1>=50)
  if(p3>=50)
     t=5;
     p2=p1;
  else
     p2=60;
     t=rtau(p2-p3);
  end
else
  if(p3>=50)
     t=5;
     p2=40;
  else
     p2=p1;
     t=rtau(p2-p3);
  end
end
pd=t*(p2-p3);

function rt=rtau(dp)
if(dp<=25)
  rt=1.0;
elseif(dp>=50)
  rt=.1;
else
  rt=1.9-.036*dp;
end

function tg=tgear(thtl)
if(thtl<=.77)
 tg=64.94*thtl;
else
 tg=217.38*thtl-117.38;
end

function thrst=thrust(pow,alt,rmach)

a=[1060 670 880 1140 1500 1860
635 425 690 1010 1330 1700
60 25 345 755 1130 1525
-1020 -170 -300 350 910 1360
-2700 -1900 -1300 -247 600 1100
-3600 -1400 -595 -342 -200 700]';

b=[12680 9150 6200 3950 2450 1400
12680 9150 6313 4040 2470 1400
12610 9312 6610 4290 2600 1560
12640 9839 7090 4660 2840 1660
12390 10176 7750 5320 3250 1930
11680 9848 8050 6100 3800 2310]';

c=[20000 15000 10800 7000 4000 2500
21420 15700 11225 7323 4435 2600
22700 16860 12250 8154 5000 2835
24240 18910 13760 9285 5700 3215
26070 21075 15975 11115 6860 3950
28886 23319 18300 13484 8642 5057]';

if(alt<0)
    alt=0.01;
end
h=.0001*alt;
i=fix(h);
if(i>=5)
    i=4;
end
dh=h-i;
rm=5*rmach;
m=fix(rm);
if(m>=5)
    m=4;
end
dm=rm-m;
cdh=1-dh;

i=i+1;
m=m+1;

s=b(i,m)*cdh+b(i+1,m)*dh;
t=b(i,m+1)*cdh+b(i+1,m+1)*dh;
tmil=s+(t-s)*dm;
if(pow<50)
    s=a(i,m)*cdh+a(i+1,m)*dh;
    t=a(i,m+1)*cdh+a(i+1,m+1)*dh;
    tidl=s+(t-s)*dm;
    thrst=tidl+(tmil-tidl)*pow*.02;
else
    s=c(i,m)*cdh+c(i+1,m)*dh;
    t=c(i,m+1)*cdh+c(i+1,m+1)*dh;
    tmax=s+(t-s)*dm;
    thrst=tmil+(tmax-tmil)*(pow-50)*.02;
end

% function [Cx,Cy,Cz,Cl,Cm,Cn]=Morellif16(alpha,beta,de,da,dr,p,q,r,cbar,b,V,xcg,xcgref)
% %This function produces the force and moment coefficients for Morelli's f16
% %aerodynamic model
% %V is in ft/s  cbar and b are in ft xcg is normalized with respect to total
% %length
% %All angles are assumed to be inputed as radian measurments
% 
% alpha=max(-10*pi/180,min(45*pi/180,alpha)); %bounds alpha between -10 deg and 45 deg
% beta=max(-30*pi/180,min(30*pi/180,beta));   %bounds beta between -30 deg and 30 deg
% de=max(-25*pi/180,min(25*pi/180,de));   %bounds elevator deflection between -25 deg and 25 deg
% da=max(-21.5*pi/180,min(21.5*pi/180,da));   %bounds aileron deflection between -21.5 deg and 21.5 deg
% dr=max(-30*pi/180,min(30*pi/180,dr));   %bounds rudder deflection between -30 deg and 30 deg
% 
% % xcgref=0.35;    %reference longitudinal cg position in Morelli f16 model
%    
% 
% phat=p*b/(2*V);
% qhat=q*cbar/(2*V);
% rhat=r*b/(2*V);
% %%
% a0=-1.943367e-2;
% a1=2.136104e-1;
% a2=-2.903457e-1;
% a3=-3.348641e-3;
% a4=-2.060504e-1;
% a5=6.988016e-1;
% a6=-9.035381e-1;
% 
% b0=4.833383e-1;
% b1=8.644627;
% b2=1.131098e1;
% b3=-7.422961e1;
% b4=6.075776e1;
% 
% c0=-1.145916;
% c1=6.016057e-2;
% c2=1.642479e-1;
% 
% d0=-1.006733e-1;
% d1=8.679799e-1;
% d2=4.260586;
% d3=-6.923267;
% 
% e0=8.071648e-1;
% e1=1.189633e-1;
% e2=4.177702;
% e3=-9.162236;
% 
% f0=-1.378278e-1;
% f1=-4.211369;
% f2=4.775187;
% f3=-1.026225e1;
% f4=8.399763;
% f5=-4.354000e-1;
% 
% g0=-3.054956e1;
% g1=-4.132305e1;
% g2=3.292788e2;
% g3=-6.848038e2;
% g4=4.080244e2;
% 
% h0=-1.05853e-1;
% h1=-5.776677e-1;
% h2=-1.672435e-2;
% h3=1.357256e-1;
% h4=2.172952e-1;
% h5=3.464156;
% h6=-2.835451;
% h7=-1.098104;
% 
% i0=-4.126806e-1;
% i1=-1.189974e-1;
% i2=1.247721;
% i3=-7.391132e-1;
% 
% j0=6.250437e-2;
% j1=6.067723e-1;
% j2=-1.101964;
% j3=9.100087;
% j4=-1.192672e1;
% 
% k0=-1.463144e-1;
% k1=-4.07391e-2;
% k2=3.253159e-2;
% k3=4.851209e-1;
% k4=2.978850e-1;
% k5=-3.746393e-1;
% k6=-3.213068e-1;
% 
% l0=2.635729e-2;
% l1=-2.192910e-2;
% l2=-3.152901e-3;
% l3=-5.817803e-2;
% l4=4.516159e-1;
% l5=-4.928702e-1;
% l6=-1.579864e-2;
% 
% m0=-2.029370e-2;
% m1=4.660702e-2;
% m2=-6.012308e-1;
% m3=-8.062977e-2;
% m4=8.320429e-2;
% m5=5.018538e-1;
% m6=6.378864e-1;
% m7=4.226356e-1;
% 
% n0=-5.19153;
% n1=-3.554716;
% n2=-3.598636e1;
% n3=2.247355e2;
% n4=-4.120991e2;
% n5=2.411750e2;
% 
% o0=2.993363e-1;
% o1=6.594004e-2;
% o2=-2.003125e-1;
% o3=-6.233977e-2;
% o4=-2.107885;
% o5=2.141420;
% o6=8.476901e-1;
% 
% p0=2.677652e-2;
% p1=-3.298246e-1;
% p2=1.926178e-1;
% p3=4.013325;
% p4=-4.404302;
% 
% q0=-3.698756e-1;
% q1=-1.167551e-1;
% q2=-7.641297e-1;
% 
% r0=-3.348717e-2;
% r1=4.276655e-2;
% r2=6.573646e-3;
% r3=3.535831e-1;
% r4=-1.373308;
% r5=1.237582;
% r6=2.302543e-1;
% r7=-2.512876e-1;
% r8=1.588105e-1;
% r9=-5.199526e-1;
% 
% s0=-8.115894e-2;
% s1=-1.156580e-2;
% s2=2.514167e-2;
% s3=2.038748e-1;
% s4=-3.337476e-1;
% s5=1.004297e-1;
% 
% %%
% Cx0=a0+a1*alpha+a2*de^2+a3*de+a4*alpha*de+a5*alpha^2+a6*alpha^3;
% Cxq=b0+b1*alpha+b2*alpha^2+b3*alpha^3+b4*alpha^4;
% Cy0=c0*beta+c1*da+c2*dr;
% Cyp=d0+d1*alpha+d2*alpha^2+d3*alpha^3;
% Cyr=e0+e1*alpha+e2*alpha^2+e3*alpha^3;
% Cz0=(f0+f1*alpha+f2*alpha^2+f3*alpha^3+f4*alpha^4)*(1-beta^2)+f5*de;
% Czq=g0+g1*alpha+g2*alpha^2+g3*alpha^3+g4*alpha^4;
% Cl0=h0*beta+h1*alpha*beta+h2*alpha^2*beta+h3*beta^2+h4*alpha*beta^2+h5*alpha^3*beta+h6*alpha^4*beta+h7*alpha^2*beta^2;
% Clp=i0+i1*alpha+i2*alpha^2+i3*alpha^3;
% Clr=j0+j1*alpha+j2*alpha^2+j3*alpha^3+j4*alpha^4;
% Clda=k0+k1*alpha+k2*beta+k3*alpha^2+k4*alpha*beta+k5*alpha^2*beta+k6*alpha^3;
% Cldr=l0+l1*alpha+l2*beta+l3*alpha*beta+l4*alpha^2*beta+l5*alpha^3*beta+l6*beta^2;
% Cm0=m0+m1*alpha+m2*de+m3*alpha*de+m4*de^2+m5*alpha^2*de+m6*de^3+m7*alpha*de^2;
% 
% 
% Cmq=n0+n1*alpha+n2*alpha^2+n3*alpha^3+n4*alpha^4+n5*alpha^5;
% Cn0=o0*beta+o1*alpha*beta+o2*beta^2+o3*alpha*beta^2+o4*alpha^2*beta+o5*alpha^2*beta^2+o6*alpha^3*beta;
% Cnp=p0+p1*alpha+p2*alpha^2+p3*alpha^3+p4*alpha^4;
% Cnr=q0+q1*alpha+q2*alpha^2;
% Cnda=r0+r1*alpha+r2*beta+r3*alpha*beta+r4*alpha^2*beta+r5*alpha^3*beta+r6*alpha^2+r7*alpha^3+r8*beta^3+r9*alpha*beta^3;
% Cndr=s0+s1*alpha+s2*beta+s3*alpha*beta+s4*alpha^2*beta+s5*alpha^2;
% %%
% 
% Cx=Cx0+Cxq*qhat;
% Cy=Cy0+Cyp*phat+Cyr*rhat;
% Cz=Cz0+Czq*qhat;
% Cl=Cl0+Clp*phat+Clr*rhat+Clda*da+Cldr*dr;
% Cm=Cm0+Cmq*qhat+Cz*(xcgref-xcg);
% Cn=Cn0+Cnp*phat+Cnr*rhat+Cnda*da+Cndr*dr-Cy*(xcgref-xcg)*(cbar/b);


function throttle_delta=throttle_feedback_controller(yd,Vt,kv)
% Throttle feedback controller
% Inputs:
%   yd--state vector
%   Vt--desired velocity
% Outputs:
%   throttle_delta--delta to throttle command
%%
V=yd(1);    %current velocity
throttle_delta=(Vt-V)*kv;

function [elevator_delta,error_g]=elevator_feedback_controller(yd,h_desired,alpha_desired,theta_desired,kq,kh,kalpha,ktheta,kgp,kgi,gamp,az)
%This function calculates the elevator delta
% Inputs
%   yd--state vector
% Outputs
%    elevator_delta--delta to elevator comman
%%
alpha=yd(2);    %angle of attack
theta=yd(5);    %pitch angle
q=yd(8);        %pitch rate about body frame y axis
h=yd(12);       %altitude
%%
error_h=h_desired-h;
error_alpha=alpha-alpha_desired;
error_theta=theta-theta_desired;
error_g=gamp-(-az/32.2);
int_error_g=yd(17); 
elevator_delta=(error_h*kh-error_theta*ktheta)-kq*q-kalpha*error_alpha+(kgp*error_g+kgi*int_error_g);

function aileron_delta=aileron_feedback_controller(yd,kphi,kp,rtod,phiamp)
%%
phi=yd(4);
p=yd(7);
%%
error_phi=phiamp-phi*rtod;
aileron_delta=error_phi*kphi-p*kp;

function rudder_delta=rudder_feedback_controller(yd)
rudder_delta=sum(yd.*zeros(17,1));

function[Cx,Cy,Cz,Cl,Cm,Cn]=SnL_f16_aero_coeffs(alpha,beta,el,ail,rdr,p,q,r,cbar,b,vt,xcg,xcgr)
%This function produces the force and moment coefficients used in the f16
%model utilized in subf16
%all angles are assumed to be degree measures
%V is in ft/s  cbar and b are in ft xcg is normalized with respect to total
%length
%%
cxt=cx(alpha,el);
cyt=cy(beta,ail,rdr);
czt=cz(alpha,beta,el);
d=dampp(alpha);
%%
dail=ail/20;
drdr=rdr/30;
clt=cl(alpha,beta)+dlda(alpha,beta)*dail+dldr(alpha,beta)*drdr;
cmt=cm(alpha,el);
cnt=cn(alpha,beta)+dnda(alpha,beta)*dail+dndr(alpha,beta)*drdr;
tvt=.5/vt;
b2v=b*tvt;
cq=cbar*q*tvt;
%%
Cx=cxt+cq*d(1);
Cy=cyt+b2v*(d(2)*r+d(3)*p);
Cz=czt+cq*d(4);
Cl=clt+b2v*(d(5)*r+d(6)*p);
Cm=cmt+cq*d(7)+czt*(xcgr-xcg);
Cn=cnt+b2v*(d(8)*r+d(9)*p)-cyt*(xcgr-xcg)*cbar/b;

function cxx=cx(alpha,el)
a=[-.099 -.081 -.081 -.063 -.025 .044 .097 .113 .145 .167 .174 .166
-.048 -.038 -.040 -.021 .016 .083 .127 .137 .162 .177 .179 .167
-.022 -.020 -.021 -.004 .032 .094 .128 .130 .154 .161 .155 .138
-.040 -.038 -.039 -.025 .006 .062 .087 .085 .100 .110 .104 .091
-.083 -.073 -.076 -.072 -.046 .012 .024 .025 .043 .053 .047 .040]';

cxx=SnL_2D_interpolation1(a,alpha,el);

function cyy=cy(beta,ail,rdr)
cyy=-.02*beta+.021*(ail/20)+.086*(rdr/30);

function czz=cz(alpha,beta,el)
a=[.770 .241 -.100 -.415 -.731 -1.053 -1.355 -1.646 -1.917 -2.120 -2.248 -2.229]';

s=SnL_1D_interpolation(a,alpha);
czz=s*(1-(beta/57.3)^2)-.19*(el/25);

function d=dampp(alpha)
d=zeros(9,1);
a=[-.267 -.110 .308 1.34 2.08 2.91 2.76 2.05 1.50 1.49 1.83 1.21
.882 .852 .876 .958 .962 .974 .819 .483 .590 1.21 -.493 -1.04
-.108 -.108 -.188 .110 .258 .226 .344 .362 .611 .529 .298 -2.27
-8.80 -25.8 -28.9 -31.4 -31.2 -30.7 -27.7 -28.2 -29.0 -29.8 -38.3 -35.3
-.126 -.026 .063 .113 .208 .230 .319 .437 .680 .100 .447 -.330
-.360 -.359 -.443 -.420 -.383 -.375 -.329 -.294 -.230 -.210 -.120 -.100
-7.21 -.540 -5.23 -5.26 -6.11 -6.64 -5.69 -6.00 -6.20 -6.40 -6.60 -6.00
-.380 -.363 -.378 -.386 -.370 -.453 -.550 -.582 -.595 -.637 -1.02 -.840
.061 .052 .052 -.012 -.013 -.024 .050 .150 .130 .158 .240 .150]';
for i=1:9
    d(i)=SnL_1D_interpolation(a(:,i),alpha);
end


function cll=cl(alpha,beta)
a=[0 0 0 0 0 0 0 0 0 0 0 0
-.001 -.004 -.008 -.012 -.016 -.022 -.022 -.021 -.015 -.008 -.013 -.015
-.003 -.009 -.017 -.024 -.030 -.041 -.045 -.040 -.016 -.002 -.010 -.019
-.001 -.010 -.020 -.030 -.039 -.054 -.057 -.054 -.023 -.006 -.014 -.027
.000 -.010 -.022 -.034 -.047 -.060 -.069 -.067 -.033 -.036 -.035 -.035
.007 -.010 -.023 -.034 -.049 -.063 -.081 -.079 -.060 -.058 -.062 -.059
.009 -.011 -.023 -.037 -.050 -.068 -.089 -.088 -.091 -.076 -.077 -.076]';
dum=SnL_2D_interpolation3(a,alpha,beta);
cll=dum*sign(beta);

function dl=dlda(alpha,beta)
a=[-.041 -.052 -.053 -.056 -.050 -.056 -.082 -.059 -.042 -.038 -.027 -.017
-.041 -.053 -.053 -.053 -.050 -.051 -.066 -.043 -.038 -.027 -.023 -.016
-.042 -.053 -.052 -.051 -.049 -.049 -.043 -.035 -.026 -.016 -.018 -.014
-.040 -.052 -.051 -.052 -.048 -.048 -.042 -.037 -.031 -.026 -.017 -.012
-.043 -.049 -.048 -.049 -.043 -.042 -.042 -.036 -.025 -.021 -.016 -.011
-.044 -.048 -.048 -.047 -.042 -.041 -.020 -.028 -.013 -.014 -.011 -.010
-.043 -.049 -.047 -.045 -.042 -.037 -.003 -.013 -.010 -.003 -.007 -.008]';
dl=SnL_2D_interpolation2(a,alpha,beta);

function dl=dldr(alpha,beta)
a=[.005 .017 .014 .010 -.005 .009 .019 .005 -.000 -.005 -.011 .008
.007 .016 .014 .014 .013 .009 .012 .005 .000 .004 .009 .007
.013 .013 .011 .012 .011 .009 .008 .005 -.002 .005 .003 .005
.018 .015 .015 .014 .014 .014 .014 .015 .013 .011 .006 .001
.015 .014 .013 .013 .012 .011 .011 .010 .008 .008 .007 .003
.021 .011 .010 .011 .010 .009 .008 .010 .006 .005 .000 .001
.023 .010 .011 .011 .011 .010 .008 .010 .006 .014 .020 .000]';

dl=SnL_2D_interpolation2(a,alpha,beta);

function cmm=cm(alpha,el)
a=[.205 .168 .186 .196 .213 .251 .245 .238 .252 .231 .198 .192
.081 .077 .107 .110 .110 .141 .127 .119 .133 .108 .081 .093
-.046 -.020 -.009 -.005 -.006 .010 .006 -.001 .014 .000 -.013 .032
-.174 -.145 -.121 -.127 -.129 -.102 -.097 -.113 -.087 -.084 -.069 -.006
-.259 -.202 -.184 -.193 -.199 -.150 -.160 -.167 -.104 -.076 -.041 -.005]';

cmm=SnL_2D_interpolation1(a,alpha,el);

function dl=dnda(alpha,beta)
a=[.001 -.027 -.017 -.013 -.012 -.016 .001 .017 .011 .017 .008 .016
.002 -.014 -.016 -.016 -.014 -.019 -.021 .002 .012 .016 .015 .011
-.006 -.008 -.006 -.006 -.005 -.008 -.005 .007 .004 .007 .006 .006
-.011 -.011 -.010 -.009 -.008 -.006 .000 .004 .007 .010 .004 .010
-.015 -.015 -.014 -.012 -.011 -.008 -.002 .002 .006 .012 .011 .011
-.024 -.010 -.004 -.002 -.001 .003 .014 .006 -.001 .004 .004 .006
-.022 .002 -.003 -.005 -.003 -.001 -.009 -.009 -.001 .003 -.002 .001]';

dl=SnL_2D_interpolation2(a,alpha,beta);

function dl=dndr(alpha,beta)
a=[-.018 -.052 -.052 -.052 -.054 -.049 -.059 -.051 -.030 -.037 -.026 -.013
-.028 -.051 -.043 -.046 -.045 -.049 -.057 -.052 -.030 -.033 -.030 -.008
-.037 -.041 -.038 -.040 -.040 -.038 -.037 -.030 -.027 -.024 -.019 -.013
-.048 -.045 -.045 -.045 -.044 -.045 -.047 -.048 -.049 -.045 -.033 -.016
-.043 -.044 -.041 -.041 -.040 -.038 -.034 -.035 -.035 -.029 -.022 -.009
-.052 -.034 -.036 -.036 -.035 -.028 -.024 -.023 -.020 -.016 -.010 -.014
-.062 -.034 -.027 -.028 -.027 -.027 -.023 -.023 -.019 -.009 -.025 -.010]';

dl=SnL_2D_interpolation2(a,alpha,beta);

function cnn=cn(alpha,beta)
a=[0 0 0 0 0 0 0 0 0 0 0 0
.018 .019 .018 .019 .019 .018 .013 .007 .004 -.014 -.017 -.033
.038 .042 .042 .042 .043 .039 .030 .017 .004 -.035 -.047 -.057
.056 .057 .059 .058 .058 .053 .032 .012 .002 -.046 -.071 -.073
.064 .077 .076 .074 .073 .057 .029 .007 .012 -.034 -.065 -.041
.074 .086 .093 .089 .080 .062 .049 .022 .028 -.012 -.002 -.013
.079 .090 .106 .106 .096 .080 .068 .030 .064 .015 .011 -.001]';

dum=SnL_2D_interpolation3(a,alpha,beta);
cnn=dum*sign(beta);

function value=SnL_2D_interpolation3(a,alpha,beta)
s=.2*alpha;
k=fix(s);
if (k<=-2)
    k=-1;
end
if (k>=9)
    k=8;
end
da=s-k;
l=k+fix(1.1*sign(da));
s=.2*abs(beta);
m=fix(s);
if (m==0)
    m=1;
end
if (m>=6)
    m=5;
end
db=s-m;
n=m+fix(1.1*sign(db));
l=l+3;
k=k+3;
m=m+1;
n=n+1;
t=a(k,m);
u=a(k,n);
v=t+abs(da)*(a(l,m)-t);
w=u+abs(da)*(a(l,n)-u);
value=v+(w-v)*abs(db);

function value=SnL_2D_interpolation1(a,alpha,el)
 s=.2*alpha;
k=fix(s);
if(k<=-2)
    k=-1;
end
if(k>=9)
    k=8;
end
da=s-k;
l=k+fix(1.1*sign(da));
s=el/12;
m=fix(s);
if(m<=-2)
    m=-1;
end
if(m>=2)
    m=1;
end
de=s-m;
n=m+fix(1.1*sign(de));
k=k+3;
l=l+3;
m=m+3;
n=n+3;
t=a(k,m);
u=a(k,n);
v=t+abs(da)*(a(l,m)-t);
w=u+abs(da)*(a(l,n)-u);
value=v+(w-v)*abs(de);

function value=SnL_2D_interpolation2(a,alpha,beta)
s=.2*alpha;
k=fix(s);
if(k<=-2)
    k=-1;
end
if(k>=9)
    k=8;
end
da=s-k;
l=k+fix(1.1*sign(da));
s=.1*beta;
m=fix(s);
if(m<=-3)
    m=-2;
end
if(m>=3)
    m=2;
end
db=s-m;
n=m+fix(1.1*sign(db));
l=l+3;
k=k+3;
m=m+4;
n=n+4;
t=a(k,m);
u=a(k,n);
v=t+abs(da)*(a(l,m)-t);
w=u+abs(da)*(a(l,n)-u);
value=v+(w-v)*abs(db);

function value1=SnL_1D_interpolation(A,alpha)
s=.2*alpha;
k=fix(s);
if(k<=-2)
    k=-1;
end
if(k>=9)
    k=8;
end
da=s-k;
l=k+fix(1.1*sign(da));
l=l+3;
k=k+3;
value1=A(k)+abs(da)*(A(l)-A(k));