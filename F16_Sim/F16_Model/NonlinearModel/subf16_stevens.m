function [xd, Nz, Ny] = subf16Fun(x,u,xcg)
%
%  xd = subf16(x,u)
%
%  This routine outputs the state vector derivative, xd, for the computer 
%  model of an F-16 aircraft.
%  This routine is called by the trimmer and jacob m-files.
%  Prior to running, this m-file must be edited to set the proper 
%  c.g. location (see 'xcg = ' below).
%  The inputs are control vector u, and the state vector x:
%
%         x(1) = air speed, VT                              (ft/sec)
%         x(2) = angle of attack, alpha                     (rad)
%         x(3) = angle of sideslip, beta                    (rad)
%         x(4) = roll angle, phi                            (rad)
%         x(5) = pitch angle, theta                         (rad)
%         x(6) = yaw angle, psi                             (rad)
%         x(7) = roll rate, P                               (rad/sec)
%         x(8) = pitch rate, Q                              (rad/sec)
%         x(9) = yaw rate, R                                (rad/sec)
%         x(10) = northward horizontal displacement, pn     (feet)
%         x(11) = eastward horizontal displacement, pe      (feet)
%         x(12) = altitude, h                               (feet)
%         x(13) = engine thrust dynamics lag state, pow     (lbs?)
%
%         u(1) = throttle command  0.0 < u(1) < 1.0         (percent/100)
%         u(2) = elevator command                           (deg)
%         u(3) = aileron command                            (deg)
%         u(4) = rudder command                             (deg)
%

global az ay;

% The following is the c.g. location which can be modified (nominal is xcg=.35)
%% BEGIN HEIDLAUF EDIT
if(nargin==2)
    xcg = 0.35;
end

%% END HEIDLAUF EDIT

thtlc=u(1);
el=u(2);
ail=u(3);
rdr=u(4);

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

xd=x;
vt=x(1);
alpha=x(2)*rtod;
beta=x(3)*rtod;
phi=x(4);
theta=x(5);
psi=x(6);
p=x(7);
q=x(8);
r=x(9);
alt=x(12);
pow=x(13);
[amach,qbar]=adc(vt,alt);
cpow=tgear(thtlc);
xd(13)=pdot(pow,cpow);
t=thrust(pow,alt,amach);

cxt=cx(alpha,el);
cyt=cy(beta,ail,rdr);
czt=cz(alpha,beta,el);
dail=ail/20;drdr=rdr/30;
clt=cl(alpha,beta)+dlda(alpha,beta)*dail+dldr(alpha,beta)*drdr;
cmt=cm(alpha,el);
cnt=cn(alpha,beta)+dnda(alpha,beta)*dail+dndr(alpha,beta)*drdr;
tvt=.5/vt;b2v=b*tvt;cq=cbar*q*tvt;
d=dampp(alpha);
cxt=cxt+cq*d(1);
cyt=cyt+b2v*(d(2)*r+d(3)*p);
czt=czt+cq*d(4);
clt=clt+b2v*(d(5)*r+d(6)*p);
cmt=cmt+cq*d(7)+czt*(xcgr-xcg);
cnt=cnt+b2v*(d(8)*r+d(9)*p)-cyt*(xcgr-xcg)*cbar/b;
cbta=cos(x(3));u=vt*cos(x(2))*cbta;
v=vt*sin(x(3));w=vt*sin(x(2))*cbta;
sth=sin(theta);cth=cos(theta);sph=sin(phi);
cph=cos(phi);spsi=sin(psi);cpsi=cos(psi);
qs=qbar*s;qsb=qs*b;rmqs=rm*qs;
gcth=g*cth;qsph=q*sph;
ay=rmqs*cyt;az=rmqs*czt;
udot=r*v-q*w-g*sth+rm*(qs*cxt+t);
vdot=p*w-r*u+gcth*sph+ay;
wdot=q*u-p*v+gcth*cph+az;
dum=(u*u+w*w);
xd(1)=(u*udot+v*vdot+w*wdot)/vt;
xd(2)=(u*wdot-w*udot)/dum;
xd(3)=(vt*vdot-v*xd(1))*cbta/dum;
xd(4)=p+(sth/cth)*(qsph+r*cph);
xd(5)=q*cph-r*sph;
xd(6)=(qsph+r*cph)/cth;
xd(7)=(c2*p+c1*r+c4*he)*q+qsb*(c3*clt+c4*cnt);
xd(8)=(c5*p-c7*he)*r+c6*(r*r-p*p)+qs*cbar*c7*cmt;
xd(9)=(c8*p-c2*r+c9*he)*q+qsb*(c4*clt+c9*cnt);
t1=sph*cpsi;t2=cph*sth;t3=sph*spsi;
s1=cth*cpsi;s2=cth*spsi;s3=t1*sth-cph*spsi;
s4=t3*sth+cph*cpsi;s5=sph*cth;s6=t2*cpsi+t3;
s7=t2*spsi-t1;s8=cph*cth;
xd(10)=u*s1+v*s3+w*s6;
xd(11)=u*s2+v*s4+w*s7;
xd(12)=u*sth-v*s5-w*s8;
xa=15.0;                  % sets distance normal accel is in front of the c.g. (xa=15.0 at pilot)
az=az-xa*xd(8);           % moves normal accel in front of c.g...
% ADDED BY HEIDLAUF
ay=ay+xa*xd(9);           % moves side accel in front of c.g.

% For extraction of Nz
Nz = (-az/g) - 1; % Zeroed at 1 g, positive g = pulling up
Ny = ay/g;
end