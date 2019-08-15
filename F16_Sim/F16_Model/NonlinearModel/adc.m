function [amach,qbar]=adc(vt,alt)
ro=2.377e-3;
tfac=1-.703e-5*alt;
t=519*tfac;
if(alt >=35000), t=390; end
rho=ro*(tfac^4.14);
amach=vt/sqrt(1.4*1716.3*t);
qbar=.5*rho*vt*vt;
