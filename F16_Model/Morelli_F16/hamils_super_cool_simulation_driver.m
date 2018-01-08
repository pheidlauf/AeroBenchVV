x_trim=[500;0.0384498037408711;0;0;0.0384498037408711;0;0;0;0;0;0;500;8.96028070038678];
control_trim=[ 0.137977836470385;-0.751570978377749;0;0];
y_init=[x_trim;control_trim(2:4);0];
%%
constants.rtod=180/pi;
constants.ktheta=-3.0*constants.rtod;
constants.kv=1.0;
constants.kq=-.3*constants.rtod;
constants.kalpha=-.5*constants.rtod;
constants.kgp=-1.0; %0.0; case a value commented
constants.kgi=-5.0; %0.0; case a value commented
constants.kh=0.0;%-1.0; case a value commented
constants.kphi=-.5;
constants.kp=-.4*constants.rtod;
constants.gamp=4.5;
constants.phiamp=70.0;
simulation_end_time=10;
desired_velocity=x_trim(1);
desired_altitude=x_trim(12);
desired_alpha=x_trim(2);
desired_theta=x_trim(5);
%%
[t_hist,y_hist]=ode23(@(tt,yy) f16_setpoint_holder(tt,yy,desired_velocity,desired_altitude,desired_alpha,desired_theta,constants),[0 simulation_end_time],y_init);
%%
throttle_act_com=zeros(length(t_hist),1);
elevator_act_com=zeros(length(t_hist),1);
aileron_act_com=zeros(length(t_hist),1);
rudder_act_com=zeros(length(t_hist),1);
axout=zeros(length(t_hist),1);
ayout=zeros(length(t_hist),1);
azout=zeros(length(t_hist),1);
alphaxout=zeros(length(t_hist),1);
alphayout=zeros(length(t_hist),1);
alphazout=zeros(length(t_hist),1);
%%
for i=1:length(t_hist)
    [~,throttle_act_com(i),elevator_act_com(i),aileron_act_com(i),rudder_act_com(i)]=f16_setpoint_holder(t_hist(i),y_hist(i,:)',desired_velocity,desired_altitude,desired_alpha,desired_theta,constants);
end
