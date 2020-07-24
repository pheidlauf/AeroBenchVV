function plotAircraftStates(t_out, x_f16_out, u_il_out, u_ol_out, ...
    u_ol_ref_out)

% Attitude
figure('Name','F-16 Attitude');
hold on;
grid on;
title('Attitude History');
xlabel('Time (sec)');
ylabel('Attitudes & Rates (deg, deg/s)');
hold on;
plot(t_out, rad2deg(x_f16_out(:,4)),'r');       % phi (deg)
plot(t_out, rad2deg(x_f16_out(:,5)),'g');       % theta (deg)
plot(t_out, rad2deg(x_f16_out(:,6)),'b');       % psi (deg)
plot(t_out, rad2deg(x_f16_out(:,7)),'r--');     % p (deg/s)
plot(t_out, rad2deg(x_f16_out(:,8)),'g--');     % q (deg/s)
plot(t_out, rad2deg(x_f16_out(:,9)),'b--');     % r (deg/s)
legend('Roll \phi',' Pitch \theta','Yaw \psi',...
    'Roll Rate p','Pitch Rate q','Yaw rate r',...
    'location','SouthEast')

% Inner Loop Control Signals
figure('Name','Inner Loop Controls');
hold on;
grid on;
title('F-16 Control');
xlabel('Time (sec)');
ylabel('Control (deg & percent)');
hold on;
plot(t_out, u_il_out(:,1));
plot(t_out, (u_il_out(:,2)));
plot(t_out, (u_il_out(:,3)));
plot(t_out, (u_il_out(:,4)));
legend('Throttle','Elevator','Aileron','Rudder','location',...
    'NorthWest')

% Outer Loop Control Signals
figure('Name','Outer Loop Controls');
hold on;
grid on;
title('Autopilot Control');
xlabel('Time (sec)');
ylabel('Autopilot (deg & percent)');
hold on;
plot(t_out, u_ol_out(:,1),'r');
plot(t_out, u_ol_ref_out(:,1),'r--');
plot(t_out, u_ol_out(:,2),'g');
plot(t_out, u_ol_ref_out(:,2),'g--');
plot(t_out, u_ol_out(:,3),'b');
plot(t_out, u_ol_ref_out(:,3),'b--');
plot(t_out, u_ol_out(:,4),'c');

legend('N_z', 'N_{z,ref}', 'p_s', 'p_{s,ref}', 'N_{yr}',...
    'N_{yr,ref}', 'Throttle',...
    'location', 'NorthWest')

end
