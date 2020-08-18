% Simple graph of simulation result

disp('Graphing Output');

if size(t_out, 1) > 0

    % x / y position
    figure(6);
    clf
    hold on;
    grid on;
    title('2D Position');
    xlabel('East / West (ft)');
    ylabel('North / South (ft)');
    hold on;
    plot(x_f16_out(:,11), x_f16_out(:,10),'k');       % x/y position
    
    plot(waypoints(:, 1), waypoints(:, 2), 'rx');
    
    axis equal
    legend('Aircraft Position', 'location','SouthEast')
    hold off
    
    % Attitude
    figure(2);
    clf
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
    
    % Altitude
    figure(3);
    clf
    subplot(3,1,1)
    hold on;
    grid on;
    title('Altitude History');
    xlabel('Time (sec)');
    ylabel('Altitude (ft)');
    hold on;
    plot(t_out, x_f16_out(:,12), 'b--');     % alt
    legend('Alt (ft)', ...
        'location','SouthEast')
    
    % GCAS Mode
    subplot(3,1,2)
    hold on;
    grid on;
    title('GCAS Mode History');
    xlabel('Time (sec)');
    ylabel('GCAS Mode');
    hold on;
    plot(t_out, GCAS_mode_out, 'b--');
    legend('GCAS Mode', ...
        'location','SouthEast')
    
    % Waypoint Index
    subplot(3,1,3)
    hold on;
    grid on;
    title('Waypoint History');
    xlabel('Time (sec)');
    ylabel('Waypoint Index');
    hold on;
    plot(t_out, WF_iter_out, 'b--');  
    legend('Waypoiny Index', ...
        'location','SouthEast')

else
    disp('No Data from simulation run?');
end