function plotStateflowModes(t_out, autopilot_mode_out, GCAS_mode_out, ...
    WF_mode_out, WF_iter_out, waypoints)

% Autopilot Modes
figure('Name','Autopilot Modes');
hold on;
grid on;
title('Autopilot Modes');
xlabel('Time (sec)');
ylabel('Mode');
hold on;

subplot(3,1,1)
plot(t_out, autopilot_mode_out);
legend('Autopilot Mode', ...
    'location', ...
    'NorthEast')

yticklabels({'Waypoint Follower'; 'GCAS'; 'Other'});
yticks(0:(size(yticklabels,1)-1));

% GCAS Modes
hold on;
grid on;
xlabel('Time (sec)');
ylabel('Mode');
subplot(3,1,2)
plot(t_out, GCAS_mode_out);
legend('GCAS Mode', ...
    'location', ...
    'NorthEast')

yticklabels({'SAFE'; 'ROLL LEVEL'; 'PULL UP' });
yticks(0:(size(yticklabels,1)-1));

% Waypoint Follower
hold on;
grid on;
xlabel('Time (sec)');
ylabel('Mode');
subplot(3,1,3)

% Waypoint Iter
plot(t_out, WF_iter_out);
waypointLabels = cell(size(waypoints,1)+1,1);
for i = 1:size(waypoints,1)
   waypointLabels{i} = sprintf('Follow WP %i', i);
end
waypointLabels{end} = 'COMPLETE';

yticklabels(waypointLabels);
yticks(1:(size(yticklabels,1)));

legend('Waypoint Follower', ...
    'location', ...
    'SouthEast')

end
