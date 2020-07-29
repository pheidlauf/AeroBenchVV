function plotWaypointCmds(t_out, x_f16_out, psi_cmd_out, ...
    WF_mode_out, WF_iter_out, waypoints)

% Waypoint Cmds
figure('Name','Waypoint Commands');
grid on;
subplot(2,1,1)
hold on;
plot(t_out, wrapTo360(rad2deg(psi_cmd_out)),'r');
plot(t_out, wrapTo360(rad2deg(x_f16_out(:,6))),'r--');

title('Waypoint Heading Command');
xlabel('Time (sec)');
ylabel('\psi_{cmd} (deg)');
axis([t_out(1) t_out(end) 0 360])

legend('Commanded','Actual', ...
    'location', ...
    'SouthEast')

% Waypoint Iter
hold on;
subplot(2,1,2);

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
