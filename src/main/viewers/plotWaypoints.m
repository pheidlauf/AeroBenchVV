function plotWaypoints(waypoints, radius)
%PLOTPOINTS given an n x 3 array of waypoints and sphere radius, plots a
%sphere at each waypoint in the current axes

% Get unit sphere X Y Z for surf or mesh plotting
[spherex, spherey, spherez] = sphere();

% Scale up
spherex = spherex*radius;
spherey = spherey*radius;
spherez = spherez*radius;

for i = 1: size(waypoints,1)
    X = spherex + waypoints(i,1);
    Y = spherey + waypoints(i,2);
    Z = spherez + waypoints(i,3);
    
    mesh(X, Y, Z, ...
        'EdgeColor', 'green', ...
        'FaceAlpha','0.25')
end
end