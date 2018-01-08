% Using the flypath3d package, this generates a 3d graphic visualizing 
% the relationship of the aircraft attitude, East-North-Up global axis, 
% and the body frame.
%
% <a href="https://github.com/pheidlauf/AeroBenchVV">AeroBenchVV</a>
% Copyright: GNU General Public License 2017
%
% See also: NEW_OBJECT, FLYPATH

% Enable flypath3d & initialize
cd('../flypath3d');
package_setup;
cd('../F16_GCAS_V2');
close all; clear; clc;



%  x_f16 States:
%       x_f16(1)  = air speed, VT (300 ft/s to 900 ft/s)        (ft/s)
%       x_f16(2)  = angle of attack, alpha (-0.174 to 0.785)    (rad)
%       x_f16(3)  = angle of sideslip, beta                     (rad)
%       x_f16(4)  = roll angle, phi                             (rad)
%       x_f16(5)  = pitch angle, theta                          (rad)
%       x_f16(6)  = yaw angle, psi                              (rad)
%       x_f16(7)  = roll rate, P                                (rad/s)
%       x_f16(8)  = pitch rate, Q                               (rad/s)
%       x_f16(9)  = yaw rate, R                                 (rad/s)
%       x_f16(10) = northward horizontal displacement, pn       (ft)
%       x_f16(11) = eastward horizontal displacement, pe        (ft)
%       x_f16(12) = altitude, h                                 (ft)
%       x_f16(13) = engine thrust dynamics lag state, pow       (lbf)

%% Create Image of F-16 with body & global coordinate axes defined
% data = [E N U Pitch Yaw Roll]
pEast = 100;
pNorth = 200;
h = 300;

% Define attitude of aircraft
psi = deg2rad(90);       % Yaw 
theta = deg2rad(-45);    % Pitch
phi = deg2rad(30);       % Roll

% Build data vector for flypath
data = ones(10,1)*[pEast pNorth h theta psi phi];

% Flypath3D Trajectory Visualization

% create an object
new_object('aircraft.mat',data,...
    'model','f-16.mat',...
    'alpha',1,...
    'edge',[0.25 0.25 0.25],...
    'path','off',...
    'scale', 10);
% aircraft trajectory visualization
flypath('aircraft.mat',...
    'step',1,...
    'axis','on','axiscolor',[0 0 0],'color',[1 1 1],...
    'font','Georgia','fontsize',6,...
    'view', [-37.5, 30],...
    'window',[1000 1000]);

hold on;


% Draw altitude floor
ground = 0;
xl = round(xlim);
yl = round(ylim);

[x_mesh,y_mesh] = meshgrid(-1000:100:1000,-1000:100:1000);
z_mesh = ones(size(x_mesh))*ground;
h_surf = surf(x_mesh,y_mesh,z_mesh,...
    'FaceColor',[.8 .8 .8],...
    'EdgeColor',[.7 .7 .7],...
    'FaceAlpha',1,...                                 
    'EdgeAlpha',1);

% Build Vectors:
dcm = angle2dcm( -psi, theta, phi, 'ZXY');

x = [0 0 0 pEast pEast pEast];
y = [0 0 0 pNorth pNorth pNorth];
z = [0 0 0 h h h];
u = [1 0 0 dcm(:,1)'];
v = [0 1 0 dcm(:,2)'];
w = [0 0 1 dcm(:,3)'];

% Flip Z-axis
u(6) = -u(6);
v(6) = -v(6);
w(6) = -w(6);

plot3(data(:,1),data(:,2),data(:,3)*0,'red','LineWidth',2,...
    'Marker','.','MarkerSize',20)

% Define colors: 'b y r b y r' for plotting
colorMat = [0 0 1;  % Blue
    1 0.75 0;       % Yellow
    1 0 0;          % Red
    1 0.75 0;       % Yellow
    0 0 1;          % Blue
    1 0 0];         % Red

for i=1:length(x)
    quiver3(x(i),y(i),z(i),u(i),v(i),w(i),100,...
        'Color', colorMat(i,:),'LineWidth',2) 
end

% Set Plot Axes
xlim([min([0,x])-100 max([0,x])+100]);
ylim([min([0,y])-100 max([0,y])+100]);
zlim([0 max(z)+100]);
xlabel('East (ft)');
ylabel('North (ft)');
zlabel('Up (ft)');


% Add title with info
str1 = sprintf('[p_{East} p_{North} h] = [ %5.1f, %5.1f, %5.1f ] ft ',...
    pEast, pNorth, h);
str2 = sprintf('[\\phi \\theta \\psi] = [ %5.1f, %5.1f, %5.1f ] deg ',...
    rad2deg(phi(1)),...
    rad2deg(theta(1)),...
    rad2deg(psi(1)));
        
titlestr = {str1, str2};
title(titlestr, 'FontSize', 16, 'FontName','FixedWidth');

% Save Figure
% print('output_pic','-dpng')