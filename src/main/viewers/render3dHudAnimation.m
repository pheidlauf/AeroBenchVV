function [] = render3dHudAnimation(output_path, t_out, x_f16_out, ...
    u_ol_out, waypoints, WF_config, GCAS_config)
%
% Note: x1: North,   x2: East,   x3: Up

% If no GCAS_config provided, use default
if nargin < 7
    render_flight_deck = false;
else
    render_flight_deck = true;
end

% Extract Data to Animate
tf = t_out(end);
nz_out = u_ol_out(:,1);

% Unique: North, East, Up, phi, theta, psi
trajectory = x_f16_out(:,[11 10 12 4 5 6]);


% Start video object

if isunix
    % mp4 not supported by matlab on linux
    v = VideoWriter(strcat(output_path,'.avi'), 'Motion JPEG AVI');
else
    v = VideoWriter(strcat(output_path,'.mp4'), 'MPEG-4');
end

v.FrameRate = 30; % Use this to set speed of animation
v.Quality = 100;
open(v);
camZoomAngle = 60; % FOV

%% Initialize figure and fixed objects

% Make figure with proper dimensions for 1280x720 video
fig = figure('Name','HUD',...
    'Units','pixels',...
    'Position',[50 50 (50+1280) (50+720)]);

axis equal;
ax = gca;
hold on;
grid on;
xlabel('East (ft)');
ylabel('North (ft)');
zlabel('Alt (ft)');

%% Plot Circular Square Grid for ground
% Create meshgrid
gridR = 100000;
gridStep = 1000;
[xgrid,ygrid] = meshgrid(-gridR:gridStep:gridR,-gridR:gridStep:gridR);
zgrid = zeros(size(xgrid)); % Z grid for ground
% Create another zgrid for FLIGHT_DECK
if(render_flight_deck)
    zgrid_deck = ones(size(xgrid))*GCAS_config.FLIGHT_DECK;
end

% Set all points outside of circle to NaN
midPoint = length(xgrid)/2;
for m=1:length(xgrid)
    for n = 1:length(xgrid)
        if(sqrt((m-midPoint)^2 + (n-midPoint)^2 ) > midPoint)
            xgrid(m,n) = NaN;
            ygrid(m,n) = NaN;
            zgrid(m,n) = NaN;
            if(render_flight_deck)
                zgrid_deck(m,n) = NaN;
            end
        end
    end
end

% Designate RGB (from 0 to 1) colors
skyBlue = [135,206,250]/256;        % Sky Blue
earthBrown = [100,84,50]/256;       % Earth Brown
earthGreen = [61,84,47]/256;        % Earth Green

% Set background (aka, sky) to blue
whitebg(fig, skyBlue);

h_surf_ground = surf(xgrid,ygrid,zgrid,...
    'FaceColor',earthGreen,...
    'EdgeColor',earthBrown,...
    'FaceAlpha',1,...
    'EdgeAlpha',1);

if(render_flight_deck)
    h_surf_deck = surf(xgrid,ygrid,zgrid_deck,...
        'EdgeColor', 'red',...
        'FaceAlpha',0,...
        'EdgeAlpha',0.5);
end

% Plot flight path for debug
h_path = plot3(trajectory(:,1),trajectory(:,2),trajectory(:,3));

% Plot Global Coordinate Axis Arrows
coord_size = 50;
h_east = quiver3(0,0,0,coord_size,0,0);
h_east.AutoScaleFactor = 500;
h_east.LineWidth = 1;
h_north = quiver3(0,0,0,0,coord_size,0);
h_north.AutoScaleFactor = 500;
h_north.LineWidth = 1;
h_up = quiver3(0,0,0,0,0,coord_size);
h_up.AutoScaleFactor = 500;
h_up.LineWidth = 1;

%% Plot first frame of all changing objects
i = 1;

cam_pos = trajectory(i,1:3);
cam_eul = trajectory(i,4:6);

% Calculate aim point (unit vec in front of nose)
aimpoint = getCamAimPoint(x_f16_out(i,:));

%% Build scope strings
nz_str = sprintf('%2.1f',nz_out(i));
v_str = sprintf('%4.0f',x_f16_out(i,1));
alt_str = sprintf('%5.0f',x_f16_out(i,12));
time_str = sprintf('t = %5.2f sec',(i/length(x_f16_out))*tf);
if(round(rad2deg(cam_eul(3))+360) < 360) % Yes this is lazy
    head_str = sprintf('%3.0f',round(rad2deg(cam_eul(3))+360));
else
    head_str = sprintf('%3.0f',abs(rad2deg(cam_eul(3))));
end
pitch_str = sprintf('%3.0f',round(rad2deg(cam_eul(2))));
state_str = sprintf('\\phi = %5.1f   \\theta = %5.1f   \\psi = %5.1f',...
    rad2deg(cam_eul));

%% Plot Things
% Plot waypoints
plotWaypoints(waypoints, WF_config.WP_RANGE_THRESHOLD);

% Add G text
g_note = annotation('textbox','String',nz_str,'LineStyle','none');
g_note.VerticalAlignment = 'middle';
g_note.FontSize = 14;
g_note.FontName = 'FixedWidth';
g_note.Position(1:2) = [0.375 0.70];

% Add KCAS text
v_note = annotation('textbox','String',v_str,'FitBoxToText','on');
v_note.VerticalAlignment = 'middle';
v_note.FontSize = 14;
v_note.FontName = 'FixedWidth';
v_note.Position(1:2) = [0.30 0.5];

% Add Altitude text
alt_note = annotation('textbox','String',alt_str,'FitBoxToText','on');
alt_note.VerticalAlignment = 'middle';
alt_note.FontSize = 14;
alt_note.FontName = 'FixedWidth';
alt_note.Position(1:2) = [0.675 0.5];

% Add Heading Indicator
head_note = annotation('textbox','String',head_str,'FitBoxToText','on');
head_note.VerticalAlignment = 'middle';
head_note.FontSize = 14;
head_note.FontName = 'FixedWidth';
head_note.Position(1:2) = [0.5 0.8];

% Add Pitch Indicator
pitch_note = annotation('textbox','String',pitch_str,'LineStyle','none');
pitch_note.VerticalAlignment = 'middle';
pitch_note.FontSize = 14;
pitch_note.FontName = 'FixedWidth';
pitch_note.Position(1:2) = [(0.5) 0.30];
pitch_note.Visible = 'off';

% Add Current Time Text
time_note = annotation('textbox','String',time_str,'LineStyle','none');
time_note.VerticalAlignment = 'middle';
time_note.FontSize = 14;
time_note.FontName = 'FixedWidth';
time_note.Position(1:2) = [0.85 0.90];

% Add Full State Text
state_note = annotation('textbox','String',state_str,'LineStyle','none');
state_note.VerticalAlignment = 'middle';
state_note.FontSize = 14;
state_note.FontName = 'FixedWidth';
state_note.Position(1:2) = [0.375 0.05];

%% TEST
% All plots shown, lock axis as desired
axis off;
axis equal;

% Set Camera Position, Zoom, and Orientation
camproj('perspective');             % Not orthographic
campos(ax, cam_pos);                % Camera Position
camva(ax, camZoomAngle);            % Camera Field of view
camtarget(ax, aimpoint);          % Camera aim point
camroll(ax, rad2deg(cam_eul(1)));   % Camera (delta) roll, relative

% Record first frame
hold off;
frame = getframe(fig);
writeVideo(v,frame);

for i = 1:length(t_out)
    % Save roll angle of previous time step
    oldRoll = cam_eul(1);
    
    % Pull out all needed data at specified time step
    cam_pos = trajectory(i,1:3);
    cam_eul = trajectory(i,4:6);
    
    aimpoint = getCamAimPoint(x_f16_out(i,:));
    
    % Build scope strings
    nz_str = sprintf('%2.1f',nz_out(i));
    v_str = sprintf('%4.0f',x_f16_out(i,1));
    alt_str = sprintf('%5.0f',x_f16_out(i,12));
    %     SR_str = sprintf('SR = %5.0f ft',SlantRange2(i));
    time_str = sprintf('t = %5.2f sec',(i/length(x_f16_out))*tf);
    if(round(rad2deg(cam_eul(3))+360) < 360)
        head_str = sprintf('%3.0f',round(rad2deg(cam_eul(3))+360));
    else
        head_str = sprintf('%3.0f',abs(rad2deg(cam_eul(3))));
    end
    pitch_str = sprintf('%3.0f',round(rad2deg(cam_eul(2))));
    state_str = sprintf('\\phi = %5.1f   \\theta = %5.1f   \\psi = %5.1f',...
        rad2deg(cam_eul));
    
    % Update G text
    set(g_note,'String',nz_str);
    
    % Update KCAS text
    set(v_note,'String',v_str);
    
    % Update Altimeter text
    set(alt_note,'String',alt_str);
    
    % Update Heading Indicator
    set(head_note,'String',head_str);
    
    % Update Pitch Indicator
    set(pitch_note,'String',pitch_str);
    
    % Update Current Time text
    set(time_note,'String',time_str);
    
    % Update State text
    set(state_note,'String',state_str);
    
    % Update Camera Position, Zoom, Aimpoint, and Orientation
    campos(ax, cam_pos);
    camva(ax, camZoomAngle);
    camtarget(ax, aimpoint);
    camroll(ax, rad2deg(cam_eul(1)-oldRoll));
    
    % Record frame
    frame = getframe(fig);
    writeVideo(v,frame);
end

% Close video object
close(v);
close(fig);

end

function aimpoint = getCamAimPoint(x_f16)
% Given x_f16, get an aimpoint out the aircraft nose.
% Calculate aim point on ground

% Extract and label needed variables
phi = x_f16(4);     % Roll (rad)
theta = x_f16(5);   % Pitch (rad)
psi = x_f16(6);     % Yaw (rad)

N = x_f16(10);      % North displacement (ft)
E = x_f16(11);      % East displacement (ft)
alt = x_f16(12);    % Altitude (ft)

% Direction Cosine Matrices
% Note: F-16 coordinate frame is NRD: Nose-Right-Down
%dcm_f16_NED = angle2dcm( psi, theta, phi); % N-E-D frame to F-16 frame
dcm_f16_NED = attitude2dcm( psi, theta, phi); % N-E-D frame to F-16 frame

dcm_NED_f16 =  dcm_f16_NED'; % F-16 frame to N-E-D frame

% F-16 unit vector out the nose (x1) in F-16 frame [NRD]
nose_f16_NRD = [1; 0; 0];
nose_f16_NED = dcm_NED_f16*nose_f16_NRD; % F-16 x1 vector in [N-E-U]

% Convert F-16 nose vector from NED to ENU
noseUnitVec_ENU = diag([1 1 -1]) * nose_f16_NED([2 1 3]);

f16_pos_ENU = [E; N; alt];
aimpoint = f16_pos_ENU + noseUnitVec_ENU;

end
