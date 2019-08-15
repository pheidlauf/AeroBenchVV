%
% FLYPATH (flypath.m)
%
% Displays static and animated trajectories of missiles and air targets
% Last updated: 2016-09-03
%
% SYNTAX:
%
%    flypath(object_1,object_2,...,object_n,varargin)
%
% PARAMETERS:
%
%    object_x    : object data set produced by 'new_object' function,
%                  use command 'help new_object' to get more information
%
% OPTIONAL ARGUMENTS:
%
%    'animate'   : animation on/off ('on','off' - default 'off')
%    'axis'      : axes visibility ('on','off' - default 'on')
%    'axiscolor' : axes and axes font color ([R G B] - default [0 0 0])
%    'color'     : display area color ([R G B] - default [1 1 1])
%    'dpi'       : dpi value (75,150,300,600 - default 150)
%    'font'      : font name (string - default 'Times New Roman')
%    'fontsize'  : font size (default 12)
%    'output'    : output file name (string - default 'none')
%    'step'      : model repetition density (default 10)
%    'view'      : camera view angles ([azimuth elevation] - default [15 30])
%    'window'    : display area size ([width height] - default [800 600])
%    'xlim'      : x axes limits ([min max] or 'off' - default 'off')
%    'ylim'      : y axes limits ([min max] or 'off' - default 'off')
%    'zlim'      : x axes limits ([min max] or 'off' - default 'off')
%
% EXAMPLES OF USE:
%
%    trajectory = load('trajectory_tbm.mat');
%    new_object('ballistic_missile.mat',trajectory,...
%               'model','scud.mat','scale',5,...
%               'path','on','pathcolor',[.89 .0 .27]);
%    flypath('ballistic_missile.mat',...
%               'animate','off','step',30,...
%               'axis','on','axiscolor',[0 0 0],'color',[1 1 1],...
%               'font','Georgia','fontsize',6,...
%               'view',[0 0],'window',[1800 600],...
%               'xlim',[-2e4 16e4],'ylim',[-10 10],'zlim',[0 4e4],...
%               'output','tbm_example.png','dpi',600);
%

function flypath(varargin)

objCount = 0;

% --- check input parameters ---
if nargin < 1
    error('Not enough input parameters!');
end;

% --- default input parameters ---
pAnimate = 'off';			 % animation: 'on','off'
pAxis = 'on';              % axes visibility: 'on','off'
pAxisColor = [ 0 0 0 ];    % axes and axes font color: [ R G B ]
pColor = [ 1 1 1 ];        % display area color: [ R G B ]
pDpi = 150;                % dpi value: 75, 150, 300, 600
pFont = 'Times New Roman'; % font name
pFontSize = 12;            % font size
pOutput = 'none';          % output filename or 'none'
pStep = 10;                % model repetition density
pView = [ 15 30 ];		 % camera view angles: [ azimuth elevation ]
pWindow = [ 800 600 ];	 % display area size (in pixels)
pXLim = 'off';             % x axes limits ([min max] or 'off' - default 'off')
pYLim = 'off';             % y axes limits ([min max] or 'off' - default 'off')
pZLim = 'off';             % z axes limits ([min max] or 'off' - default 'off')
plotGround = false;

% --- read input parameters ---
i = 1;
while i <= length(varargin)
    switch lower(varargin{i})
        case 'animate'
            pAnimate = varargin{i+1};
            i = i + 2;
        case 'axis'
            pAxis = varargin{i+1};
            i = i + 2;
        case 'axiscolor'
            pAxisColor = varargin{i+1};
            i = i + 2;
        case 'color'
            pColor = varargin{i+1};
            i = i + 2;
        case 'dpi'
            pDpi = varargin{i+1};
            i = i + 2;
        case 'font'
            pFont = varargin{i+1};
            i = i + 2;
        case 'fontsize'
            pFontSize = varargin{i+1};
            i = i + 2;
        case 'output'
            pOutput = varargin{i+1};
            i = i + 2;
        case 'step'
            pStep = varargin{i+1};
            i = i + 2;
        case 'view'
            pView = varargin{i+1};
            i = i + 2;
        case 'window'
            pWindow = varargin{i+1};
            i = i + 2;
        case 'xlim'
            pXLim = varargin{i+1};
            i = i + 2;
        case 'ylim'
            pYLim = varargin{i+1};
            i = i + 2;
        case 'zlim'
            pZLim = varargin{i+1};
            i = i + 2;
        case 'tvec'
            tvec = varargin{i+1};
            i = i + 2;
        case 'ground'
            ground = varargin{i+1};
            i = i + 2;
            plotGround = true;
        case 'zoom'
            zoomWidth = varargin{i+1};
            zoom = true;
            i = i + 2;
        case 'telemetry'
            telemetry = varargin{i+1};
            fullTelemetry = true;
            i = i + 2;
        otherwise
            if exist(varargin{i},'file') == 2
                objCount = objCount + 1;
            else
                error('file %s does not exist!',varargin{i});
            end
            i = i + 1;
    end
end;

% --- object loader ---
i = 1;
while i <= objCount
    % --- load data to tmp variable ---
    tmp = load(varargin{i});
    % --- add data to obj structure ---
    if size(tmp.pMatrix,1) < size(tmp.pMatrix,2)
        obj(i).matrix = tmp.pMatrix';
    else
        obj(i).matrix = tmp.pMatrix;
    end;
    obj(i).face = tmp.pFace;
    obj(i).edge = tmp.pEdge;
    obj(i).alpha = tmp.pAlpha;
    obj(i).path = tmp.pPath;
    obj(i).pathcolor = tmp.pPathColor;
    obj(i).pathwidth = tmp.pPathWidth;
    obj(i).scale = tmp.pScale;
    % --- add 3d model to the structure ---
    tmp = load(tmp.pModel);
    obj(i).v(:,1) = -1 * obj(i).scale * tmp.V(:,1);
    obj(i).v(:,2) = 1 * obj(i).scale * tmp.V(:,2);
    obj(i).v(:,3) = 1 * obj(i).scale * tmp.V(:,3);
    obj(i).f = tmp.F;
    % --- increment counter ---
    i = i + 1;
end;

% --- check matrix sizes ---
sizeCounter = 0;
maxLength = size(obj(1).matrix,1);
if objCount > 1
    i = 2;
    while i <= objCount
        if size(obj(i).matrix,1) ~= maxLength
            sizeCounter = sizeCounter + 1;
        end;
        i = i + 1;
    end;
    if sizeCounter > 0
        warning('data array sizes are different!');
        warning('data cutted to the smallest matrix dimension');
        i = 1;
        maxLength = 9e100;
        while i <= objCount
            if size(obj(i).matrix,1) < maxLength
                maxLength = size(obj(i).matrix,1);
            end
            i = i + 1;
        end;
    end;
end

% --- check step size ---
if pStep >= maxLength
    warning('step size is equal or greather than max length of data array!');
    warning('default step value used');
    pStep = 10;
end;

% --- prepare display area ---
fig = figure(1);
set(fig,...
    'Name','flypath3d',...
    'NumberTitle','off',...
    'MenuBar','none',...
    'ToolBar','figure',...
    'Pointer','crosshair',...
    'Position',[50 50 pWindow(1) pWindow(2)],...
    'Color',pColor);

set(gca,...
    'FontName',pFont,...
    'Color',pColor,...
    'XColor',pAxisColor,...
    'YColor',pAxisColor,...
    'ZColor',pAxisColor,...
    'FontSize',pFontSize);
clf(fig);
hold on;


% --- animate ---
for i = 1:pStep:maxLength
    % --- prepare new frame for animation ---
    if strcmp(pAnimate,'on') == 1
        clf(fig);
        hold on;
    end;
    set(gca,...
        'FontName',pFont,...
        'Color',pColor,...
        'XColor',pAxisColor,...
        'YColor',pAxisColor,...
        'ZColor',pAxisColor,...
        'FontSize',pFontSize);
    
    %% Heidlauf Edits
    
    % Add telemetry at each time step to the title
    if(fullTelemetry)
        % Time
        t_maneuver = telemetry.t_maneuver;
        if(tvec(i) < t_maneuver(1))
            titlestr1 = sprintf('t = %7.2f sec   Waiting          ', tvec(i));
        elseif(t_maneuver(2) < 0 || tvec(i) < t_maneuver(2))
            titlestr1 = sprintf('t = %7.2f sec   Maneuvering      ', tvec(i));
        elseif(t_maneuver(2) > 0 && tvec(i) >= t_maneuver(2))
            % If maneuver time is a PASS/FAIL, print PASS or FAIL
            if(telemetry.passFail.maneuverTime)
                titlestr1 = sprintf('t = %7.2f sec   {\\color{green}Complete}         ', tvec(i));
            else
                titlestr1 = sprintf('t = %7.2f sec   {\\color{red}Complete}         ', tvec(i));
            end
        else
            % Just in case
            % titlestr1 = sprintf('t = %7.2f sec                    ', tvec(i));
            warning('Weird issue with t_maneuver in my_flypath');
        end
        
        % Altitude & Velocity
        if(obj(1).matrix(i,3) > 0 && ...
                telemetry.states.VT(i) > telemetry.flightLimits.vMin && ...
                telemetry.states.VT(i) < telemetry.flightLimits.vMax)
            titlestr2 = sprintf('h = %7.2f ft    V = %7.2f ft/s ',...
                obj(1).matrix(i,3),telemetry.states.VT(i));
        elseif(telemetry.states.VT(i) > telemetry.flightLimits.vMin && ...
                telemetry.states.VT(i) < telemetry.flightLimits.vMax)
            titlestr2 = sprintf('{\\color{red}h = %7.2f ft}    V = %7.2f ft/s ',...
                obj(1).matrix(i,3),telemetry.states.VT(i));
        elseif(obj(1).matrix(i,3) > 0)
            titlestr2 = sprintf('h = %7.2f ft    {\\color{red}V = %7.2f ft/s} ',...
                obj(1).matrix(i,3),telemetry.states.VT(i));
        else
            titlestr2 = sprintf('{\\color{red}h = %7.2f ft    V = %7.2f ft/s} ',...
                obj(1).matrix(i,3),telemetry.states.VT(i));
        end
        
        % Alpha & Beta
        alpha = rad2deg(telemetry.states.alpha(i));
        beta = rad2deg(telemetry.states.beta(i));
        if(alpha < telemetry.flightLimits.alphaMaxDeg && ...
                alpha > telemetry.flightLimits.alphaMinDeg && ...
                abs(beta) < telemetry.flightLimits.betaMaxDeg)
            % Alpha & Beta Pass
            titlestr3 = sprintf('\\alpha = %7.2f deg   \\beta = %7.2f deg  ',...
                alpha,beta);
        elseif(alpha < telemetry.flightLimits.alphaMaxDeg && ...
                alpha > telemetry.flightLimits.alphaMinDeg)
            % Beta Fail
            titlestr3 = sprintf('\\alpha = %7.2f deg   {\\color{red}\\beta = %7.2f deg}  ',...
                alpha,beta);
        elseif(abs(beta) < telemetry.flightLimits.betaMinDeg)
            % Alpha Fail
             titlestr3 = sprintf('{\\color{red}\\alpha = %7.2f deg}   \\beta = %7.2f deg  ',...
                alpha,beta);
        else
            % Alpha & Beta Fail
            titlestr3 = sprintf('{\\color{red}\\alpha = %7.2f deg   \\beta = %7.2f deg}  ',...
                alpha,beta);
        end
        
        % Nz & P
        Nz = telemetry.states.Nz_hist(i);
        ps = rad2deg(telemetry.states.ps_hist(i));
        if(Nz < telemetry.flightLimits.NzMax && ...
                Nz > telemetry.flightLimits.NzMin)
            % Nz Pass
            titlestr4 = sprintf('N_z = %7.2f g     p_s= %7.2f deg/s ',...
                Nz,ps);
        else
            % Nz Fail
            titlestr4 = sprintf('{\\color{red}N_z = %7.2f g}     p_s= %7.2f deg/s ',...
                Nz,ps);
        end
        
        
        % Roll Pitch & Yaw?
        phi = rad2deg(telemetry.states.phi(i));
        theta = rad2deg(telemetry.states.theta(i));
        psi = rad2deg(telemetry.states.psi(i));
        titlestr5 = sprintf('[\\phi \\theta \\psi] = [ %5.1f, %5.1f, %5.1f ] deg ',...
                phi,theta,psi);
        
        % Position
        e_pos = obj(1).matrix(i,1);
        n_pos = obj(1).matrix(i,2);
        alt = obj(1).matrix(i,3);
        titlestr6 = sprintf('[E N h] = [ %5.0f, %5.0f, %5.0f ] ft ',...
            e_pos, n_pos, alt);
        
        
        titlestr = {titlestr1,titlestr2,titlestr3,titlestr4,titlestr5,...
            titlestr6};
    else
        % Use only "flypath" provided data
        titlestr1 = sprintf('t = %7.2f sec', tvec(i));
        if(obj(1).matrix(i,3) > 0)
            titlestr2 = sprintf('h = %7.2f ft ', obj(1).matrix(i,3));
        else
            titlestr2 = sprintf('{\\color{red}h = %7.2f ft} ',...
                obj(1).matrix(i,3));
        end
        titlestr3 = sprintf('\\phi = %7.2f deg',...
            rad2deg(obj(1).matrix(i,6)));
        titlestr4 = sprintf('\\theta = %7.2f deg',...
            rad2deg(obj(1).matrix(i,4)));
        titlestr5 = sprintf('\\psi = %7.2f deg',...
            rad2deg(obj(1).matrix(i,5)));
        titlestr = {titlestr1,titlestr2,titlestr3,titlestr4,titlestr5};
    end
    title(titlestr,...
        'FontSize',16,...
        'FontName','FixedWidth');
    
    %% Original flypath
    % --- draw scene ---
    for j = 1:1:objCount
        theta = obj(j).matrix(i,4);
        psi = obj(j).matrix(i,5) - pi/2;
        phi = -1 * obj(j).matrix(i,6);
        % --- transformation matrix ---
        sinTheta = sin(theta);
        cosTheta = cos(theta);
        sinPsi = sin(psi);
        cosPsi = cos(psi);
        sinPhi = sin(phi);
        cosPhi = cos(phi);
        transformMatrix = [ ...
            cosPsi * cosTheta ...
            -sinPsi * cosTheta ...
            sinTheta; ...
            cosPsi * sinTheta * sinPhi + sinPsi * cosPhi ...
            -sinPsi * sinTheta * sinPhi + cosPsi * cosPhi ...
            -cosTheta * sinPhi; ...
            -cosPsi * sinTheta * cosPhi + sinPsi * sinPhi ...
            sinPsi * sinTheta * cosPhi + cosPsi * sinPhi ...
            cosTheta * cosPhi ];
        vertices = obj(j).v * transformMatrix;
        objectSet = [ obj(j).matrix(i,1) obj(j).matrix(i,2) obj(j).matrix(i,3) ];
        delta = repmat( objectSet, size( vertices, 1 ), 1 );
        vertices = vertices + delta;
        
        objPath = patch( 'faces', obj(j).f, 'vertices', vertices );
        set(objPath, ...
            'FaceColor',obj(j).face,...
            'FaceAlpha',obj(j).alpha,...
            'EdgeColor',obj(j).edge );
        if strcmp(obj(j).path,'on') == 1
            plot3(obj(j).matrix(1:i,1),...
                obj(j).matrix(1:i,2),...
                obj(j).matrix(1:i,3),...
                'Color', obj(j).pathcolor, ...
                'LineWidth', obj(j).pathwidth, ...
                'LineStyle', '-');
        end;
    end;
    
    % --- display parameters ---
    if strcmp(pXLim,'off') ~= 1
        xlim( pXLim );
    end
    if strcmp(pYLim,'off') ~= 1
        ylim( pYLim );
    end
    if strcmp(pZLim,'off') ~= 1
        zlim( pZLim );
    end
    xlabel('East (ft)');
    ylabel('North (ft)');
    zlabel('Altitude (ft)');
    
    %% Heidlauf Edits
    xl = (xlim);
    yl = (ylim);
    zl = (zlim);
    
    if(plotGround)
        % Draw altitude floor
        if(zoom && strcmp(zoomWidth,'full'))
            xl(1) = min(obj(j).matrix(:,1));
            yl(1) = min(obj(j).matrix(:,2));
            xl(2) = max(obj(j).matrix(:,1));
            yl(2) = max(obj(j).matrix(:,2));
        elseif(zoom)
            xl(1) = min(obj(j).matrix(:,1))-zoomWidth/2;
            yl(1) = min(obj(j).matrix(:,2))-zoomWidth/2;
            xl(2) = max(obj(j).matrix(:,1))+zoomWidth/2;
            yl(2) = max(obj(j).matrix(:,2))+zoomWidth/2;
        end        
        [x_mesh,y_mesh] = meshgrid(xl(1):100:xl(2),yl(1):100:yl(2));
        z_mesh = ones(size(x_mesh))*ground;
        surf(x_mesh,y_mesh,z_mesh,...
            'FaceColor',[.8 .8 .8],...
            'EdgeColor',[.7 .7 .7],...
            'FaceAlpha',1,...
            'EdgeAlpha',1);
    end
     

    % Handle zoom
    if(zoom && strcmp(zoomWidth,'full'))
        x = obj(j).matrix(i,1);
        y = obj(j).matrix(i,2);
        z = obj(j).matrix(i,3);
    elseif(zoom)
        x = obj(j).matrix(i,1);
        y = obj(j).matrix(i,2);
        z = obj(j).matrix(i,3);
        xlim([x-zoomWidth/2,x+zoomWidth/2]);
        ylim([y-zoomWidth/2,y+zoomWidth/2]);
        zlim([z-zoomWidth/2,z+zoomWidth/2]);
    else
        xlim(xl);
        ylim(yl);
        zlim(zl);
    end

    
    %% Original flypath
    
    if strcmp(pAxis,'on') == 1
        axis on;
        axis square;
        grid on;
    else
        axis off;
        axis square;
        grid off;
    end;
    lighting phong;
    daspect([1 1 1]);
    view( pView );
    if strcmp(pAnimate,'on') == 1
        drawnow;
    end;
    %{
    % BACKUP OF ORIGINAL GIF CODE
    % --- save the results to a .gif file ---
    if strcmp(pAnimate,'on') == 1
        if strcmp(pOutput,'none') ~= 1
            frame = getframe(1);
            im = frame2im(frame);
            [A,map] = rgb2ind(im,256);
            if i == 1;
                imwrite(A,map,pOutput,'gif','LoopCount',Inf,'DelayTime',.04);
            else
                imwrite(A,map,pOutput,'gif','WriteMode','append','DelayTime',.04);
            end;
        end;
    end;
    %}
    % NEW CODE TO SAVE AS MP4
    % --- save the results to a .mp4 file ---
    if strcmp(pAnimate,'on') == 1
        if strcmp(pOutput,'none') ~= 1
            frame = getframe(1);
            if i == 1;                
                % Start video object
                v = VideoWriter(pOutput,'MPEG-4');
                v.FrameRate = 30; % Use this to set speed of animation
                v.Quality = 100;
                open(v);
                writeVideo(v,frame);
            else
                writeVideo(v,frame);
            end;
        end;
    end;
end;
if strcmp(pAnimate,'off') == 1
    if strcmp(pOutput,'none') ~= 1
        switch(pDpi)
            case 75
                eval(sprintf('print -dpng -r75 %s;', pOutput));
            case 150
                eval(sprintf('print -dpng -r150 %s;', pOutput));
            case 300
                eval(sprintf('print -dpng -r300 %s;', pOutput));
            case 600
                eval(sprintf('print -dpng -r600 %s;', pOutput));
            otherwise
                eval(sprintf('print -dpng -r600 %s;', pOutput));
        end;
    end;
end;

% Close video object
close(v);
end