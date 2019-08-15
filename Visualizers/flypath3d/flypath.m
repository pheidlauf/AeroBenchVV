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
        xlabel('x [m]');
        ylabel('y [m]');
        zlabel('z [m]');
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
        % --- save the results to the gif file ---
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
     
  
end