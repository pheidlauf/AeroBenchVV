%
% MODEL_SHOW (model_show.m)
%
% Displays static and animated 3d models
% Last updated: 2016-03-02
%
% SYNTAX:
%
%    model_show(filename,varargin)
%
% PARAMETERS:
%
%    filename  : 3d model file name (*.mat)
%
% OPTIONAL ARGUMENTS:
%
%    'alpha'   : alpha channel value (0.0-1.0 - default 1.0)
%    'animate' : animation on/off ('on','off' - default 'off')
%    'color'   : display area color ([R G B] - default [1 1 1])
%    'dpi'     : dpi value (75,150,300,600 - default 150)
%    'edge'    : model edge color ([R G B] - default [.4 .4 .4])
%    'face'    : model face color ([R G B] - default [.5 .5 .5])
%    'output'  : output file name (string - default 'none') 
%    'view'    : camera view angles ([azimuth elevation] - default [15 30])
%    'window'  : display area size ([width height] - default [800 600])
%
% EXAMPLES OF USE:
%
%    model_show('model.mat');
%    model_show('model.mat','output','model.png','dpi',600);
%    model_show('model.mat','animation','on','output','model.gif');
%

function model_show(filename,varargin)

  % --- checking file exists ---
  if exist(filename,'file') ~= 2
     warning('file does not exist!');
     return;
  end;
  
  % --- default input parameters ---
  pAlpha = 1.0;				% alpha channel value: 0-1
  pAnimate = 'off';			% animation: 'on','off'
  pColor = [ 1 1 1 ];       % display area color: [ R G B ]
  pDpi = 150;               % dpi value: 75, 150, 300, 600
  pEdge = [ .4 .4 .4 ];		% model edge color: [ R G B ]
  pFace = [ .5 .5 .5 ];		% model face color: [ R G B ]
  pOutput = 'none';         % output filename or 'none'
  pView = [ 15 30 ];		% camera view angles: [ azimuth elevation ]
  pWindow = [ 800 600 ];	% display area size (in pixels)
  
  % --- read input parameters ---
  i = 1;
  while i <= length(varargin)
     switch lower(varargin{i})
        case 'alpha'
           pAlpha = varargin{i+1};
		case 'animate'
		   pAnimate = varargin{i+1};
        case 'color'
		   pColor = varargin{i+1};
        case 'dpi'
           pDpi = varargin{i+1};
		case 'edge'
		   pEdge = varargin{i+1};
		case 'face'
		   pFace = varargin{i+1};
		case 'output'
		   pOutput = varargin{i+1};
		case 'view'
		   pView = varargin{i+1};
		case 'window'
		   pWindow = varargin{i+1};
     end
     i = i + 2;
  end;
  
  % --- some extra checks ---
  [~,name,~] = fileparts(pOutput);
  if strcmp(pAnimate,'on') == 1
     if strcmp(pOutput,'none') ~= 1
        pOutput = strcat(name,'.gif');
     end;
  else
     if strcmp(pOutput,'none') ~= 1
        pOutput = strcat(name,'.png');
     end;
  end;
    
  % --- load data from file ---
  V = [];
  load(filename);
  val = 1.1 * max(max(V));
  
  % --- display model on the screen ---
  fig = figure(1);
  set(fig,...
	 'Name', 'flypath3d',...
	 'NumberTitle', 'off',...
	 'MenuBar', 'none',...
	 'ToolBar', 'figure',...
	 'Pointer', 'crosshair',...
	 'Position', [20 40 pWindow(1) pWindow(2)],...
	 'Color', pColor);
  clf(fig);
  
  if strcmp(pAnimate,'on') == 1
     index = 0;
     for i = 0.1:0.67:360
         clf(fig);
         hold on;
         trisurf( F,...
                  V(:,1), V(:,2), V(:,3),...
		          'FaceColor', pFace, 'FaceAlpha', pAlpha,...
		          'EdgeColor', pEdge );
         light('Position', [-val -val val], 'Style', 'infinite');
         lighting phong;
         view( [i pView(2)] );
         axis equal;
         axis off;
	     grid off;
	     box off;
         hold off;
         drawnow;
         if strcmp(pOutput,'none') ~= 1
            frame = getframe(1);
            im = frame2im(frame);
            [A,map] = rgb2ind(im,256);  
            if index == 0;
               imwrite(A,map,pOutput,'gif','LoopCount',Inf,'DelayTime',.04);
               index = index + 1;
            else
               imwrite(A,map,pOutput,'gif','WriteMode','append','DelayTime',.04);
            end;
         end;
     end;
  else
     clf(fig);
     hold on;
     trisurf( F,...
              V(:,1), V(:,2), V(:,3),...
		      'FaceColor', pFace, 'FaceAlpha', pAlpha,...
		      'EdgeColor', pEdge ); 
     light('Position', [-val -val val], 'Style', 'local');
     lighting phong;
     view( pView );
     axis equal;
     axis off;
	 grid off;
	 box off;
     hold off;
     drawnow;
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