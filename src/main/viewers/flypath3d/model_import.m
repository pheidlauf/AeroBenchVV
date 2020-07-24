%
% MODEL_IMPORT (model_import.m)
%
% Converts *.obj mesh file to the vertex and face list *.mat file
% Last updated: 2016-03-02
%
% SYNTAX:
%
%    model_import(filename,varargin)
%
% PARAMETERS:
%
%    filename  : 3d model file name (*.obj)
%
% OPTIONAL ARGUMENTS:
%
%    'output'  : output file name (string - default filename) 
%
% EXAMPLES OF USE:
%
%    model_import('model.obj');
%    model_import('model.obj','output','new_model.mat');
%

function model_import(filename,varargin)
  
  % --- checking file exists ---
  if exist(filename,'file') ~= 2
     warning('file does not exist!');
     return;
  end;
  
  % --- default input parameters ---
  [~,name,~] = fileparts(filename);
  pOutput = strcat(name,'.mat');
  
  % --- read input parameters ---
  i = 1;
  while i <= length(varargin),
     switch lower(varargin{i})
		case 'output'
		   pOutput = varargin{i+1};
     end
     i = i + 2;
  end;
  
  % --- open obj file ---  
  V = zeros(0,3);
  F = zeros(0,3);
  vertex_index = 1;
  face_index = 1;
  fid = fopen(filename,'rt');
  line = fgets(fid);
  
  % --- convert it to the vertex and face list ---
  while ischar(line)
     vertex = sscanf(line,'v %f %f %f');
     face3 = sscanf(line,'f %d %d %d');
     face6 = sscanf(line,'f %d//%d %d//%d %d//%d',6);
     face9 = sscanf(line,'f %d/%d/%d %d/%d/%d %d/%d/%d',9);
     % --- check if line is vertex command (if so add to vertices) ---
     if size(vertex) > 0
        V(vertex_index,:) = vertex;
        vertex_index = vertex_index + 1;
     % --- check if line is simple face command (if so add to faces) ---
     elseif size(face3,1) == 3 
        F(face_index,:) = face3;
        face_index = face_index + 1;
     % --- check if line is a face with normal indices command (if so add to faces) ---
     elseif size(face6,1) == 6
        face6 = face6(1:2:end);
        F(face_index,:) = face6;
        face_index = face_index + 1;
     % --- see if line is a face with normal and texture indices command (if so add to faces) ---
     elseif size(face9,1) == 9
        face9 = face9(1:3:end);
        F(face_index,:) = face9;
        face_index = face_index + 1;
     else
        warning('unsupported line: %s',line);
     end
     line = fgets(fid);
  end
  fclose(fid);
  
  % --- save model to the *.mat file format ---
  save(pOutput,'V','F');
end