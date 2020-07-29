function [myAddedPaths] = listAddedPaths(reqStr)
%LISTADDEDPATHS This returns a string array of all non-default added paths
% If reqStr is supplied, only paths containing that pattern are included
if nargin ~= 1
    reqStr = '';
end

% Select all added paths
allAddedPaths = strsplit(path, ';');
% Select default paths
isDefault = contains(allAddedPaths, 'Program Files');
% Select required paths matching reqStr
hasReqStr = contains(allAddedPaths, reqStr);

% Filter and convert to string array
myAddedPaths = string(allAddedPaths(and(hasReqStr,~isDefault)))';

end
