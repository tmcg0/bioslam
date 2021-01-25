% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function varType=determineVarType(vals,key)
% god this is gonna be ugly.
varTypeFound=0;
varType='Unknown';
% Pose3
if ~varTypeFound
    try
        v=vals.atPose3(key);
        varTypeFound=1;
        varType='Pose3';
    catch
    end
end
% Vector
if ~varTypeFound
    try
        v=vals.atVector(key);
        varTypeFound=1;
        varType=strcat('Vector',num2str(length(v)));
    catch
    end
end
% ConstantBias
if ~varTypeFound
    try
        v=vals.atConstantBias(key);
        varTypeFound=1;
        varType='ConstantBias';
    catch
    end
end
end