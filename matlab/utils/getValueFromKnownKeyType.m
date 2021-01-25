% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function val=getValueFromKnownKeyType(vals,key,valtype)
%GETVALUEFROMKNOWNKEYTYPE Summary of this function goes here
%   Detailed explanation goes here
if strcmp(valtype,'Pose3')
    val=vals.atPose3(key);
elseif strcmp(valtype,'Vector3')
    val=vals.atVector(key);
elseif strcmp(valtype,'ConstantBias')
    val=vals.atConstantBias(key);
elseif strcmp(valtype,'Unit3')
    warning('implement this');
else
    error('unknown key type!');
end
end