% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function varCols=varTypeToVarSize(varTypes)
varCols=zeros(1,length(varTypes));
for k=1:length(varTypes)
    if contains(varTypes{k},'Pose3')
        varCols(k)=6; % Pose3
    elseif contains(varTypes{k},'Vector3')
        varCols(k)=3; % velocity vector3
    elseif contains(varTypes{k},'ConstantBias')
        varCols(k)=6; % imu bias vector6
    else
        error('do not recognize this variable type');
    end
end
end