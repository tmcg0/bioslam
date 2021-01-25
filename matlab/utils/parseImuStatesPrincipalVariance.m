% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function [ImuOrientationPrinVar,ImuPositionPrinVar,ImuVelPrinVar,ImuGyroBiasPrinVar,ImuAccelBiasPrinVar]=parseImuStatesPrincipalVariance(imuPoseKeys,imuVelKeys,imuBiasKeys,marginals)
% a convenient helper function to reduce code size
[ImuPosePrinVar,~]=computePrincipalVariance(imuPoseKeys,marginals,6);
ImuOrientationPrinVar=ImuPosePrinVar(:,1:3);
ImuPositionPrinVar=ImuPosePrinVar(:,4:6);
[ImuVelPrinVar,~]=computePrincipalVariance(imuVelKeys,marginals,3);
[ImuBiasPrinVar,~]=computePrincipalVariance(imuBiasKeys,marginals,6);
ImuGyroBiasPrinVar=ImuBiasPrinVar(:,4:6);
ImuAccelBiasPrinVar=ImuBiasPrinVar(:,1:3);
end