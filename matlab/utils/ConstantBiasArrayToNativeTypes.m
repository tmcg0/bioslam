% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function [gyroBias,accelBias]=ConstantBiasArrayToNativeTypes(constantBiasArray)
accelBias=zeros(length(constantBiasArray),3); gyroBias=zeros(length(constantBiasArray),3); 
for k=1:length(constantBiasArray)
    gb=constantBiasArray(k).gyroscope;
    ab=constantBiasArray(k).accelerometer;
    accelBias(k,:)=ab(:)';
    gyroBias(k,:)=gb(:)';
end
end