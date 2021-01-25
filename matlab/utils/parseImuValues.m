% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function [orientation,position,velocity,accelBias,gyroBias] = parseImuValues(vals)
%PARSEIMUVALUES parse gtsam::Values of a single IMU pose problem to constituent data
%   Detailed explanation goes here
% Assumes that vals is for exactly one IMU problem
pose3s=getAllPose3FromValues(vals);
orientation=Pose3ArrayToRot3Array(pose3s);
position=Pose3ArrayToPoint3Array(pose3s);
velocity=getAllVectorFromValues(vals,3);
estimatedImuBiases=getAllConstantBiasFromValues(vals);
accelBias=zeros(length(estimatedImuBiases),3);
gyroBias=zeros(length(estimatedImuBiases),3);
for k=1:length(estimatedImuBiases)
    accelBias(k,:)=estimatedImuBiases(k).accelerometer';
    gyroBias(k,:)=estimatedImuBiases(k).gyroscope';
end
end

