% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

% unit test to test implementations of adjusting distal imu orientation based on desired joint angles
clc; clear; close all;
nTests=10000;
for a=1:nTests
    % () construct random proximal imu orientation and states
    R_ProxImu_to_N=quat2dcm(quatnormalize(rand(1,4)));
    rightAxisProx=rand(3,1); rightAxisProx=rightAxisProx/sqrt(sum(rightAxisProx.^2,1));
    refVec=rand(3,1); refVec=refVec/sqrt(sum(refVec.^2,1));
    proxImuToProxJointCtr=cross(refVec,rightAxisProx); proxImuToProxJointCtr=proxImuToProxJointCtr/sqrt(sum(proxImuToProxJointCtr.^2,1));
    proxImuToDistalJointCtr=-proxImuToProxJointCtr;
    R_ProxSeg_to_N=get_R_Segment_to_N(R_ProxImu_to_N,rightAxisProx,proxImuToProxJointCtr,proxImuToDistalJointCtr,true);
    % () construct random distal imu orientation and states
    R_DistImu_to_N=quat2dcm(quatnormalize(rand(1,4)));
    rightAxisDist=rand(3,1); rightAxisDist=rightAxisDist/sqrt(sum(rightAxisDist.^2,1));
    refVec=rand(3,1); refVec=refVec/sqrt(sum(refVec.^2,1));
    distImuToProxJointCtr=cross(refVec,rightAxisDist); distImuToProxJointCtr=distImuToProxJointCtr/sqrt(sum(distImuToProxJointCtr.^2,1));
    distImuToDistalJointCtr=-distImuToProxJointCtr;
    R_DistSeg_to_N=get_R_Segment_to_N(R_DistImu_to_N,rightAxisDist,distImuToProxJointCtr,distImuToDistalJointCtr,true);
    % test
    a=-pi/2; b=pi/2; 
    desiredFlexExAng = (b-a).*rand(1,1) + a;
    desiredIntExtRotAng = (b-a).*rand(1,1) + a;
    desiredAbAdAng = (b-a).*rand(1,1) + a;
    % old angles
    [flexExAngRadOrig,abAdAngRadOrig,intExtRotAngleRadOrig]=consistent3DofJcsAngles(R_ProxSeg_to_N, R_DistSeg_to_N);
    % adjust
    [R_DistalImu_to_N_1,R_B_to_B2]=adjustDistalImuOrientationGivenDesiredIntExtRotAngle(R_ProxSeg_to_N,R_DistImu_to_N,desiredIntExtRotAng, rightAxisDist, distImuToProxJointCtr,distImuToDistalJointCtr);
    [R_DistalImu_to_N_2,R_B_to_B2]=adjustDistalImuOrientationGivenDesiredFlexExAngle(R_ProxSeg_to_N,R_ProxImu_to_N,R_DistalImu_to_N_1,desiredFlexExAng, rightAxisProx,rightAxisDist, distImuToProxJointCtr,distImuToDistalJointCtr);
    [R_DistalImu_to_N_3,R_B_to_B2]=adjustDistalImuOrientationGivenDesiredAbAdAngle(R_ProxSeg_to_N,R_DistalImu_to_N_2,desiredAbAdAng, rightAxisDist, distImuToProxJointCtr,distImuToDistalJointCtr);
    % new angles
    R_newDistalSeg_to_N=get_R_Segment_to_N(R_DistalImu_to_N_3,rightAxisDist,distImuToProxJointCtr,distImuToDistalJointCtr,true);
    [flexExAngRadNew,abAdAngRadNew,intExtRotAngleRadNew]=consistent3DofJcsAngles(R_ProxSeg_to_N, R_newDistalSeg_to_N);
    % assert new angles are same as desired angles
    assert(abs(flexExAngRadNew-desiredFlexExAng)<1.0e-10);
    assert(abs(intExtRotAngleRadNew-desiredIntExtRotAng)<1.0e-10);
    assert(abs(abAdAngRadNew-abAdAngRadNew)<1.0e-10);
end