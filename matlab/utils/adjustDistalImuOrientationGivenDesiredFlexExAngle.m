% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function [R_newDistalImu_to_N,R_B_to_B2]=adjustDistalImuOrientationGivenDesiredFlexExAngle(R_ProxSeg_to_N,R_ProxImu_to_N,R_DistalImu_to_N,desiredFlexExAng, rightAxisProximal,rightAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr)
% given the proximal segment orientation, reorient the distal IMU orientation to produce the desired flex ex angle (consistent JCS)
% INPUTS:
%    (insert info here) assumes all orientations are [B->N]
% OUTPUTS:
%    R_newDistalImu_to_N: R[B2->N], the new distal imu orientation which produces the desired angle
%    R_B_to_B2: R[B->B2], the incremental rotation applied to B in order to produce the new orientation, i.e., satisfies R[B2->N]=(R[B->B2]*R[B->N]')'
% --- settings ---
checkAngleAtEnd=1; % double check the angle at the end is the desired angle?
% ----------------
% () convert gtsam types to native MATLAB types and check types
if isa(R_ProxImu_to_N,'gtsam.Pose3'); R_ProxImu_to_N=Pose3ArrayToRot3Array(R_ProxImu_to_N); end
if isa(R_ProxImu_to_N,'gtsam.Rot3'); R_ProxImu_to_N=Rot3ArrayToMatrices(R_ProxImu_to_N); end
if isa(R_DistalImu_to_N,'gtsam.Pose3'); R_DistalImu_to_N=Pose3ArrayToRot3Array(R_DistalImu_to_N); end
if isa(R_DistalImu_to_N,'gtsam.Rot3'); R_DistalImu_to_N=Rot3ArrayToMatrices(R_DistalImu_to_N); end
if isa(rightAxisProximal,'gtsam.Unit3'); rightAxisProximal=rightAxisProximal.point3().vector(); end
if isa(rightAxisDistal,'gtsam.Unit3'); rightAxisDistal=rightAxisDistal.point3().vector(); end
if isa(distalImuToProxJointCtr,'gtsam.Point3'); distalImuToProxJointCtr=distalImuToProxJointCtr.vector(); end
if isa(distalImuToDistalJointCtr,'gtsam.Point3'); distalImuToDistalJointCtr=distalImuToDistalJointCtr.vector(); end
assert(isa(R_ProxImu_to_N,'double')); assert(isa(R_DistalImu_to_N,'double')); assert(isa(rightAxisDistal,'double')); assert(isa(distalImuToProxJointCtr,'double')); assert(isa(distalImuToDistalJointCtr,'double')); assert(isa(desiredFlexExAng,'double'));
assert(size(R_DistalImu_to_N,3)==1,'does not support arrays yet');
% () find original R_Seg_to_N
R_DistalSeg_to_N=get_R_Segment_to_N(R_DistalImu_to_N,rightAxisDistal,distalImuToProxJointCtr, distalImuToDistalJointCtr,true);
% () find original angle
[flexExAngleRadOrig,~,~]=consistent3DofJcsAngles(R_ProxSeg_to_N, R_DistalSeg_to_N);
% () determine delta rotation necessary to produce desired angle
%  desiredAng = origAng + deltaAng => deltaAng=desiredAng-origAng
spinAngle=desiredFlexExAng-flexExAngleRadOrig;
kneeAxisProxInDistalFrame=R_DistalImu_to_N'*R_ProxImu_to_N*rightAxisProximal; % vA[B]= R_B_to_N'*R_A_to_N*vA[A]
R_B_to_B2=axang2rotm([kneeAxisProxInDistalFrame(:)',-spinAngle]); % R[B->B2]. neg. spin angle because it's applied to distal imu frame.
% () find new distal imu orientation, R[B2->N]
R_newDistalImu_to_N=R_DistalImu_to_N*R_B_to_B2'; % R[B2->N]=R[B->N]*R[B->B2]'
% () optional: double check that when applied, this R_B_to_B2 produces desired angle
if checkAngleAtEnd
    % () get new distal segment orientation
    R_newDistalSeg_to_N=get_R_Segment_to_N(R_newDistalImu_to_N,rightAxisDistal,distalImuToProxJointCtr, distalImuToDistalJointCtr,true);
    % () get new angle
    [flexExAngleRadNew,~,~]=consistent3DofJcsAngles(R_ProxSeg_to_N, R_newDistalSeg_to_N);
    % () assert it's what you asked for
    assert(abs(desiredFlexExAng-flexExAngleRadNew)<5.0e-3);
end
end