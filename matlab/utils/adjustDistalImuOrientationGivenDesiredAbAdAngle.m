% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function [R_newDistalImu_to_N,R_B_to_B2]=adjustDistalImuOrientationGivenDesiredAbAdAngle(R_ProxSeg_to_N,R_DistalImu_to_N,desiredAbAdAng, rightAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr)
% given the proximal segment orientation, reorient the distal IMU orientation to produce the desired consistent JCS int/ext rot angle (consistent JCS)
% INPUTS:
%    (insert info here) assumes all orientations are [B->N]
% OUTPUTS:
%    R_newDistalImu_to_N: R[B2->N], the new distal imu orientation which produces the desired angle
%    R_B_to_B2: R[B->B2], the incremental rotation applied to B in order to produce the new orientation, i.e., satisfies R[B2->N]=(R[B->B2]*R[B->N]')'
% --- settings ---
checkAngleAtEnd=1; % double check the angle at the end is the desired angle?
% ----------------
% () convert gtsam types to native MATLAB types and check types
if isa(R_DistalImu_to_N,'gtsam.Pose3'); R_DistalImu_to_N=Pose3ArrayToRot3Array(R_DistalImu_to_N); end
if isa(R_DistalImu_to_N,'gtsam.Rot3'); R_DistalImu_to_N=Rot3ArrayToMatrices(R_DistalImu_to_N); end
if isa(rightAxisDistal,'gtsam.Unit3'); rightAxisDistal=rightAxisDistal.point3().vector(); end
if isa(distalImuToProxJointCtr,'gtsam.Point3'); distalImuToProxJointCtr=distalImuToProxJointCtr.vector(); end
if isa(distalImuToDistalJointCtr,'gtsam.Point3'); distalImuToDistalJointCtr=distalImuToDistalJointCtr.vector(); end
assert(isa(R_ProxSeg_to_N,'double')); assert(isa(rightAxisDistal,'double')); assert(isa(distalImuToProxJointCtr,'double')); assert(isa(distalImuToDistalJointCtr,'double')); assert(isa(desiredAbAdAng,'double'));
assert(size(R_DistalImu_to_N,3)==1,'does not support arrays yet');
% () find original R_DistalSeg_to_N
R_DistalSeg_to_N=get_R_Segment_to_N(R_DistalImu_to_N,rightAxisDistal,distalImuToProxJointCtr, distalImuToDistalJointCtr,true);
% () find original angle
[~,abAdAngOrig,~]=consistent3DofJcsAngles(R_ProxSeg_to_N, R_DistalSeg_to_N);
% () determine delta rotation necessary to produce desired angle
% the actual rotation axis of ab/ad according to Grood is e2=cross(e3,e1); so we need to construct e2 in the distal frame and rotate about that.
INav=R_ProxSeg_to_N*[1.0,0.0,0.0]'; % I in nav frame
kNav=R_DistalSeg_to_N*[0.0,0.0,1.0]'; % k in nav frame
e1Nav=INav/sqrt(sum(INav.^2,1)); % e1 = proximal segment x, assumed to the subject's right
e3Nav=kNav/sqrt(sum(kNav.^2,1)); % e3 = distal segment z, assumed to be proximal
e2Nav=cross(e3Nav,e1Nav); e2Nav=e2Nav/sqrt(sum(e2Nav.^2,1)); % e2 in nav frame. e2 floats, defined from cross product. if subject were at zero angles, e2 would point anterior
e2DistImuFrame=R_DistalImu_to_N'*e2Nav;
% () determine rotation angle
%  desiredAng = origAng + deltaAng => deltaAng=desiredAng-origAng
spinAngle=desiredAbAdAng-abAdAngOrig;
% () find new distal imu orientation, R[B2->N]
R_B_to_B2=axang2rotm([e2DistImuFrame(:)',spinAngle]); % R[B->B2]. inconsistency: on other DOF, this is -spinAngle. Can't figure out why it's positive for ab/ad.
R_newDistalImu_to_N=R_DistalImu_to_N*R_B_to_B2'; % R[B2->N]=R[B->N]*R[B->B2]'
% () optional: double check that when applied, this R_B_to_B2 produces desired angle
if checkAngleAtEnd
    % () get new distal segment orientation
    R_newDistalSeg_to_N=get_R_Segment_to_N(R_newDistalImu_to_N,rightAxisDistal,distalImuToProxJointCtr, distalImuToDistalJointCtr,true);
    % () get new angle
    [~,abAdAngNew,~]=consistent3DofJcsAngles(R_ProxSeg_to_N, R_newDistalSeg_to_N);
    % () assert it's what you asked for
    assert(abs(desiredAbAdAng-abAdAngNew)<1.0e-4);
end
end