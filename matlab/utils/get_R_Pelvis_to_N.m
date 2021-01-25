% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function R_Pelvis_to_N = get_R_Pelvis_to_N(R_SacrumImu_to_N, upVec, sacrumImuToRHipCtr, sacrumImuToLHipCtr, useHipCtrAsMasterVec)
% convert gtsam types to native MATLAB types
if isa(R_SacrumImu_to_N,'gtsam.Pose3'); R_SacrumImu_to_N=Pose3ArrayToRot3Array(R_SacrumImu_to_N); end
if isa(R_SacrumImu_to_N,'gtsam.Rot3'); R_SacrumImu_to_N=Rot3ArrayToMatrices(R_SacrumImu_to_N); end
if isa(upVec,'gtsam.Unit3'); upVec=rightAxis.point3().vector(); end
if isa(sacrumImuToRHipCtr,'gtsam.Point3'); sacrumImuToRHipCtr=sacrumImuToRHipCtr.vector(); end
if isa(sacrumImuToLHipCtr,'gtsam.Point3'); sacrumImuToLHipCtr=sacrumImuToLHipCtr.vector(); end
assert(isa(R_SacrumImu_to_N,'double'));  assert(isa(upVec,'double')); assert(isa(sacrumImuToRHipCtr,'double')); assert(isa(sacrumImuToLHipCtr,'double'));
R_SacrumImu_to_Pelvis=get_R_SacrumImu_to_Pelvis(upVec, sacrumImuToRHipCtr,sacrumImuToLHipCtr,useHipCtrAsMasterVec);
R_Pelvis_to_N=zeros(3,3,size(R_SacrumImu_to_N,3));
for k=1:size(R_SacrumImu_to_N,3)
    R_Pelvis_to_N(:,:,k)=R_SacrumImu_to_N(:,:,k)*R_SacrumImu_to_Pelvis'; % R[Segment->N]=R[IMU->N]*R[Imu->Segment]'
end
end