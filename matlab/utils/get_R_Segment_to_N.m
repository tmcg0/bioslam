% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function R_Segment_to_N = get_R_Segment_to_N(R_Imu_to_N, rightAxis, imuToProximalJointVec, imuToDistalJointVec,useProximalAsMasterVec)
% convert gtsam types to native MATLAB types
if isa(R_Imu_to_N,'gtsam.Pose3'); R_Imu_to_N=Pose3ArrayToRot3Array(R_Imu_to_N); end
if isa(R_Imu_to_N,'gtsam.Rot3'); R_Imu_to_N=Rot3ArrayToMatrices(R_Imu_to_N); end
if isa(rightAxis,'gtsam.Unit3'); rightAxis=rightAxis.point3().vector(); end
if isa(imuToProximalJointVec,'gtsam.Point3'); imuToProximalJointVec=imuToProximalJointVec.vector(); end
if isa(imuToDistalJointVec,'gtsam.Point3'); imuToDistalJointVec=imuToDistalJointVec.vector(); end
assert(isa(R_Imu_to_N,'double')); assert(isa(rightAxis,'double')); assert(isa(imuToProximalJointVec,'double')); assert(isa(imuToDistalJointVec,'double'));
R_Imu_to_Segment=get_R_Imu_to_Segment(rightAxis,imuToProximalJointVec, imuToDistalJointVec, useProximalAsMasterVec);
R_Segment_to_N=zeros(3,3,size(R_Imu_to_N,3));
for k=1:size(R_Imu_to_N,3)
    R_Segment_to_N(:,:,k)=R_Imu_to_N(:,:,k)*R_Imu_to_Segment'; % R[Segment->N]=R[IMU->N]*R[Imu->Segment]'
end
end
%{
    std::vector<gtsam::Rot3> get_R_Segment_to_N(const std::vector<gtsam::Rot3>& R_Imu_to_N,const gtsam::Unit3 &rightAxis, const gtsam::Point3 &imuToProximalJointVec, const gtsam::Point3 &imuToDistalJointVec, bool useProximalAsMasterVec){
        // () get constant imu to segment relationship
        const gtsam::Rot3 R_Imu_to_Segment=gtsam::Rot3(get_R_Imu_to_Segment(rightAxis,imuToProximalJointVec, imuToDistalJointVec, useProximalAsMasterVec));
        // () now loop over orientations and create R[Segment->N]
        std::vector<gtsam::Rot3> R_Segment_to_N(R_Imu_to_N.size());
        for(uint k=0; k<R_Imu_to_N.size();k++){
            R_Segment_to_N[k]=R_Imu_to_N[k]*R_Imu_to_Segment.inverse(); // R[Segment->N]=R[IMU->N]*R[Imu->Segment]'
        }
        return R_Segment_to_N;
    }
%}
