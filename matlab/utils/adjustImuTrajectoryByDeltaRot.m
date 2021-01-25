% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function [imuPoseTrajectory,imuVelTrajectory] = adjustImuTrajectoryByDeltaRot(imuPoseTrajectory,imuVelTrajectory,deltaRot)
% applies input deltaRot to entire imu trajectory
% logic based on c++ test yawImuStatesByConstantValueInPlace (below)
% deltaRot is assumed to be R[B->B2], where B2 is the new coordinate system
% deltaRot can be of length 1 or the same length as the trajectories
% () arg checking
assert(isa(imuPoseTrajectory,'gtsam.Pose3')); assert(isa(deltaRot,'gtsam.Rot3'));
assert(size(imuVelTrajectory,1)==length(imuPoseTrajectory)); assert(size(imuVelTrajectory,2)==3);
if length(deltaRot)==1 % repmat it to be the same size
   deltaRot=repmat(deltaRot,[length(imuPoseTrajectory) 1]); 
end
% go
for k=1:length(imuPoseTrajectory)
    % originally, you have R[B->N]. you want to construct R[B2->N], where B2 is a coordiante system very similar to B, but slightly rotated about the global vertical axis (assuming +Z)
    % update all orientations by the heading rotation offset
    origRot3=imuPoseTrajectory(k).rotation(); % this is R[B->N]
    newRot3=gtsam.Rot3((deltaRot(k).matrix()*origRot3.inverse().matrix())'); %R[B2->N]=(R[B->B2]*R[B->N]')'
    % update positions
    newPoint3=gtsam.Point3(deltaRot(k).matrix()*(imuPoseTrajectory(k).translation().vector()));
    imuPoseTrajectory(k)=gtsam.Pose3(newRot3,newPoint3); % update pose
    % update velocities
    imuVelTrajectory(k,1:3)=(deltaRot(k).matrix()*(imuVelTrajectory(k,1:3)'))'; % update vel
end
end

%{
void imuPoseEstimator::yawImuStatesByConstantValueInPlace(gtsam::Values& vals, const double& headingAngleDeltaRad, const gtsam::KeyVector& poseKeys, const gtsam::KeyVector& velKeys){
    // in place edits a set of Values (vals) to yaw just the imu states which are described by keys poseKeys and velKeys.
    gtsam::Rot3 pureYawR=gtsam::Rot3::Ypr(headingAngleDeltaRad,0.0,0.0); // note: this is equivalent to gtsam::Rot3::Rz(headingAngleDeltaRad)
    for (uint k=0; k<poseKeys.size(); k++){
        // originally, you have R[B->N]. you want to construct R[B2->N], where B2 is a coordiante system very similar to B, but slightly rotated about the global vertical axis (assuming +Z)
        // update all orientations by the heading rotation offset
        gtsam::Pose3 origPose3=vals.at<gtsam::Pose3>(poseKeys[k]);
        gtsam::Rot3 origRot3=origPose3.rotation(); // this is R[B->N]
        gtsam::Rot3 newRot3=gtsam::Rot3::Ypr(origRot3.yaw()+headingAngleDeltaRad,origRot3.pitch(),origRot3.roll()); // this is R[B2->N]: reconstruct the same Rot3 but with a small change to yaw
        // update positions
        gtsam::Point3 origPoint3=origPose3.translation();
        gtsam::Point3 newPoint3=pureYawR*(origPoint3);
        gtsam::Pose3 newPose3=gtsam::Pose3(newRot3,newPoint3);
        vals.update(poseKeys[k], newPose3 ); // <-- update
        // update velocities
        gtsam::Vector3 origVel=vals.at<gtsam::Vector3>(velKeys[k]);
        gtsam::Vector3 newVel=pureYawR*(origVel);
        vals.update(velKeys[k], newVel); // <-- update
    }
}
%}