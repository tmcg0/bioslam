// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// utility functions for biomechanics -- including joint angle calculations

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/geometry/Point3.h>

namespace bioutils {
    // coordinate system functions
    gtsam::Matrix get_R_Imu_to_Segment(const gtsam::Unit3 &rightAxis, const gtsam::Point3 &imuToProximalJointVec, const gtsam::Point3 &imuToDistalJointVec, bool useProximalAsMasterVec=true); // for thighs, shanks, feet
    gtsam::Matrix get_R_SacrumImu_to_Pelvis(const gtsam::Vector3& upVec, const gtsam::Point3 &sacrumImuToRHipCtr, const gtsam::Point3 &sacrumImuToLHipCtr, bool useHipCtrAsMasterVec=true);
    std::vector<gtsam::Rot3> get_R_Segment_to_N(const std::vector<gtsam::Rot3>& R_Imu_to_N,const gtsam::Unit3 &rightAxis, const gtsam::Point3 &imuToProximalJointVec, const gtsam::Point3 &imuToDistalJointVec, bool useProximalAsMasterVec=true);
    gtsam::Rot3 get_R_Segment_to_N(const gtsam::Rot3& R_Imu_to_N,const gtsam::Unit3 &rightAxis, const gtsam::Point3 &imuToProximalJointVec, const gtsam::Point3 &imuToDistalJointVec, bool useProximalAsMasterVec=true);
    std::vector<gtsam::Rot3> get_R_Pelvis_to_N(const std::vector<gtsam::Rot3>& R_SacrumImu_to_N,const gtsam::Vector3& upVec, const gtsam::Point3 &sacrumImuToRHipCtr, const gtsam::Point3 &sacrumImuToLHipCtr, bool useHipCtrAsMasterVec=true);
    gtsam::Rot3 get_R_Pelvis_to_N(const gtsam::Rot3& R_SacrumImu_to_N,const gtsam::Vector3& upVec, const gtsam::Point3 &sacrumImuToRHipCtr, const gtsam::Point3 &sacrumImuToLHipCtr, bool useHipCtrAsMasterVec=true);
    // angle calculation functions
    Eigen::RowVector3d consistent3DofJcsAngles(const gtsam::Rot3& R_proximalSeg_to_N, const gtsam::Rot3& R_distalSeg_to_N); // "consistent" angles maintain same sign convention for a rightward or leftward rotation across both legs, e.g., + int/ext rot is external rotation on right leg and internal rotation on left leg
    Eigen::MatrixXd consistent3DofJcsAngles(const std::vector<gtsam::Rot3>& R_proximalSeg_to_N, const std::vector<gtsam::Rot3>& R_distalSeg_to_N, const bool& angleUnwrap=true); // "consistent" angles maintain same sign convention for a rightward or leftward rotation across both legs, e.g., + int/ext rot is external rotation on right leg and internal rotation on left leg
    Eigen::RowVector3d clinical3DofJcsAngles(const gtsam::Rot3& R_proximalSeg_to_N, const gtsam::Rot3& R_distalSeg_to_N, const bool& isRightLeg, const bool& isHip); // "clinical" angles use the mirrored sign convention that clinicians use, e.g., internal rotation is always inward toward the body centerline
    Eigen::MatrixXd clinical3DofJcsAngles(const std::vector<gtsam::Rot3>& R_proximalSeg_to_N, const std::vector<gtsam::Rot3>& R_distalSeg_to_N, const bool& isRightLeg, const bool& isHip, const bool& angleUnwrap=false); // "clinical" angles use the mirrored sign convention that clinicians use, e.g., internal rotation is always inward toward the body centerline
}
