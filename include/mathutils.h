// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// utility functions for assorted mathematics and other operations

#pragma once

#include <gtsam/geometry/Pose3.h>

namespace mathutils {
    // --- type conversions --- //
    std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> Rot3VectorToQuaternionVector(std::vector<gtsam::Rot3> Rot3Vector);
    Eigen::MatrixXd Rot3VectorToQuaternionMatrix(const std::vector<gtsam::Rot3>& Rot3Vector);
    std::vector<gtsam::Rot3> QuaternionVectorToRot3Vector(std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> quatVector);
    std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> VectorVectorDoubleToVectorEigenVector(std::vector<std::vector<double>> vecIn);
    gtsam::Matrix33 quat2dcm(const gtsam::Vector4& q);
    gtsam::Vector4 dcm2quat(const gtsam::Matrix33& r);
    std::vector<double> EigenVectorToStdVector(const Eigen::VectorXd& vec);
    Eigen::VectorXd StdVectorToEigenVector(const std::vector<double>& vec);
    // --- angle calc methods --- //
    double signedAngleBetweenVectorsInSamePlane(const Eigen::Vector3d& vA, const Eigen::Vector3d& vB, const Eigen::Vector3d& vN);
    double unsignedAngle(const Eigen::Vector3d& a, const Eigen::Vector3d& b, boost::optional<gtsam::Matrix13 &> H_a=boost::none, boost::optional<gtsam::Matrix13 &> H_b=boost::none);
    double signedAngle(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& refVec, boost::optional<gtsam::Matrix13 &> H_a=boost::none, boost::optional<gtsam::Matrix13 &> H_b=boost::none);
    // --- data wrapping and unwrapping --- //
    double wrapToPi(const double& ang);
    Eigen::VectorXd angleUnwrapOnKnownDomain(const Eigen::VectorXd& angle, const double& domainMin=-1.0*M_PI, const double& domainMax=M_PI, const double& tol=2*M_PI*0.95);
    std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> quatUnwrap(std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> quatVector);
    std::vector<double> unwrappedYawAngle(const std::vector<gtsam::Rot3>& R);
    // --- statistics --- //
    double mean(const std::vector<double>& x);
    double stdev(const std::vector<double>& x);
    double max(const std::vector<double>& x);
    double min(const std::vector<double>& x);
    double median(const std::vector<double>& x);
    std::string distributionInfoString(const std::vector<double>& x); // string info about a distribution: mean, median, std, max, min
    std::string distributionInfoString(const Eigen::VectorXd& x); // string info about a distribution: mean, median, std, max, min
    // --- regression --- //
    double slope(const std::vector<double>& x, const std::vector<double>& y);
    void linearRegression(const Eigen::VectorXd& x, const Eigen::VectorXd& y, double& m, double& b);
    std::vector<double> adjustSlopeToTargetSlope(const std::vector<double>& origX, const std::vector<double>& origY, const double& newSlope);
    Eigen::VectorXd adjustRegressionSlopeToTargetSlope(const Eigen::VectorXd& x, const Eigen::VectorXd& y, const double& targetSlope);
    // --- filtering --- //
    std::tuple<Eigen::MatrixXd,Eigen::Matrix4d> simpleImuOrientationForwardEkf(const Eigen::MatrixXd& gyros, const Eigen::MatrixXd& accels, const double& dt, const Eigen::RowVector4d& initX, const Eigen::Matrix4d& initP);
    Eigen::MatrixXd simpleImuOrientationForwardBackwardEkfWrapper(const Eigen::MatrixXd& gyros, const Eigen::MatrixXd& accels, const double& dt, const uint& numFBPasses);
    // --- print methods --- //
    void printQuatVector(const std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>>& quatVector);
    void printQuatVectorToFile(const std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>>& quatVector, std::string filepath);
    // --- general math functions --- //
    gtsam::Point3 sub(const gtsam::Point3& v1, const gtsam::Point3& v2, boost::optional<gtsam::Matrix33 &> H_v1=boost::none, boost::optional<gtsam::Matrix33 &> H_v2=boost::none);
    gtsam::Vector3 scalarTimesVector(const double& a, const gtsam::Vector3& b, boost::optional<gtsam::Matrix31&> H_a=boost::none, boost::optional<gtsam::Matrix33&> H_b=boost::none);
    gtsam::Matrix33 skewSymmetric(gtsam::Vector3& v);
    double atan2(double y, double x, boost::optional<gtsam::Matrix11 &> Hy=boost::none, boost::optional<gtsam::Matrix11 &> Hx=boost::none);
    gtsam::Vector3 projmk(const gtsam::Vector3& m, const gtsam::Unit3& k, boost::optional<gtsam::Matrix33 &> H_m=boost::none, boost::optional<gtsam::Matrix32 &> H_k=boost::none); // projection of m onto k
    gtsam::Vector3 vectorFromPointToAxis(const gtsam::Vector3& p, const gtsam::Vector3& a, const gtsam::Unit3& k); // find vector from point p to axis k (perpendicular to axis), where a is any point on line.
    gtsam::Vector3 ptSeparation(const gtsam::Pose3& xA, const gtsam::Point3& sA, const gtsam::Pose3& xB, const gtsam::Point3& sB, boost::optional<gtsam::Matrix36 &> H_xA=boost::none, boost::optional<gtsam::Matrix33 &> H_sA=boost::none, boost::optional<gtsam::Matrix36 &> H_xB=boost::none, boost::optional<gtsam::Matrix33 &> H_sB=boost::none);
    std::vector<double> ptSeparationNorm(const std::vector<gtsam::Pose3>& xA, const gtsam::Point3& sA, const std::vector<gtsam::Pose3>& xB, const gtsam::Point3& sB);
    double ptSeparationNorm(const gtsam::Pose3& xA, const gtsam::Point3& sA, const gtsam::Pose3& xB, const gtsam::Point3& sB, boost::optional<gtsam::Matrix16 &> H_xA=boost::none, boost::optional<gtsam::Matrix13 &> H_sA=boost::none, boost::optional<gtsam::Matrix16 &> H_xB=boost::none, boost::optional<gtsam::Matrix13 &> H_sB=boost::none);
    gtsam::Vector3 projVecIntoPlane(const gtsam::Vector3& v, const gtsam::Vector3& n, boost::optional<gtsam::Matrix33 &> H_v=boost::none, boost::optional<gtsam::Matrix33 &> H_n=boost::none);
    // --- unsorted --- //
    void rotateImuPoseAboutPointAxisAngle(gtsam::Pose3& p, const gtsam::Point3& rotPointNav, const gtsam::Vector3& rotVecNav, const double& rotAngle);
    double errorFunScalarMax(double x, double xmax, double a, boost::optional<gtsam::Matrix11 &> H_x=boost::none);
    double errorFunScalarMin(double x, double xmin, double a, boost::optional<gtsam::Matrix11 &> H_x=boost::none);
    double errorFunScalarMaxMin(double x, double xmax, double xmin, double a, boost::optional<gtsam::Matrix11 &> H_x=boost::none);
    double meanAngBwAxesNavFrame(const std::vector<gtsam::Rot3>& rA, const gtsam::Unit3& kA, const std::vector<gtsam::Rot3>& rB, const gtsam::Unit3& kB);
    gtsam::Vector3 compute_wRel(gtsam::Vector4 qA, gtsam::Vector4 qB, gtsam::Vector3 wA, gtsam::Vector3 wB);
    Eigen::MatrixXd d_errorVec_d_qA_func(gtsam::Vector4 qA, gtsam::Vector4 qB, gtsam::Vector3 wA, gtsam::Vector3 wB);
    Eigen::MatrixXd d_errorVec_d_qB_func(gtsam::Vector4 qA, gtsam::Vector4 qB, gtsam::Vector3 wA, gtsam::Vector3 wB);
    bool are_quats_in_same_half_space(std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> qA, std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> qB);
    Eigen::Vector3d rollPitchYawRmseBetweenRot3Arrays(const std::vector<gtsam::Rot3>& rA, const std::vector<gtsam::Rot3>& rB);
    Eigen::VectorXd imuInternalPrecessionAboutStaticVector(const std::vector<gtsam::Rot3>& R_B_to_N, const gtsam::Point3& vproxB, const Eigen::Vector3d& seedStartVec=Eigen::Vector3d(0.0,0.0,1.0));
    uint findIdxOfNearestValInArray(const std::vector<double>& inputArray, double findVal); // get nearest index in vector<double>
}
