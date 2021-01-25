// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// this is the header file to declare the required bioslam factors in MATLAB. This is used with CMake to generate the mex files for these factors.
// syntax rules to overcome parser failing:
// - classes must start with capital letter
// - must have empty line at end of this file

virtual class gtsam::NonlinearFactor;
virtual class gtsam::NoiseModelFactor : gtsam::NonlinearFactor;
virtual class gtsam::Pose3;
virtual class gtsam::Point3;
virtual class gtsam::Unit3;
virtual class gtsam::imuBias::ConstantBias;

namespace bioslam {
#include <factors/HingeJointFactors.h>
virtual class HingeJointConstraintVecErrEstAngVel : gtsam::NoiseModelFactor {
    HingeJointConstraintVecErrEstAngVel(size_t xA, size_t omegaA, size_t xB, size_t omegaB, size_t axisA, gtsam::noiseModel::Diagonal* model);
    Vector evaluateError(const gtsam::Pose3 &xA, const Vector& omegaA, const gtsam::Pose3 &xB, const Vector& omegaB, const gtsam::Unit3& axisA);
};
virtual class HingeJointConstraintNormErrEstAngVel : gtsam::NoiseModelFactor {
    HingeJointConstraintNormErrEstAngVel(size_t xA, size_t omegaA, size_t xB, size_t omegaB, size_t axisA, gtsam::noiseModel::Diagonal* model);
    Vector evaluateError(const gtsam::Pose3 &xA, const Vector& omegaA, const gtsam::Pose3 &xB, const Vector& omegaB, const gtsam::Unit3& axisA);
};
#include <factors/ConstrainedJointCenterPositionFactor.h>
    virtual class ConstrainedJointCenterPositionFactor : gtsam::NoiseModelFactor {
        ConstrainedJointCenterPositionFactor(size_t x_A_to_N, size_t x_B_to_N, size_t L_A_to_ctr, size_t L_B_to_ctr, gtsam::noiseModel::Diagonal* model);
        Vector evaluateError(const gtsam::Pose3& xA, const gtsam::Pose3& xB, const gtsam::Point3& L_A_to_ctr, const gtsam::Point3& L_B_to_ctr);
    };
    virtual class ConstrainedJointCenterNormPositionFactor : gtsam::NoiseModelFactor {
        ConstrainedJointCenterNormPositionFactor(size_t x_A_to_N, size_t x_B_to_N, size_t L_A_to_ctr, size_t L_B_to_ctr, gtsam::noiseModel::Diagonal* model);
        Vector evaluateError(const gtsam::Pose3& xA, const gtsam::Pose3& xB, const gtsam::Point3& L_A_to_ctr, const gtsam::Point3& L_B_to_ctr);
    };
#include <factors/ConstrainedJointCenterVelocityFactor.h>
virtual class ConstrainedJointCenterVelocityFactor : gtsam::NoiseModelFactor {
    ConstrainedJointCenterVelocityFactor(size_t poseA, size_t linVelA, size_t angVelA, size_t sA, size_t poseB, size_t linVelB, size_t angVelB, size_t sB, gtsam::noiseModel::Diagonal* model);
    Vector evaluateError(const gtsam::Pose3& poseA, const Vector& linVelA, const Vector& angVelA, const gtsam::Point3& sA, const gtsam::Pose3& poseB, const Vector& linVelB, const Vector& angVelB, const gtsam::Point3& sB);
};
virtual class ConstrainedJointCenterNormVelocityFactor : gtsam::NoiseModelFactor {
    ConstrainedJointCenterNormVelocityFactor(size_t poseA, size_t linVelA, size_t angVelA, size_t sA, size_t poseB, size_t linVelB, size_t angVelB, size_t sB, gtsam::noiseModel::Diagonal* model);
    Vector evaluateError(const gtsam::Pose3& poseA, const Vector& linVelA, const Vector& angVelA, const gtsam::Point3& sA, const gtsam::Pose3& poseB, const Vector& linVelB, const Vector& angVelB, const gtsam::Point3& sB);
};
#include <factors/MagPose3Factor.h>
    virtual class MagPose3Factor : gtsam::NoiseModelFactor {
        MagPose3Factor(size_t pose3key, const gtsam::Point3& measured, double scale, const gtsam::Unit3& direction, const gtsam::Point3& bias, gtsam::noiseModel::Diagonal* model);
        Vector evaluateError(const gtsam::Pose3& pose_i);
    };
#include <factors/Point3Priors.h>
    virtual class MaxPoint3MagnitudeFactor : gtsam::NoiseModelFactor {
        MaxPoint3MagnitudeFactor(size_t point3key, const double maxNorm, gtsam::noiseModel::Diagonal* model);
        Vector evaluateError(const gtsam::Point3& p);
    };
    virtual class Point3MagnitudeDifferenceFactor : gtsam::NoiseModelFactor {
        Point3MagnitudeDifferenceFactor(size_t v1, size_t v2, gtsam::noiseModel::Diagonal* model);
        Vector evaluateError(const gtsam::Point3& v1, const gtsam::Point3& v2);
    };
#include <factors/AngularVelocityFactor.h>
virtual class AngularVelocityFactor : gtsam::NoiseModelFactor {
    AngularVelocityFactor(size_t estAngVelKey, size_t imuBiasKey, const Vector& measuredAngVelAtJ, gtsam::noiseModel::Diagonal* model);
    Vector evaluateError(const Vector& estAngVel, const gtsam::imuBias::ConstantBias& imuBias);
};
#include <factors/SegmentLengthMagnitudeFactor.h>
    virtual class SegmentLengthMagnitudeFactor : gtsam::NoiseModelFactor {
        SegmentLengthMagnitudeFactor(size_t v1key, size_t v2key, const double idealSegmentLength, gtsam::noiseModel::Diagonal* model);
        Vector evaluateError(const gtsam::Point3& v1, const gtsam::Point3& v2);
    };
virtual class SegmentLengthMaxMagnitudeFactor : gtsam::NoiseModelFactor {
    SegmentLengthMaxMagnitudeFactor(size_t v1key, size_t v2key, const double maxSegmentLength, gtsam::noiseModel::Diagonal* model);
    Vector evaluateError(const gtsam::Point3& v1, const gtsam::Point3& v2);
};
virtual class SegmentLengthMinMagnitudeFactor : gtsam::NoiseModelFactor {
    SegmentLengthMinMagnitudeFactor(size_t v1key, size_t v2key, const double minSegmentLength, gtsam::noiseModel::Diagonal* model);
    Vector evaluateError(const gtsam::Point3& v1, const gtsam::Point3& v2);
};
#include <factors/SegmentLengthDiscrepancyFactor.h>
    virtual class SegmentLengthDiscrepancyFactor : gtsam::NoiseModelFactor {
        SegmentLengthDiscrepancyFactor(size_t imuAToProximalJointCtr, size_t imuAToDistalJointCtr, size_t imuBToProximalJointCtr, size_t imuBToDistalJointCtr, gtsam::noiseModel::Diagonal* model);
        Vector evaluateError(const gtsam::Point3& imuAToProximalJointCtr, const gtsam::Point3& imuAToDistalJointCtr, const gtsam::Point3& imuBToProximalJointCtr, const gtsam::Point3& imuBToDistalJointCtr);
    };
#include <factors/AngleBetweenAxisAndSegmentFactor.h>
    virtual class AngleBetweenAxisAndSegmentFactor : gtsam::NoiseModelFactor {
        AngleBetweenAxisAndSegmentFactor(size_t axisKey, size_t v1Key, size_t v2Key, const double angle, gtsam::noiseModel::Diagonal* model);
        Vector evaluateError(const gtsam::Unit3& axis, const gtsam::Point3& v1, const gtsam::Point3& v2);
    };
    virtual class MinAngleBetweenAxisAndSegmentFactor : gtsam::NoiseModelFactor {
        AngleBetweenAxisAndSegmentFactor(size_t axisKey, size_t v1Key, size_t v2Key, const double minAngle, gtsam::noiseModel::Diagonal* model);
        Vector evaluateError(const gtsam::Unit3& axis, const gtsam::Point3& v1, const gtsam::Point3& v2);
    };
    virtual class MaxAngleBetweenAxisAndSegmentFactor : gtsam::NoiseModelFactor {
        AngleBetweenAxisAndSegmentFactor(size_t axisKey, size_t v1Key, size_t v2Key, const double maxAngle, gtsam::noiseModel::Diagonal* model);
        Vector evaluateError(const gtsam::Unit3& axis, const gtsam::Point3& v1, const gtsam::Point3& v2);
    };
#include <factors/Pose3Priors.h>
    virtual class Pose3TranslationPrior : gtsam::NoiseModelFactor {
        Pose3TranslationPrior(size_t pose, const Vector& prior, gtsam::noiseModel::Diagonal* model);
        Vector evaluateError(const gtsam::Pose3& pose);
    };
    virtual class Pose3CompassPrior : gtsam::NoiseModelFactor {
        Pose3CompassPrior(size_t pose,  const Vector& refVecLocal, const Vector& refVecNav, const Vector& upVecNav, double expectedAng, gtsam::noiseModel::Diagonal* model);
        Vector evaluateError(const gtsam::Pose3& pose);
    };
} // namespace bioslam
