// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// a factor which constrains the velocity difference of a joint center

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsamutils.h>

namespace bioslam {
class ConstrainedJointCenterVelocityFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Point3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Point3> {
    public:
        /* Constructor */
        ConstrainedJointCenterVelocityFactor(const gtsam::Key& poseA, const gtsam::Key& linVelA, const gtsam::Key& angVelA, const gtsam::Key& sA, const gtsam::Key& poseB, const gtsam::Key& linVelB, const gtsam::Key& angVelB, const gtsam::Key& sB, const gtsam::SharedNoiseModel &model) :
                gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Point3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Point3>(model, poseA, linVelA, angVelA, sA, poseB, linVelB, angVelB, sB) {};

        gtsam::Vector evaluateError(const gtsam::Pose3& poseA, const gtsam::Vector3& linVelA, const gtsam::Vector3& angVelA, const gtsam::Point3& sA, const gtsam::Pose3& poseB, const gtsam::Vector3& linVelB, const gtsam::Vector3& angVelB, const gtsam::Point3& sB,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none, // include optional jacobians H
                                    boost::optional<gtsam::Matrix &> H2 = boost::none,
                                    boost::optional<gtsam::Matrix &> H3 = boost::none,
                                    boost::optional<gtsam::Matrix &> H4 = boost::none,
                                    boost::optional<gtsam::Matrix &> H5 = boost::none,
                                    boost::optional<gtsam::Matrix &> H6 = boost::none,
                                    boost::optional<gtsam::Matrix &> H7 = boost::none,
                                    boost::optional<gtsam::Matrix &> H8 = boost::none) const;

        gtsam::Vector evaluateErrorNoJacCall(const gtsam::Pose3& poseA, const gtsam::Vector3& linVelA, const gtsam::Vector3& angVelA, const gtsam::Point3& sA, const gtsam::Pose3& poseB, const gtsam::Vector3& linVelB, const gtsam::Vector3& angVelB, const gtsam::Point3& sB) const; // like the normal one, but without the boost::optional calls

        static gtsam::Vector3 errorModel(const gtsam::Pose3& xA, const gtsam::Vector3& vA, const gtsam::Vector3& wA, const gtsam::Point3& sA, const gtsam::Pose3& xB, const gtsam::Vector3& vB, const gtsam::Vector3& wB, const gtsam::Point3& sB,
                                    boost::optional<gtsam::Matrix36 &> H_xA = boost::none, // include optional jacobians H
                                    boost::optional<gtsam::Matrix33 &> H_vA = boost::none,
                                    boost::optional<gtsam::Matrix33 &> H_wA = boost::none,
                                    boost::optional<gtsam::Matrix33 &> H_sA = boost::none,
                                    boost::optional<gtsam::Matrix36 &> H_xB = boost::none,
                                    boost::optional<gtsam::Matrix33 &> H_vB = boost::none,
                                    boost::optional<gtsam::Matrix33 &> H_wB = boost::none,
                                    boost::optional<gtsam::Matrix33 &> H_sB = boost::none);

        // linear velocity of a rotating point in the navigation frame
        static gtsam::Vector3 linearVelocity(const gtsam::Pose3&x, const gtsam::Vector3& linVel, const gtsam::Vector3& angVel, const gtsam::Vector3& pt,
                                            boost::optional<gtsam::Matrix36 &> H_x = boost::none, // include optional jacobians H
                                            boost::optional<gtsam::Matrix33 &> H_linVel = boost::none,
                                            boost::optional<gtsam::Matrix33 &> H_angVel = boost::none,
                                            boost::optional<gtsam::Matrix33 &> H_pt = boost::none);

        // stuff required to add
        // return a deep copy of this factor
        virtual NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new ConstrainedJointCenterVelocityFactor(*this)));
        }
        virtual ~ConstrainedJointCenterVelocityFactor() = default; // trivial deconstructor
    };

    // ------------------------------------------------------------------------------------------------------------------ //
    // ------------------------------------ norm error model version of this factor ------------------------------------- //
    // ------------------------------------------------------------------------------------------------------------------ //
    class ConstrainedJointCenterNormVelocityFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Point3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Point3> {
    public:
        /* Constructor */
        ConstrainedJointCenterNormVelocityFactor(const gtsam::Key& poseA, const gtsam::Key& linVelA, const gtsam::Key& angVelA, const gtsam::Key& sA, const gtsam::Key& poseB, const gtsam::Key& linVelB, const gtsam::Key& angVelB, const gtsam::Key& sB, const gtsam::SharedNoiseModel &model) :
                gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Point3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3, gtsam::Point3>(model, poseA, linVelA, angVelA, sA, poseB, linVelB, angVelB, sB) {};

        gtsam::Vector evaluateError(const gtsam::Pose3& poseA, const gtsam::Vector3& linVelA, const gtsam::Vector3& angVelA, const gtsam::Point3& sA, const gtsam::Pose3& poseB, const gtsam::Vector3& linVelB, const gtsam::Vector3& angVelB, const gtsam::Point3& sB,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none, // include optional jacobians H
                                    boost::optional<gtsam::Matrix &> H2 = boost::none,
                                    boost::optional<gtsam::Matrix &> H3 = boost::none,
                                    boost::optional<gtsam::Matrix &> H4 = boost::none,
                                    boost::optional<gtsam::Matrix &> H5 = boost::none,
                                    boost::optional<gtsam::Matrix &> H6 = boost::none,
                                    boost::optional<gtsam::Matrix &> H7 = boost::none,
                                    boost::optional<gtsam::Matrix &> H8 = boost::none) const;

        gtsam::Vector evaluateErrorNoJacCall(const gtsam::Pose3& poseA, const gtsam::Vector3& linVelA, const gtsam::Vector3& angVelA, const gtsam::Point3& sA, const gtsam::Pose3& poseB, const gtsam::Vector3& linVelB, const gtsam::Vector3& angVelB, const gtsam::Point3& sB) const; // like the normal one, but without the boost::optional calls

        static gtsam::Vector1 errorModel(const gtsam::Pose3& xA, const gtsam::Vector3& vA, const gtsam::Vector3& wA, const gtsam::Point3& sA, const gtsam::Pose3& xB, const gtsam::Vector3& vB, const gtsam::Vector3& wB, const gtsam::Point3& sB,
                                         boost::optional<gtsam::Matrix16 &> H_xA = boost::none, // include optional jacobians H
                                         boost::optional<gtsam::Matrix13 &> H_vA = boost::none,
                                         boost::optional<gtsam::Matrix13 &> H_wA = boost::none,
                                         boost::optional<gtsam::Matrix13 &> H_sA = boost::none,
                                         boost::optional<gtsam::Matrix16 &> H_xB = boost::none,
                                         boost::optional<gtsam::Matrix13 &> H_vB = boost::none,
                                         boost::optional<gtsam::Matrix13 &> H_wB = boost::none,
                                         boost::optional<gtsam::Matrix13 &> H_sB = boost::none);

        // stuff required to add
        // return a deep copy of this factor
        virtual NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new ConstrainedJointCenterNormVelocityFactor(*this)));
        }
        virtual ~ConstrainedJointCenterNormVelocityFactor() = default; // trivial deconstructor
    };
}