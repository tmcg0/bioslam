// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// This factor constrains a hinge joint between two rotational frames. This version is expressed as a function of the angular velocity of these frames.
// there is both a vector error and norm error implementation of this factor.
 /* ASSUMPTIONS:
 * - rotations are expressed as [body->nav]
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <iostream>
#include <gtsam/geometry/Pose3.h>
#include "mathutils.h"

namespace bioslam {

class HingeJointConstraintVecErrEstAngVel : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Unit3> {
    // ------------------------------------------------------------------------------------ //
    // This factor constrains hinge-like rotation dynamics between the any two IMUs flanking a joint, for the proximal frame axis.
    // error is function of: Proximal IMU Pose (gtsam::Pose3), angular velocity of proximal IMU frame (gtsam::Vector3), Distal IMU Pose (gtsam::Pose3), angular velocity of distal IMU frame (gtsam::Vector3), hinge axis in proximal IMU frame (gtsam::Unit3)
    // ------------------------------------------------------------------------------------ //
    public:

        // shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<HingeJointConstraintVecErrEstAngVel> shared_ptr;

        HingeJointConstraintVecErrEstAngVel(const gtsam::Key &xA, const gtsam::Key& omegaA, const gtsam::Key &xB, const gtsam::Key& omegaB, const gtsam::Key &axisA, const gtsam::SharedNoiseModel &model):
                                                            NoiseModelFactorN<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Unit3>(model, xA, omegaA, xB, omegaB, axisA) {};
        // override evaluteError()
        gtsam::Vector evaluateError(const gtsam::Pose3 &xA, const gtsam::Vector3& omegaA, const gtsam::Pose3 &xB, const gtsam::Vector3& omegaB, const gtsam::Unit3 &axisA,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none, // <- also include optional derivatives
                                    boost::optional<gtsam::Matrix &> H2 = boost::none,
                                    boost::optional<gtsam::Matrix &> H3 = boost::none,
                                    boost::optional<gtsam::Matrix &> H4 = boost::none,
                                    boost::optional<gtsam::Matrix &> H5 = boost::none) const;

    gtsam::Vector evaluateErrorNoJacCall(const gtsam::Pose3 &xA, const gtsam::Vector3& omegaA, const gtsam::Pose3 &xB, const gtsam::Vector3& omegaB, const gtsam::Unit3 &axisA) const;

    static gtsam::Vector3 errorModel(const gtsam::Pose3& xA, const gtsam::Vector3& wA, const gtsam::Pose3& xB, const gtsam::Vector3& wB, const gtsam::Unit3& kA,
                                                                                boost::optional<gtsam::Matrix36 &> H_xA=boost::none,
                                                                                boost::optional<gtsam::Matrix33 &> H_wA=boost::none,
                                                                                boost::optional<gtsam::Matrix36 &> H_xB=boost::none,
                                                                                boost::optional<gtsam::Matrix33 &> H_wB=boost::none,
                                                                                boost::optional<gtsam::Matrix32 &> H_kA=boost::none);

    virtual ~HingeJointConstraintVecErrEstAngVel() = default; // trivial deconstructor
        // also need to override a second method. according to Frank Dellaert:
        // "The second is a 'clone' function that allows the factor to be copied. Under most circumstances, the following code that employs the default copy constructor should work fine."
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new HingeJointConstraintVecErrEstAngVel(*this)));
        }
    }; // classdef

    // -------------------------------------------------------------------------------------------------------------- //
    // ------------------------------------- norm error version of these factors ------------------------------------ //
    // -------------------------------------------------------------------------------------------------------------- //

    class HingeJointConstraintNormErrEstAngVel : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Unit3> {
        // ------------------------------------------------------------------------------------ //
        // This factor constrains hinge-like rotation dynamics between the any two IMUs flanking a joint, for the proximal frame axis.
        // error is function of: Proximal IMU Pose (gtsam::Pose3), angular velocity of proximal IMU frame (gtsam::Vector3), Distal IMU Pose (gtsam::Pose3), angular velocity of distal IMU frame (gtsam::Vector3), hinge axis in proximal IMU frame (gtsam::Unit3)
        // ------------------------------------------------------------------------------------ //
    public:

        // shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<HingeJointConstraintNormErrEstAngVel> shared_ptr;

        HingeJointConstraintNormErrEstAngVel(const gtsam::Key &xA, const gtsam::Key& omegaA, const gtsam::Key &xB, const gtsam::Key& omegaB, const gtsam::Key &axisA, const gtsam::SharedNoiseModel &model):
                NoiseModelFactorN<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Unit3>(model, xA, omegaA, xB, omegaB, axisA) {};
        // override evaluteError()
        gtsam::Vector evaluateError(const gtsam::Pose3 &xA, const gtsam::Vector3& omegaA, const gtsam::Pose3 &xB, const gtsam::Vector3& omegaB, const gtsam::Unit3 &axisA,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none, // <- also include optional derivatives
                                    boost::optional<gtsam::Matrix &> H2 = boost::none,
                                    boost::optional<gtsam::Matrix &> H3 = boost::none,
                                    boost::optional<gtsam::Matrix &> H4 = boost::none,
                                    boost::optional<gtsam::Matrix &> H5 = boost::none) const;

        gtsam::Vector evaluateErrorNoJacCall(const gtsam::Pose3 &xA, const gtsam::Vector3& omegaA, const gtsam::Pose3 &xB, const gtsam::Vector3& omegaB, const gtsam::Unit3 &axisA) const;

        static gtsam::Vector1 errorModel(const gtsam::Pose3& xA, const gtsam::Vector3& wA, const gtsam::Pose3& xB, const gtsam::Vector3& wB, const gtsam::Unit3& kA,
                                         boost::optional<gtsam::Matrix16 &> H_xA=boost::none,
                                         boost::optional<gtsam::Matrix13 &> H_wA=boost::none,
                                         boost::optional<gtsam::Matrix16 &> H_xB=boost::none,
                                         boost::optional<gtsam::Matrix13 &> H_wB=boost::none,
                                         boost::optional<gtsam::Matrix12 &> H_kA=boost::none);
        virtual ~HingeJointConstraintNormErrEstAngVel() = default; // trivial deconstructor
        // also need to override a second method. according to Frank Dellaert:
        // "The second is a 'clone' function that allows the factor to be copied. Under most circumstances, the following code that employs the default copy constructor should work fine."
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new HingeJointConstraintNormErrEstAngVel(*this)));
        }
    }; // classdef

} // namespace