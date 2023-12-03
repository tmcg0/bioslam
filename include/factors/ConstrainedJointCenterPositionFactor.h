// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// header of class ConstrainedJointCenterPositionFactor.h
/* This custom factor implement the position constraint between two IMU flanking a joint center.
 * It takes in two imu poses and then two points static in the IMU orientation frame which represent the distance to the joint center in meters
 * ASSUMPTIONS (DATA):
 * Poses are [body->nav], static Point3's are in body frame
 * ASSUMPTIONS (PHYSICS):
 * vectors from imu center to joint rotation center are static
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace bioslam {

    class ConstrainedJointCenterPositionFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3, gtsam::Point3, gtsam::Point3> {

    public:
        // shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<ConstrainedJointCenterPositionFactor> shared_ptr;

        ConstrainedJointCenterPositionFactor(gtsam::Key x_A_to_N, gtsam::Key x_B_to_N, gtsam::Key L_A_to_ctr, gtsam::Key L_B_to_ctr, const gtsam::SharedNoiseModel &model) :
                NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3, gtsam::Point3, gtsam::Point3>(model, x_A_to_N, x_B_to_N, L_A_to_ctr, L_B_to_ctr) {};

        virtual ~ConstrainedJointCenterPositionFactor() {}

        // override evaluteError()
        gtsam::Vector evaluateError(const gtsam::Pose3 &xA, const gtsam::Pose3 &xB, const gtsam::Point3 &L_A_to_ctr, const gtsam::Point3 &L_B_to_ctr,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none, // <- also include optional derivatives
                                    boost::optional<gtsam::Matrix &> H2 = boost::none,
                                    boost::optional<gtsam::Matrix &> H3 = boost::none,
                                    boost::optional<gtsam::Matrix &> H4 = boost::none) const;

        static gtsam::Vector3 errorModel(const gtsam::Pose3 &xA, const gtsam::Pose3 &xB, const gtsam::Point3 &vA, const gtsam::Point3 &vB,
                                    boost::optional<gtsam::Matrix36 &> H_xA = boost::none, // <- also include optional derivatives
                                    boost::optional<gtsam::Matrix36 &> H_xB = boost::none,
                                    boost::optional<gtsam::Matrix33 &> H_vA = boost::none,
                                    boost::optional<gtsam::Matrix33 &> H_vB = boost::none);

        // also need to override a second method. according to Frank Dellaert:
        // "The second is a 'clone' function that allows the factor to be copied. Under most
        // circumstances, the following code that employs the default copy constructor should
        // work fine."
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new ConstrainedJointCenterPositionFactor(*this)));
        }
    }; // classdef

    // ------ norm error version of factor ----- //

    class ConstrainedJointCenterNormPositionFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3, gtsam::Point3, gtsam::Point3> {

    public:
        double m_expectedDist=0.0;
        // shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<ConstrainedJointCenterNormPositionFactor> shared_ptr;

        // default constructor with zero expectedDist
        ConstrainedJointCenterNormPositionFactor(gtsam::Key x_A_to_N, gtsam::Key x_B_to_N, gtsam::Key L_A_to_ctr, gtsam::Key L_B_to_ctr, const gtsam::SharedNoiseModel &model) :
                NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3, gtsam::Point3, gtsam::Point3>(model, x_A_to_N, x_B_to_N, L_A_to_ctr, L_B_to_ctr) {};

        // constructor with non-zero expectedDist
        ConstrainedJointCenterNormPositionFactor(gtsam::Key x_A_to_N, gtsam::Key x_B_to_N, gtsam::Key L_A_to_ctr, gtsam::Key L_B_to_ctr, double expectedDist, const gtsam::SharedNoiseModel &model) :
                NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3, gtsam::Point3, gtsam::Point3>(model, x_A_to_N, x_B_to_N, L_A_to_ctr, L_B_to_ctr), m_expectedDist(expectedDist) {};

        virtual ~ConstrainedJointCenterNormPositionFactor() {}

        // override evaluteError()
        gtsam::Vector evaluateError(const gtsam::Pose3 &xA, const gtsam::Pose3 &xB, const gtsam::Point3 &L_A_to_ctr, const gtsam::Point3 &L_B_to_ctr,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none, // <- also include optional derivatives
                                    boost::optional<gtsam::Matrix &> H2 = boost::none,
                                    boost::optional<gtsam::Matrix &> H3 = boost::none,
                                    boost::optional<gtsam::Matrix &> H4 = boost::none) const;

        static gtsam::Vector1 errorModel(const gtsam::Pose3 &xA, const gtsam::Pose3 &xB, const gtsam::Point3 &vA, const gtsam::Point3 &vB, double expectedDist,
                                         boost::optional<gtsam::Matrix16 &> H_xA = boost::none, // <- also include optional derivatives
                                         boost::optional<gtsam::Matrix16 &> H_xB = boost::none,
                                         boost::optional<gtsam::Matrix13 &> H_vA = boost::none,
                                         boost::optional<gtsam::Matrix13 &> H_vB = boost::none);

        // also need to override a second method. according to Frank Dellaert:
        // "The second is a 'clone' function that allows the factor to be copied. Under most
        // circumstances, the following code that employs the default copy constructor should
        // work fine."
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new ConstrainedJointCenterNormPositionFactor(*this)));
        }
    }; // classdef

} // namespace bioslam