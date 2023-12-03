// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// ----------------------------------------------------------------------------
// A factor to represent 3D magnetometer measurements for integration with the imu pose estimation problem.
// This model assumes zero bias in the magnetometer measurement.
//-------------------------------------------------------------------------------

#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace bioslam {
    class MagPose3Factor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
        const gtsam::Point3 m_measurement; //< The measured magnetometer values (local frame)
        const gtsam::Point3 B_Global; //< Global magnetic field definition
        const gtsam::Point3 bias_; //< bias (unused in MagPose3Factor)
    public:
        /* Constructor */
        MagPose3Factor(gtsam::Key pose3key, const gtsam::Point3 &measured, double scale, const gtsam::Unit3 &direction, const gtsam::Point3 &bias, const gtsam::SharedNoiseModel &model) :
                gtsam::NoiseModelFactorN<gtsam::Pose3>(model, pose3key), m_measurement(measured), B_Global(scale * direction), bias_(bias) {}

        gtsam::Vector evaluateError(const gtsam::Pose3 &pose_k, boost::optional<gtsam::Matrix &> H = boost::none) const;

        static gtsam::Vector3 errorModel(const gtsam::Pose3& x, const gtsam::Vector3& meas, const gtsam::Vector3& bN, boost::optional<gtsam::Matrix36&> H_x=boost::none);

        virtual ~MagPose3Factor() = default; // trivial deconstructor
        typedef boost::shared_ptr<MagPose3Factor> shared_ptr;  // shorthand for a smart pointer to a factor
        // return a deep copy of this factor
        virtual NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new MagPose3Factor(*this)));
        }
    };
}