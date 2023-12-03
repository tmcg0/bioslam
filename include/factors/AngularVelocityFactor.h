// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

/* A factor to estimate the instantaneous angular velocity at a keyframe */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/ImuBias.h>

namespace bioslam {

    class AngularVelocityFactor : public gtsam::NoiseModelFactorN<gtsam::Vector3, gtsam::imuBias::ConstantBias> {

    public:
        // member variables for gyro measurements
        const gtsam::Vector3 m_measuredAngVel;
        // shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<AngularVelocityFactor> shared_ptr;

        AngularVelocityFactor(const gtsam::Key &estimatedAngVelKey, const gtsam::Key &imuBiasKey, const gtsam::Vector3& angVelMeas, const gtsam::SharedNoiseModel &model) :
                NoiseModelFactorN<gtsam::Vector3, gtsam::imuBias::ConstantBias>(model, estimatedAngVelKey, imuBiasKey),
                m_measuredAngVel(angVelMeas) {};

        gtsam::Vector evaluateError(const gtsam::Vector3 &estimatedAngVel, const gtsam::imuBias::ConstantBias &imuBias,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none, // <- also include optional derivatives
                                    boost::optional<gtsam::Matrix &> H2 = boost::none) const;

        // also need to override a second method. according to Frank Dellaert:
        // "The second is a 'clone' function that allows the factor to be copied. Under most
        // circumstances, the following code that employs the default copy constructor should
        // work fine."
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new AngularVelocityFactor(*this)));
        }

        virtual ~AngularVelocityFactor() = default; // trivial deconstructor

    }; // class
} // namespace bioslam