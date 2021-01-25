// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include <factors/AngularVelocityFactor.h>

namespace bioslam {

        gtsam::Vector AngularVelocityFactor::evaluateError(const gtsam::Vector3 &estimatedAngVel, const gtsam::imuBias::ConstantBias &imuBias,
                                                           boost::optional<gtsam::Matrix &> H1, // <- also include optional derivatives
                                                           boost::optional<gtsam::Matrix &> H2) const {
            // ------- error model of this factor ------- //
            // according to [1] Eq. 20:
            //    meas = true + bias + noise
            //        ==> error = true + bias - meas ~ N(0,sigma)
            const gtsam::Vector3 &gyroBias = imuBias.gyroscope(); // gyro bias is the LAST three
            gtsam::Vector3 error = estimatedAngVel + gyroBias - m_measuredAngVel;
            // ----- optionally handle derivatives ------ //
            if (H1) { // dErr/dEstimatedAngVel (3x3)
                *H1 = gtsam::Matrix33::Identity();
            }
            if (H2) { // dErr/dImuBias (3x6)
                // dErr/dImuBias = dErr/dGyroBias*dGyroBias/dImuBias
                gtsam::Matrix33 dErr_dGyroBias=gtsam::Matrix33::Identity();
                gtsam::Matrix36 dGyroBias_dImuBias=gtsam::Matrix36::Zero();
                dGyroBias_dImuBias.block<3,3>(0,3)=gtsam::Matrix33::Identity(); // set identity matrix on gyroscope component of the jacobian
                gtsam::Matrix36 dErr_dImuBias=dErr_dGyroBias*dGyroBias_dImuBias;
                *H2 = dErr_dImuBias;
            }
            return error;
        }
} // namespace bioslam

// References:
// [1]: Forster et al. "IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation"
// https://smartech.gatech.edu/bitstream/handle/1853/55417/IMU%20Preintegration%20on%20Manifold%20for%20Efficient.pdf?sequence=1&isAllowed=y