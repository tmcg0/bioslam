// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// class to hold noise parameters for a physical IMU sensor
// some relevant links on how these should be defined/calculated:
//    - https://github.com/borglab/gtsam/issues/213
//    - https://groups.google.com/forum/?utm_medium=email&utm_source=footer#!msg/gtsam-users/o9brW87ZjZ0/dWOigRcjBAAJl

#pragma once

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>

struct imuBiasModel{ // the struct which actually holds the parameters of the model
    // these are specified as scalars which represent the sigma (std) of continuous-time noise. when applied to the PreintegrationParams,
    // they are squared and put in the diagonals of a 3x3 matrix to represent the continuous-time covariance
    // remember: sigma_discrete = sigma_continuous * 1/(sqrt(dt))
    //     --> sigma_continuous = sigma_discrete * sqrt(dt)
    double accelNoiseSigma = 0.0;
    double accelBiasRandomWalkSigma = 0.0;
    double gyroNoiseSigma = 0.0;
    double gyroBiasRandomWalkSigma = 0.0;
    double preintBiasSigma = 0.0; // variance of bias used for pre-integration
    double integrationErrSigma = 0.0;  // error committed in integrating position from velocities
    bool isSet=false; // has the imubiasmodel been set?
    void setAsOpalv2Default(){ // bias model for the APDM Opal v2 IMUs
        accelNoiseSigma=0.04905; // from datasheet for v2: [acc1, acc2] noise density = [150, 5000] ug/sqrt(Hz) (converted 5000 ug/sqrt(Hz) to m/s^2/sqrt(Hz)
        accelBiasRandomWalkSigma=0.005905; // no info on datasheet for acc1 or acc2 bias stability. this is a placeholder number
        gyroNoiseSigma=0.000157079633; // from datasheet for v2: gyro noise density = 9 mdps/sqrt(Hz) (converted to rad/s/sqrt(Hz)
        gyroBiasRandomWalkSigma=1.93925e-5; // from datasheet for v2: gyro bias stability = 4 deg/hr (converted to rad/s^2/sqrt(Hz)
        integrationErrSigma=1.0e-8; // this has an effective minimum. When dropping it below 1.0e-8 I don't see any numerically difference in the covariance matrix which is built from it.
        preintBiasSigma=1.0e-5;
    }
};

class imuNoiseModelHandler {
public:
    static imuBiasModel model;
    void Opalv2(){ // construct for Opal v2
        model.setAsOpalv2Default();
        model.isSet=true;
    };
    void applyModelToPreintMeasParams(boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p, bool printVerbose=false);
    // what I called gtsam::CombinedPreintegrationParams (added a broken out class) was originally gtsam::PreintegratedCombinedMeasurements::Params
    void applyModelToPreintMeasParams(boost::shared_ptr<gtsam::PreintegrationParams> p, bool printVerbose=false);
    static void print();
}; // class imuBiasModelHander

