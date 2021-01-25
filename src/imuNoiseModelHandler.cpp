// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include "imuNoiseModelHandler.h"

imuBiasModel imuNoiseModelHandler::model;

void imuNoiseModelHandler::applyModelToPreintMeasParams(boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p, bool printVerbose){
    if(!model.isSet){std::cerr<<"imubiasmodel has not been set!"<<std::endl; return;}
    // apply the imuBiasModel to the boost::shared_ptr for the PreintegratedCombinedMeasurements
    p->accelerometerCovariance = gtsam::Matrix33::Identity(3,3) * pow(model.accelNoiseSigma,2); // acc white noise in continuous
    p->integrationCovariance = gtsam::Matrix33::Identity(3,3)*model.integrationErrSigma; // integration uncertainty continuous
    p->gyroscopeCovariance = gtsam::Matrix33::Identity(3,3) * pow(model.gyroNoiseSigma,2); // gyro white noise in continuous
    // these are for using combinedImuFactor:
    p->biasAccCovariance = gtsam::Matrix33::Identity(3,3) * pow(model.accelBiasRandomWalkSigma,2); // unset default is eye(3)
    p->biasOmegaCovariance = gtsam::Matrix33::Identity(3,3) * pow(model.gyroBiasRandomWalkSigma,2); // unset default is eye(3)
    p->biasAccOmegaInt = gtsam::Matrix66::Identity(6,6)*model.preintBiasSigma; // variance of bias used for pre-integration
    if(printVerbose){
        std::cout<<"set PreintegratedCombinedMeasurements to: accelerometerCovariance="<<p->accelerometerCovariance<<std::endl;
    }
}

void imuNoiseModelHandler::applyModelToPreintMeasParams(boost::shared_ptr<gtsam::PreintegrationParams> p, bool printVerbose){
    if(!model.isSet){std::cerr<<"imubiasmodel has not been set!"<<std::endl; return;}
    // apply the imuBiasModel to the boost::shared_ptr for the PreintegratedCombinedMeasurements
    p->accelerometerCovariance = gtsam::Matrix33::Identity(3,3) * pow(model.accelNoiseSigma,2); // acc white noise in continuous
    p->integrationCovariance = gtsam::Matrix33::Identity(3,3)*model.integrationErrSigma; // integration uncertainty continuous
    p->gyroscopeCovariance = gtsam::Matrix33::Identity(3,3) * pow(model.gyroNoiseSigma,2); // gyro white noise in continuous
    if(printVerbose){
        std::cout<<"set PreintegratedImuMeasurements to: accelerometerCovariance="<<p->accelerometerCovariance<<std::endl;
    }
}

void imuNoiseModelHandler::print(){
    // todo: finish filling out this print method
    std::cout<<"imuNoiseModelHandler with model: accelNoiseSigma="<<model.accelNoiseSigma<<std::endl;
}