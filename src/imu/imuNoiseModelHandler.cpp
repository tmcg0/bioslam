// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include "imu/imuNoiseModelHandler.h"

imuNoiseModel imuNoiseModelHandler::model;

void imuNoiseModelHandler::applyModelToPreintMeasParams(boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p){
    if(!model.isSet){throw std::runtime_error("imuNoiseModel has not been set!");}
    // apply the imuNoiseModel to the boost::shared_ptr for the PreintegratedCombinedMeasurements
    p->accelerometerCovariance = gtsam::Matrix33::Identity(3,3) * pow(model.accelNoiseSigma,2.0); // acc white noise in continuous
    p->integrationCovariance = gtsam::Matrix33::Identity(3,3)* pow(model.integrationErrSigma,2.0); // integration uncertainty continuous
    p->gyroscopeCovariance = gtsam::Matrix33::Identity(3,3) * pow(model.gyroNoiseSigma,2.0); // gyro white noise in continuous
    // for use with combinedImuFactor:
    p->biasAccCovariance = gtsam::Matrix33::Identity(3,3) * pow(model.accelBiasRandomWalkSigma,2.0); // unset default is eye(3)
    p->biasOmegaCovariance = gtsam::Matrix33::Identity(3,3) * pow(model.gyroBiasRandomWalkSigma,2.0); // unset default is eye(3)
    p->biasAccOmegaInt = gtsam::Matrix66::Identity(6,6)* pow(model.preintBiasSigma,2.0); // variance of bias used for pre-integration
}

void imuNoiseModelHandler::applyModelToPreintMeasParams(boost::shared_ptr<gtsam::PreintegrationParams> p){
    if(!model.isSet){throw std::runtime_error("imuNoiseModel has not been set!");}
    // apply the imuNoiseModel to the boost::shared_ptr for the PreintegratedImuMeasurements
    p->accelerometerCovariance = gtsam::Matrix33::Identity(3,3) * pow(model.accelNoiseSigma,2.0); // acc white noise in continuous
    p->integrationCovariance = gtsam::Matrix33::Identity(3,3)* pow(model.integrationErrSigma,2.0); // integration uncertainty continuous
    p->gyroscopeCovariance = gtsam::Matrix33::Identity(3,3) * pow(model.gyroNoiseSigma,2.0); // gyro white noise in continuous
}

void imuNoiseModelHandler::print(){
    std::cout<<"imuNoiseModelHandler with model:"<<std::endl;
    std::cout<<"\taccel noise std="<<this->model.accelNoiseSigma<<std::endl;
    std::cout<<"\tgyro noise std="<<this->model.gyroNoiseSigma<<std::endl;
    std::cout<<"\tintegration uncertainty std="<<this->model.integrationErrSigma<<std::endl;
    std::cout<<"\taccel bias random walk std="<<this->model.accelBiasRandomWalkSigma<<std::endl;
    std::cout<<"\tgyro bias random walk std="<<this->model.gyroBiasRandomWalkSigma<<std::endl;
    std::cout<<"\tpreintegration bias std="<<this->model.preintBiasSigma<<std::endl;
}