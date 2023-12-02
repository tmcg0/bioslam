// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// unit test of imuNoiseModelHandler

#include "imu/imuNoiseModelHandler.h"

int main(){
    Eigen::Vector3d acc_G(0.0,0.0,9.81);
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p =boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>(new gtsam::PreintegratedCombinedMeasurements::Params(acc_G));
    imuNoiseModelHandler sensorBiasModel;
    sensorBiasModel.Opalv2();
    gtsam::Matrix33 oldAccelCov=p->accelerometerCovariance;
    std::cout<<"preintegratedcombinedmeasurements before application of bias model:"<<oldAccelCov<<std::endl;
    sensorBiasModel.applyModelToPreintMeasParams(p);
    sensorBiasModel.print();
    gtsam::Matrix33 newAccelCov=p->accelerometerCovariance;
    std::cout<<"preintegratedcombinedmeasurements after application of bias model:"<<newAccelCov<<std::endl;
    // test: assert that it changes
    bool didChange=!gtsam::assert_equal(oldAccelCov,newAccelCov,1.0e-1);
    if(!didChange){
        throw std::runtime_error("accelerometer covariance did not change when IMU noise model was appplied.");
    }
    // also test with normal preintegrated imu measurements (not combined)
    boost::shared_ptr<gtsam::PreintegrationParams> p2 =boost::shared_ptr<gtsam::PreintegrationParams>(new gtsam::PreintegrationParams(acc_G));
    oldAccelCov=p2->accelerometerCovariance;
    sensorBiasModel.applyModelToPreintMeasParams(p2);
    sensorBiasModel.print();
    newAccelCov=p2->accelerometerCovariance;
    // test: assert that it changes
    bool didChange2=!gtsam::assert_equal(oldAccelCov,newAccelCov,1.0e-1);
    if(!didChange2){
        throw std::runtime_error("accelerometer covariance did not change when IMU noise model was appplied.");
    }
    return 0;
}