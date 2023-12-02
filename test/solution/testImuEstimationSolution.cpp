// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// a solution test for IMU pose estimation algorithms

#include "imu/imu.h"
#include "imuPoseEstimator.h"
#include <string>
#include <VarStrToCharMap.h>
#include "mathutils.h"
#include "testutils.h"

double discreteVelocityDerivativeComparisonRmse(const imuPoseEstimator& ipe);

int main(){
    // --- test parameters --- //
    double finalMaxOptError=1.0e-5;
    // ----------------------- //
    // --- example settings ---
    bool useMagnetometer=false;
    Eigen::Vector3d magnetometerNoise(1000.0,1000.0,1000.0);
    uint imuBiasModelMode=1; // 0 for static bias, 1 for dynamic (quasi-constant) bias model
    uint initializationScheme=0; // 0 for zeros, 1 for EKF
    // ------------------------
    VarStrToCharMap::clear();
    std::map<std::string,imu> ImuMap=imu::getImuMapFromDataFile(testutils::getTestDataFile("20170411-154746-Y1_TUG_6.h5"));
    imu testImu=ImuMap["Right Thigh"];

    imuPoseEstimator poseProblem(testImu, "solutionTest");

    poseProblem.m_useMagnetometer=useMagnetometer;
    poseProblem.m_magnetometerNoise=magnetometerNoise;
    poseProblem.setImuBiasModelMode(imuBiasModelMode);
    poseProblem.m_initializationScheme=initializationScheme;
    poseProblem.m_absErrLimit=finalMaxOptError*0.99; // to make it quit early once you've passed the test
    poseProblem.setup();
    poseProblem.fastOptimize();
    // test for convergence
    if(poseProblem.m_optimizationTotalError[poseProblem.m_optimizationTotalError.size()-1] > finalMaxOptError){
        // error is too high, it didn't converge
        throw std::runtime_error("final error too high!");
    }
    // test to make sure estimated velocity is similar to discrete derivative of position
    double velRmse=discreteVelocityDerivativeComparisonRmse(poseProblem);
    std::cout<<"velRmse="<<velRmse<<std::endl;
    if(velRmse>0.5){ // if greater than 0.5 m/s, that's pretty erroneous
        throw std::runtime_error("estimated velocity is fairly different from discrete derivative of estimated position.");
    }
    return 0;
}

double discreteVelocityDerivativeComparisonRmse(const imuPoseEstimator& ipe) {
    // how does the (a) discrete derivative of estimated position compare to (b) estimated velocity?
    // returns this difference as an RMSE
    Eigen::MatrixXd position=gtsamutils::Point3VectorToEigenMatrix(ipe.m_position);
    Eigen::MatrixXd velocity=gtsamutils::Vector3VectorToEigenMatrix(ipe.m_velocity);
    // get discrete velocity from position
    Eigen::MatrixXd velocityDiscrete(ipe.m_nKeyframes-1,3);
    for(uint k=0; k<ipe.m_nKeyframes-1;k++){
        velocityDiscrete.block<1,3>(k,0) = (position.block<1,3>(k+1,0) - position.block<1,3>(k,0)).transpose()/(ipe.m_time[1]-ipe.m_time[0]); // m/s
    }
    // find velocity difference
    Eigen::MatrixXd velocityErr(ipe.m_nKeyframes-1,3);
    std::vector<double> velocityErrNorm(ipe.m_nKeyframes-1), velocityErrNormSq(ipe.m_nKeyframes-1);
    for(uint k=0; k<ipe.m_nKeyframes-1; k++){
        velocityErr.block<1,3>(k,0)= velocity.block<1,3>(k,0) - velocityDiscrete.block<1,3>(k,0);
        velocityErrNorm[k]=(velocityErr.block<1,3>(k,0)).norm();
        velocityErrNormSq[k]=pow(velocityErrNorm[k],2.0);
    }
    // compute rmse
    double velocityRmse = sqrt(accumulate(velocityErrNormSq.begin(),velocityErrNormSq.end(),0.0)/velocityErrNormSq.size()); // units is m/s
    return velocityRmse;
}