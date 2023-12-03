// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// --------
// SOLUTION TEST
// this tests general estimation of human lower body kinematics
// right now, only tests that we can drive global error low
// --------

#include "imu/imu.h"
#include "lowerBodyPoseEstimator.h"
#include <string>
#include "testutils.h"
#include <VarStrToCharMap.h>

int main(){
    // --- test parameters --- //
    double finalMaxOptError=35.0; // in reality the converged error is something like 25
    // ----------------------- //
    // --- settings --- //
    bool useMagnetometer=false;
    Eigen::Vector3d magnetometerNoise(1000.0,1000.0,1000.0);
    uint imuPoseEstimatorInitializationScheme=0; // 0 for zeros, 1 for FBEKF
    uint lbpeInitializationScheme=0; // lowerBodyPoseEstimator initialization: 0 for nothing, 1 to apply sacrum IMU's yaw angle to other IMUs, 2 for yaw search of reasonable joint angles
    uint numIpeIterations=0; // if zero, ipe won't optimize at all. any number>0 it will permit this many iterations
    // ---------------- //
    VarStrToCharMap::clear();
    std::map<std::string,imu> ImuMap=imu::getImuMapFromDataFile(testutils::getTestDataFile("20170411-154746-Y1_TUG_6.h5"));
    // pull out imus
    imu sacrumImu=ImuMap["Sacrum"], rightThighImu=ImuMap["Right Thigh"], rightShankImu=ImuMap["Right Tibia"], rightFootImu=ImuMap["Right Foot"], leftThighImu=ImuMap["Left Thigh"], leftShankImu=ImuMap["Left Tibia"], leftFootImu=ImuMap["Left Foot"];
    // setup and run imu pose problems
    imuPoseEstimator sacrumImuPoseProblem(sacrumImu,"sacrum");
    sacrumImuPoseProblem.setImuBiasModelMode(1);
    sacrumImuPoseProblem.m_usePositionPrior=true; sacrumImuPoseProblem.m_useVelocityPrior=true; sacrumImuPoseProblem.m_useImuBiasPrior=false;
    sacrumImuPoseProblem.m_useMagnetometer=useMagnetometer;
    sacrumImuPoseProblem.m_magnetometerNoise=magnetometerNoise;
    sacrumImuPoseProblem.m_initializationScheme=imuPoseEstimatorInitializationScheme;
    sacrumImuPoseProblem.setup();
    if(numIpeIterations>0){
        sacrumImuPoseProblem.m_maxIterations=numIpeIterations;
        sacrumImuPoseProblem.fastOptimize();
    }else{ // just set optimized values to initial values
        sacrumImuPoseProblem.setOptimizedValuesToInitialValues();
    }

    imuPoseEstimator rthighImuPoseProblem(rightThighImu, "rthigh");
    rthighImuPoseProblem.setImuBiasModelMode(1);
    rthighImuPoseProblem.m_usePositionPrior=false; rthighImuPoseProblem.m_useVelocityPrior=false; rthighImuPoseProblem.m_useImuBiasPrior=false;
    rthighImuPoseProblem.m_useMagnetometer=useMagnetometer;
    rthighImuPoseProblem.m_magnetometerNoise=magnetometerNoise;
    rthighImuPoseProblem.m_initializationScheme=imuPoseEstimatorInitializationScheme;
    rthighImuPoseProblem.setup();
    if(numIpeIterations>0){
        rthighImuPoseProblem.m_maxIterations=numIpeIterations;
        rthighImuPoseProblem.fastOptimize();
    }else{ // just set optimized values to initial values
        rthighImuPoseProblem.setOptimizedValuesToInitialValues();
    }

    imuPoseEstimator rshankImuPoseProblem(rightShankImu, "rshank");
    rshankImuPoseProblem.setImuBiasModelMode(1);
    rshankImuPoseProblem.m_usePositionPrior=false; rshankImuPoseProblem.m_useVelocityPrior=false; rshankImuPoseProblem.m_useImuBiasPrior=false;
    rshankImuPoseProblem.m_useMagnetometer=useMagnetometer;
    rshankImuPoseProblem.m_magnetometerNoise=magnetometerNoise;
    rshankImuPoseProblem.m_initializationScheme=imuPoseEstimatorInitializationScheme;
    rshankImuPoseProblem.setup();
    if(numIpeIterations>0){
        rshankImuPoseProblem.m_maxIterations=numIpeIterations;
        rshankImuPoseProblem.fastOptimize();
    }else{ // just set optimized values to initial values
        rshankImuPoseProblem.setOptimizedValuesToInitialValues();
    }

    imuPoseEstimator rfootImuPoseProblem(rightFootImu, "rfoot");
    rfootImuPoseProblem.setImuBiasModelMode(1);
    rfootImuPoseProblem.m_usePositionPrior=false; rfootImuPoseProblem.m_useVelocityPrior=false; rfootImuPoseProblem.m_useImuBiasPrior=false;
    rfootImuPoseProblem.m_useMagnetometer=useMagnetometer;
    rfootImuPoseProblem.m_magnetometerNoise=magnetometerNoise;
    rfootImuPoseProblem.m_initializationScheme=imuPoseEstimatorInitializationScheme;
    rfootImuPoseProblem.setup();
    if(numIpeIterations>0){
        rfootImuPoseProblem.m_maxIterations=numIpeIterations;
        rfootImuPoseProblem.fastOptimize();
    }else{ // just set optimized values to initial values
        rfootImuPoseProblem.setOptimizedValuesToInitialValues();
    }

    imuPoseEstimator lthighImuPoseProblem(leftThighImu, "lthigh");
    lthighImuPoseProblem.setImuBiasModelMode(1);
    lthighImuPoseProblem.m_usePositionPrior=false; lthighImuPoseProblem.m_useVelocityPrior=false; lthighImuPoseProblem.m_useImuBiasPrior=false;
    lthighImuPoseProblem.m_useMagnetometer=useMagnetometer;
    lthighImuPoseProblem.m_magnetometerNoise=magnetometerNoise;
    lthighImuPoseProblem.m_initializationScheme=imuPoseEstimatorInitializationScheme;
    lthighImuPoseProblem.setup();
    if(numIpeIterations>0){
        lthighImuPoseProblem.m_maxIterations=numIpeIterations;
        lthighImuPoseProblem.fastOptimize();
    }else{ // just set optimized values to initial values
        lthighImuPoseProblem.setOptimizedValuesToInitialValues();
    }

    imuPoseEstimator lshankImuPoseProblem(leftShankImu, "lshank");
    lshankImuPoseProblem.setImuBiasModelMode(1);
    lshankImuPoseProblem.m_usePositionPrior=false; lshankImuPoseProblem.m_useVelocityPrior=false; lshankImuPoseProblem.m_useImuBiasPrior=false;
    lshankImuPoseProblem.m_useMagnetometer=useMagnetometer;
    lshankImuPoseProblem.m_magnetometerNoise=magnetometerNoise;
    lshankImuPoseProblem.m_initializationScheme=imuPoseEstimatorInitializationScheme;
    lshankImuPoseProblem.setup();
    if(numIpeIterations>0){
        lshankImuPoseProblem.m_maxIterations=numIpeIterations;
        lshankImuPoseProblem.fastOptimize();
    }else{ // just set optimized values to initial values
        lshankImuPoseProblem.setOptimizedValuesToInitialValues();
    }

    imuPoseEstimator lfootImuPoseProblem(leftFootImu, "lfoot");
    lfootImuPoseProblem.setImuBiasModelMode(1);
    lfootImuPoseProblem.m_usePositionPrior=false; lfootImuPoseProblem.m_useVelocityPrior=false; lfootImuPoseProblem.m_useImuBiasPrior=false;
    lfootImuPoseProblem.m_useMagnetometer=useMagnetometer;
    lfootImuPoseProblem.m_magnetometerNoise=magnetometerNoise;
    lfootImuPoseProblem.m_initializationScheme=imuPoseEstimatorInitializationScheme;
    lfootImuPoseProblem.setup();
    if(numIpeIterations>0){
        lfootImuPoseProblem.m_maxIterations=numIpeIterations;
        lfootImuPoseProblem.fastOptimize();
    }else{ // just set optimized values to initial values
        lfootImuPoseProblem.setOptimizedValuesToInitialValues();
    }

    // setup pose estimator
    lowerBodyPoseEstimator lbpe(sacrumImuPoseProblem,rthighImuPoseProblem,rshankImuPoseProblem,rfootImuPoseProblem,lthighImuPoseProblem,lshankImuPoseProblem,lfootImuPoseProblem,"example");
    lbpe.m_initializationScheme=lbpeInitializationScheme;
    lbpe.m_absErrLimit=finalMaxOptError*0.99; // to make it quit early once you've passed the test
    lbpe.setup();
    lbpe.fastOptimize();

    // check conditions for test
    if(lbpe.m_optimizationTotalError[lbpe.m_optimizationTotalError.size()-1] > finalMaxOptError){
        // error is too high, it didn't converge
        throw std::runtime_error("final error too high!");
    }
    return 0;
}