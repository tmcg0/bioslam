// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// example usage of lowerBodyPoseEstimator

#include "imu/imu.h"
#include "lowerBodyPoseEstimator.h"
#include <string>
#include <boost/filesystem.hpp>
#include "testutils.h"
#include <VarStrToCharMap.h>

int main(){
    // --- example settings (there are other settings, too) --- //
    Eigen::Vector3d mag_G(19.5563,5.0972,-47.9409), acc_G(0.0,0.0,-9.81); // global magnetic field and acceleration definitions
    bool useMagnetometer=false, useConstrainedJointVelocity=false;
    Eigen::Vector3d magnetometerNoise(1000.0,1000.0,1000.0);
    uint imuPoseEstimatorInitializationScheme=0; // drives how imuPoseEstimator is initialized: 0 for zeros, 1 for FBEKF
    uint lbpeInitializationScheme=0; // lowerBodyPoseEstimator initialization: 0 for nothing, 1 to apply sacrum IMU's yaw angle to other IMUs, 2 for yaw search of reasonable joint angles
    uint numIpeIterations=0; // if zero, ipe won't optimize at all. any number>0 it will permit this many iterations (this is done *before* those values are added to the lowerBodyPoseEstimator graph)
    uint imuBiasMode=1; // 0 for static gyro/accel bias, 1 for dynamic bias
    gtsam::Vector3 sacrumImuCompassRefVec(0.0,1.0,0.0), sacrumImuNavRefVec(1.0,0.0,0.0); // i.e., sacrum Z axis should point towards global [1 0 0]
    bool useSacrumImuCompassPrior=true; // without magnetometers, it is useful to define a prior on at least one IMU heading angle
    // -------------------------------------------------------- //

    VarStrToCharMap::clear(); // good idea to clear the variable map before running estimators

    // pull out IMUs from data
    std::map<std::string,imu> ImuMap=imu::getImuMapFromDataFile(testutils::getTestDataFile("20170411-154746-Y1_TUG_6.h5"));
    imu sacrumImu=ImuMap["Sacrum"], rightThighImu=ImuMap["Right Thigh"], rightShankImu=ImuMap["Right Tibia"], rightFootImu=ImuMap["Right Foot"], leftThighImu=ImuMap["Left Thigh"], leftShankImu=ImuMap["Left Tibia"], leftFootImu=ImuMap["Left Foot"];

    // setup and run individual imu pose problems
    imuPoseEstimator sacrumImuPoseProblem(sacrumImu,"sacrum");
    sacrumImuPoseProblem.m_globalB=mag_G; sacrumImuPoseProblem.m_globalAcc=acc_G;
    sacrumImuPoseProblem.m_useCompassPrior=useSacrumImuCompassPrior;
    sacrumImuPoseProblem.m_refVecLocal=sacrumImuCompassRefVec; sacrumImuPoseProblem.m_refVecNav=sacrumImuNavRefVec;
    sacrumImuPoseProblem.setImuBiasModelMode(imuBiasMode);
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
    rthighImuPoseProblem.setImuBiasModelMode(imuBiasMode);
    rthighImuPoseProblem.m_usePositionPrior=false; rthighImuPoseProblem.m_useVelocityPrior=false; rthighImuPoseProblem.m_useImuBiasPrior=false;
    rthighImuPoseProblem.m_globalB=mag_G; rthighImuPoseProblem.m_globalAcc=acc_G;
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
    rshankImuPoseProblem.setImuBiasModelMode(imuBiasMode);
    rshankImuPoseProblem.m_usePositionPrior=false; rshankImuPoseProblem.m_useVelocityPrior=false; rshankImuPoseProblem.m_useImuBiasPrior=false;
    rshankImuPoseProblem.m_globalB=mag_G; rshankImuPoseProblem.m_globalAcc=acc_G;
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
    rfootImuPoseProblem.setImuBiasModelMode(imuBiasMode);
    rfootImuPoseProblem.m_usePositionPrior=false; rfootImuPoseProblem.m_useVelocityPrior=false; rfootImuPoseProblem.m_useImuBiasPrior=false;
    rfootImuPoseProblem.m_globalB=mag_G; rfootImuPoseProblem.m_globalAcc=acc_G;
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
    lthighImuPoseProblem.setImuBiasModelMode(imuBiasMode);
    lthighImuPoseProblem.m_usePositionPrior=false; lthighImuPoseProblem.m_useVelocityPrior=false; lthighImuPoseProblem.m_useImuBiasPrior=false;
    lthighImuPoseProblem.m_globalB=mag_G; lthighImuPoseProblem.m_globalAcc=acc_G;
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
    lshankImuPoseProblem.setImuBiasModelMode(imuBiasMode);
    lshankImuPoseProblem.m_usePositionPrior=false; lshankImuPoseProblem.m_useVelocityPrior=false; lshankImuPoseProblem.m_useImuBiasPrior=false;
    lshankImuPoseProblem.m_globalB=mag_G; lshankImuPoseProblem.m_globalAcc=acc_G;
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
    lfootImuPoseProblem.setImuBiasModelMode(imuBiasMode);
    lfootImuPoseProblem.m_usePositionPrior=false; lfootImuPoseProblem.m_useVelocityPrior=false; lfootImuPoseProblem.m_useImuBiasPrior=false;
    lfootImuPoseProblem.m_globalB=mag_G; lfootImuPoseProblem.m_globalAcc=acc_G;
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

    // setup lower body pose estimator
    lowerBodyPoseEstimator lbpe(sacrumImuPoseProblem,rthighImuPoseProblem,rshankImuPoseProblem,rfootImuPoseProblem,lthighImuPoseProblem,lshankImuPoseProblem,lfootImuPoseProblem,"example"); // construct
    // --- apply above lowerBodyPoseEstimator settings --- //
    lbpe.m_initializationScheme=lbpeInitializationScheme;
    lbpe.m_useJointVelFactor=useConstrainedJointVelocity;
    // --------------------------------------------------- //
    lbpe.setup(); // sets up lowerBodyPoseEstimator factor graph
    lbpe.fastOptimize(); // by default solves via global Levenberg-Marquardt. saves results to lbpe.m_estimate
    lbpe.saveResultsToH5File(boost::filesystem::current_path().append("example_results.h5").string()); // write results to h5 file

    return 0;
}