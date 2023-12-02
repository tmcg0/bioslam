// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// example usage of imuPoseEstimator

#include "imu/imu.h"
#include "imuPoseEstimator.h"
#include <string>
#include <gtsam/navigation/NavState.h>
#include "VarStrToCharMap.h"
#include "testutils.h"

int main(){
    // --- example settings (there are other settings, too) --- //
    Eigen::Vector3d mag_G(19.5563,5.0972,-47.9409), acc_G(0.0,0.0,-9.81); // global magnetic field and acceleration definitions
    bool useMagnetometer=false;
    Eigen::Vector3d magnetometerNoise(1000.0,1000.0,1000.0);
    uint imuBiasModelMode=0; // 0 for static bias, 1 for dynamic (quasi-constant) bias model
    uint initializationScheme=1; // 0 for zeros, 1 for EKF
    bool useCompassPrior=true;
    // -------------------------------------------------------- //

    VarStrToCharMap::clear(); // good idea to clear the variable map before running estimators
    std::string dataFileToUse=testutils::getTestDataFile("20170411-154746-Y1_TUG_6.h5"); // specify the data file to use
    std::map<std::string,imu> ImuMap=imu::getImuMapFromDataFile(dataFileToUse); // gets the set of IMUs mapped to their string labels
    imu testImu=ImuMap["Right Thigh"]; // pull out the individual IMU from the set of IMUs in ImuMap

    imuPoseEstimator poseProblem(testImu, "unitTest"); // construct imuPoseEstimator

    // --- apply those earlier settings to the imuPoseEstimator --- //
    poseProblem.m_useMagnetometer=useMagnetometer;
    poseProblem.m_magnetometerNoise=magnetometerNoise;
    poseProblem.m_globalB=mag_G; poseProblem.m_globalAcc=acc_G;
    poseProblem.setImuBiasModelMode(imuBiasModelMode);
    poseProblem.m_initializationScheme=initializationScheme;
    poseProblem.m_useCompassPrior=useCompassPrior;
    // ------------------------------------------------------------ //

    poseProblem.setup(); // setup the factor graph for the imuPoseEstimator
    poseProblem.fastOptimize(); // default Levenberg-Marquardt global optimization. saves solution to poseProblem.m_estimate

    return 0;
}








