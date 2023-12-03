// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// test the setup of an IMU and its pose estimation via imuPoseEstimator

#include <vector>
#include "imu/imu.h"
#include "imuPoseEstimator.h"
#include <string>
#include <gtsam/navigation/NavState.h>
#include "testutils.h"
#include "gtsamutils.h"

imuPoseEstimator test_posthoc_smoother(const std::string& dataFileToUse, const std::string& imuLabel);

int main(){
    
    // (1) attempt to construct imu object
    std::string dataFileToUse=testutils::getTestDataFile("20170411-154746-Y1_TUG_6.h5");
    std::map<std::string,imu> ImuMap=imu::getImuMapFromDataFile(testutils::getTestDataFile("20170411-154746-Y1_TUG_6.h5"));
    imu::printLabelsInFile(dataFileToUse);
    std::string imuLabel="Right Thigh";
    imu testImu=ImuMap[imuLabel];
    testutils::runtime_assert(testImu.length()>0);

    // (2) run test methods
    test_posthoc_smoother(dataFileToUse,imuLabel);
    std::cout<<"completed imuPoseEstimator unit test!"<<std::endl;
    return 0;
}


imuPoseEstimator test_posthoc_smoother(const std::string& dataFileToUse, const std::string& imuLabel){
    std::map<std::string,imu> ImuMap=imu::getImuMapFromDataFile(dataFileToUse);
    imu testImu=ImuMap[imuLabel];
    std::cout<<"found imu! no. of measurements= "<<testImu.length()<<std::endl;
    std::vector<gtsam::Rot3> qAPDM=gtsamutils::imuOrientation(testImu);
    imuPoseEstimator poseProblem(testImu, "unitTest");
    poseProblem.setImuBiasModelMode(1); // 0 for static bias, 1 for dynamic (quasi-constant) bias model
    poseProblem.m_useMagnetometer=false;
    //poseProblem.m_priorOrientationNoise=Eigen::Vector3d(10.,10.,10.);
    //poseProblem.m_priorAccelBiasConstantNoise=Eigen::Vector3d(1.0e-1,1.0e-1,1.0e-1);
    //poseProblem.m_priorGyroBiasConstantNoise=Eigen::Vector3d(1.0e-1,1.0e-1,1.0e-1);
    poseProblem.setup();
    //VarStrToCharMap::print_graph(poseProblem.m_graph);
    // for speed, shorten the max number of iterations. this is just a unit test, after all, not a solution test.
    poseProblem.m_maxIterations=5;
    poseProblem.fastOptimize();
    // check for any infinity errors
    for(uint i=0;i<poseProblem.m_optimizationTotalError.size();i++){
        if(std::isinf(poseProblem.m_optimizationTotalError[i])){
            throw std::runtime_error("optimizer error is infinity!");
        }
    }
    std::cout<<"posthoc smoother ran successfully"<<std::endl;
    return poseProblem;
}







