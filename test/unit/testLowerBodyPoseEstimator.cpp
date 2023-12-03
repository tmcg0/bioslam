// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// unit test of lowerBodyPoseEstimator. Does not test solution, only tests basic functionality and interface.

#include "imu/imu.h"
#include "lowerBodyPoseEstimator.h"
#include <string>
#include "testutils.h"

int main(){
    std::map<std::string,imu> ImuMap=imu::getImuMapFromDataFile(testutils::getTestDataFile("20170411-154746-Y1_TUG_6.h5"));
    // pull out imus
    imu sacrumImu=ImuMap["Sacrum"];
    imu rightThighImu=ImuMap["Right Thigh"];
    imu rightShankImu=ImuMap["Right Tibia"];
    imu rightFootImu=ImuMap["Right Foot"];
    imu leftThighImu=ImuMap["Left Thigh"];
    imu leftShankImu=ImuMap["Left Tibia"];
    imu leftFootImu=ImuMap["Left Foot"];
    // setup and run imu pose problems
    imuPoseEstimator sacrumImuPoseProblem(sacrumImu,"sacrum");
    sacrumImuPoseProblem.setImuBiasModelMode(1);
    sacrumImuPoseProblem.setup(); sacrumImuPoseProblem.setOptimizedValuesToInitialValues();
    imuPoseEstimator rthighImuPoseProblem(rightThighImu, "rthigh");
    rthighImuPoseProblem.setImuBiasModelMode(1);
    rthighImuPoseProblem.setup(); rthighImuPoseProblem.setOptimizedValuesToInitialValues();
    imuPoseEstimator rshankImuPoseProblem(rightShankImu, "rshank");
    rshankImuPoseProblem.setImuBiasModelMode(1);
    rshankImuPoseProblem.setup(); rshankImuPoseProblem.setOptimizedValuesToInitialValues();
    imuPoseEstimator rfootImuPoseProblem(rightFootImu, "rfoot");
    rfootImuPoseProblem.setImuBiasModelMode(1);
    rfootImuPoseProblem.setup(); rfootImuPoseProblem.setOptimizedValuesToInitialValues();
    imuPoseEstimator lthighImuPoseProblem(leftThighImu, "lthigh");
    lthighImuPoseProblem.setImuBiasModelMode(1);
    lthighImuPoseProblem.setup(); lthighImuPoseProblem.setOptimizedValuesToInitialValues();
    imuPoseEstimator lshankImuPoseProblem(leftShankImu, "lshank");
    lshankImuPoseProblem.setImuBiasModelMode(1);
    lshankImuPoseProblem.setup(); lshankImuPoseProblem.setOptimizedValuesToInitialValues();
    imuPoseEstimator lfootImuPoseProblem(leftFootImu, "lfoot");
    lfootImuPoseProblem.setImuBiasModelMode(1);
    lfootImuPoseProblem.setup(); lfootImuPoseProblem.setOptimizedValuesToInitialValues();
    // setup pose estimator
    lowerBodyPoseEstimator lbpe(sacrumImuPoseProblem,rthighImuPoseProblem,rshankImuPoseProblem,rfootImuPoseProblem,lthighImuPoseProblem,lshankImuPoseProblem,lfootImuPoseProblem,"example");
    // for speed, shorten the max number of iterations. this is just a unit test, after all, not a solution test.
    lbpe.m_maxIterations=5;
    lbpe.setup();
    lbpe.fastOptimize();

    return 0;
}
