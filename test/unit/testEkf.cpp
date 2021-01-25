// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// testing of EKF implementations against APDM manufacturer UKF for the Opal v2

#include "imuPoseEstimator.h"
#include "VarStrToCharMap.h"
#include "testutils.h"
#include "mathutils.h"

int main(){
    // --- settings --- //
    double tolDeg=5.0;
    // ---------------- //
    // setup stuff
    VarStrToCharMap::clear();
    //std::string dataFileToUse=testutils::getTestDataFile("20170719-113432-Y13_TUG_13.h5");
    std::string dataFileToUse=testutils::getTestDataFile("20170411-154746-Y1_TUG_6.h5");
    std::map<std::string,imu> ImuMap=imu::getImuMapFromDataFile(dataFileToUse);
    // imu::printLabelsInFile(dataFileToUse);
    std::string imuLabel="Right Thigh";
    imu testImu=ImuMap[imuLabel];
    testutils::runtime_assert(testImu.length()>0);
    std::cout<<"test imu contains "<<testImu.length()<<" measurements ("<<testImu.relTimeSec[testImu.length()-1]<<" seconds)"<<std::endl;
    imuPoseEstimator poseProblem(testImu, "unitTest");
    // get APDM orientatioon
    std::vector<gtsam::Rot3> rAPDM=gtsamutils::imuOrientation(testImu); // get APDM orientation as a gtsam::Rot3 set
    // remember that APDM orientation and my EKF is [N->B]
    // get orientation according to EKF
    Eigen::MatrixXd q_B_to_N=mathutils::simpleImuOrientationForwardBackwardEkfWrapper(gtsamutils::gyroMatrix(poseProblem.m_imu), gtsamutils::accelMatrix(poseProblem.m_imu), poseProblem.m_imu.getDeltaT(), 4);
    std::vector<gtsam::Rot3> rEKF(q_B_to_N.rows()), rRel(q_B_to_N.rows());
    std::vector<double> yawSqErr(rAPDM.size()), pitchSqErr(rAPDM.size()), rollSqErr(rAPDM.size());
    for(uint k=0;k<q_B_to_N.rows();k++){
        // convert into a vector of rot3
        rEKF[k]=gtsam::Rot3(q_B_to_N(k,0),q_B_to_N(k,1),q_B_to_N(k,2),q_B_to_N(k,3));
        rRel[k]=rAPDM[k]*rEKF[k].inverse(); // R[EKF->APDM]=R[N->APDM]*R[N->EKF]'
        //std::cout<<"relative orientation ["<<k<<"] rpy()="<<rRel[k].rpy().transpose()<<std::endl;
        gtsam::Vector3 rpyErr=rRel[k].rpy();
        yawSqErr[k]=pow(rpyErr[2],2);
        pitchSqErr[k]=pow(rpyErr[1],2);
        rollSqErr[k]=pow(rpyErr[0],2);
    }
    // now compute RMSEs
    double yawRMSE = sqrt((std::accumulate(yawSqErr.begin(), yawSqErr.end(), 0.0))/yawSqErr.size());
    double pitchRMSE = sqrt((std::accumulate(pitchSqErr.begin(), pitchSqErr.end(), 0.0))/pitchSqErr.size());
    double rollRMSE = sqrt((std::accumulate(rollSqErr.begin(), rollSqErr.end(), 0.0))/rollSqErr.size());
    std::cout<<"[roll, pitch, yaw] RMSE = ["<<rollRMSE<<", "<<pitchRMSE<<", "<<yawRMSE<<"]"<<std::endl;
    // assess if these errors are good enough
    if(pitchRMSE*180.0/M_PI>tolDeg){
        std::cerr<<"error: pitch rmse ("<<pitchRMSE*180.0/M_PI<<" deg) greater than tolerance ("<<tolDeg<<" deg)"<<std::endl;
        return 1;
    }
    if(rollRMSE*180.0/M_PI>tolDeg){
        std::cerr<<"error: roll rmse ("<<rollRMSE*180.0/M_PI<<" deg) greater than tolerance ("<<tolDeg<<" deg)"<<std::endl;
        return 1;
    }
    return 0;
}

