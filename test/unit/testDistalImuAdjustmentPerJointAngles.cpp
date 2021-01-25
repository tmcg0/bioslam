// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// unit test adjustment of the distal imu orientation based on desired joint angles

#include <testutils.h>
#include <bioutils.h>
#include "lowerBodyPoseEstimator.h"

int main(){
    uint nTests=10000;
    for(uint j=0; j<nTests; j++){
        // () construct random proximal imu orientation and states
        gtsam::Rot3 R_ProxImu_to_N=testutils::randomRot3();
        gtsam::Unit3 rightAxisProx=testutils::randomUnit3();
        gtsam::Vector3 refVec=testutils::randomVector3();
        gtsam::Vector3 proxImuToProxJointCtr=(refVec.cross(rightAxisProx.unitVector())).normalized();
        gtsam::Vector3 proxImuToDistJointCtr=-1.0*proxImuToProxJointCtr;
        gtsam::Rot3 R_ProxSeg_to_N=bioutils::get_R_Segment_to_N(R_ProxImu_to_N,rightAxisProx,proxImuToProxJointCtr,proxImuToDistJointCtr);
        // () construct random distal imu orientation and states
        gtsam::Rot3 R_DistImu_to_N=testutils::randomRot3();
        gtsam::Unit3 rightAxisDist=testutils::randomUnit3();
        gtsam::Vector3 refVecDist=testutils::randomVector3();
        gtsam::Vector3 distImuToProxJointCtr=(refVecDist.cross(rightAxisDist.unitVector())).normalized();
        gtsam::Vector3 distImuToDistJointCtr=-1.0*proxImuToProxJointCtr;
        gtsam::Rot3 R_DistSeg_to_N=bioutils::get_R_Segment_to_N(R_DistImu_to_N,rightAxisDist,distImuToProxJointCtr,distImuToDistJointCtr);
        // () generate random desired angles
        double desiredFlexExAng=testutils::dRand(), desiredIntExtRotAng=testutils::dRand(), desiredAbAdAng=testutils::dRand();
        // () get old angles
        Eigen::RowVector3d jointAnglesOrig=bioutils::consistent3DofJcsAngles(R_ProxSeg_to_N,R_DistSeg_to_N);
        // () adjust for int/ext rot
        gtsam::Rot3 R_B_to_B2=lowerBodyPoseEstimator::getDistalImuOrientationAdjustmentGivenDesiredIntExtRotAngle(R_ProxSeg_to_N,  R_DistImu_to_N, desiredIntExtRotAng,rightAxisDist,distImuToProxJointCtr,distImuToDistJointCtr);
        R_DistImu_to_N=R_DistImu_to_N*R_B_to_B2.inverse(); // apply adjustment: R[B2->N]=R[B->N]*R[B->B2]'
        // () adjust for ab/ad
        R_B_to_B2=lowerBodyPoseEstimator::getDistalImuOrientationAdjustmentGivenDesiredAbAdAngle(R_ProxSeg_to_N,  R_DistImu_to_N, desiredAbAdAng,rightAxisDist,distImuToProxJointCtr,distImuToDistJointCtr);
        R_DistImu_to_N=R_DistImu_to_N*R_B_to_B2.inverse(); // apply adjustment: R[B2->N]=R[B->N]*R[B->B2]'
        // () adjust for flex/ex
        R_B_to_B2=lowerBodyPoseEstimator::getDistalImuOrientationAdjustmentGivenDesiredFlexExAngle(R_ProxSeg_to_N, R_ProxImu_to_N, R_DistImu_to_N, desiredFlexExAng, rightAxisProx, rightAxisDist,distImuToProxJointCtr,distImuToDistJointCtr);
        R_DistImu_to_N=R_DistImu_to_N*R_B_to_B2.inverse(); // apply adjustment: R[B2->N]=R[B->N]*R[B->B2]'
        // () assert that new angles are what was desired
        R_DistSeg_to_N=bioutils::get_R_Segment_to_N(R_DistImu_to_N,rightAxisDist,distImuToProxJointCtr,distImuToDistJointCtr);
        Eigen::RowVector3d jointAnglesNew=bioutils::consistent3DofJcsAngles(R_ProxSeg_to_N,R_DistSeg_to_N);
        if(abs(desiredIntExtRotAng-jointAnglesNew(2))>1.0e-8){
            std::cout<<"desiredIntExtRotAng="<<desiredIntExtRotAng<<", jointAnglesNew(2)="<<jointAnglesNew(2)<<std::endl;
            throw std::runtime_error("did not create desired angle value");
        }
        if(abs(desiredAbAdAng-jointAnglesNew(1))>1.0e-8){
            std::cout<<"desiredAbAdAng="<<desiredAbAdAng<<", jointAnglesNew(1)="<<jointAnglesNew(1)<<std::endl;
            throw std::runtime_error("did not create desired angle value");
        }
        if(abs(desiredFlexExAng-jointAnglesNew(0))>1.0e-8){
            std::cout<<"desiredFlexExAng="<<desiredFlexExAng<<", jointAnglesNew(0)="<<jointAnglesNew(0)<<std::endl;
            throw std::runtime_error("did not create desired angle value");
        }
    }
    return 0;
}