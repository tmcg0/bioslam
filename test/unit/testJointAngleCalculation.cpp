// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// unit test of consistent Jcs definition

#include <bioutils.h>
#include <testutils.h>
#include <mathutils.h>

int testExpectedJointAngles(uint nTests=1000,uint subTests=10, bool constructRandomCoordSys=true, bool setDistImuCoordSysSameAsProx=false);
int testZeroJointAngleCondition(uint nTests=100);

int main(){
    testZeroJointAngleCondition();
    testExpectedJointAngles(1000,10,true,true);
    testExpectedJointAngles(1000,10,true,false);
    testExpectedJointAngles(1000,10,false,true);
    testExpectedJointAngles(1000,10,false,false);
    return 0;
}

int testExpectedJointAngles(uint nTests, uint subTests, bool constructRandomCoordSys, bool setDistImuCoordSysSameAsProx){
    // construct identical proximal and distal segments, then rotate the distal segment by an angle to create an expected joint angle. test that the angle calc method returns this.
    gtsam::Rot3 R_proxImu_to_N, R_ProxSeg_to_N, R_distImu_to_N, R_DistSeg_to_N;
    gtsam::Vector3 distImuProx, distImuDist, proxImuProx, proxImuDist;
    gtsam::Unit3 proxImuRight, distImuRight;
    for(uint k=0; k<nTests; k++) {
        if(constructRandomCoordSys) {
            // construct random coordinate systems
            // --- construct proximal segment coordinate system ---
            R_proxImu_to_N = testutils::randomRot3();
            proxImuRight = testutils::randomUnit3(); // I
            gtsam::Unit3 refVecProxImu = testutils::randomUnit3();
            proxImuProx = refVecProxImu.unitVector().cross(proxImuRight.unitVector()); // K
            proxImuDist = -proxImuProx;
            R_ProxSeg_to_N = bioutils::get_R_Segment_to_N(R_proxImu_to_N, proxImuRight, proxImuProx, proxImuDist);
            // --- construct distal imu coordinate system  ---
            if(setDistImuCoordSysSameAsProx){
                R_distImu_to_N = R_proxImu_to_N;
            }else{ R_distImu_to_N =  testutils::randomRot3(); }
            distImuRight = testutils::randomUnit3(); // i
            gtsam::Unit3 refVecDistImu = testutils::randomUnit3();
            distImuProx = refVecDistImu.unitVector().cross(distImuRight.unitVector()); // k
            distImuDist = -distImuProx;
            R_DistSeg_to_N = bioutils::get_R_Segment_to_N(R_distImu_to_N, distImuRight, distImuProx, distImuDist);
        }else{
            // construct trivial coordinate systems
            // --- construct proximal segment coordinate system ---
            R_proxImu_to_N = testutils::randomRot3();
            proxImuRight = gtsam::Unit3(0.0,0.0,1.0); // I
            proxImuProx = gtsam::Point3(-1.0,0.0,0.0); // K
            proxImuDist = -proxImuProx;
            R_ProxSeg_to_N = bioutils::get_R_Segment_to_N(R_proxImu_to_N, proxImuRight, proxImuProx, proxImuDist);
            // --- construct distal imu coordinate system  ---
            if(setDistImuCoordSysSameAsProx){
                R_distImu_to_N = R_proxImu_to_N;
            }else{ R_distImu_to_N =  testutils::randomRot3(); }
            distImuRight = gtsam::Unit3(0.0,0.0,1.0); // i
            distImuProx = gtsam::Point3(-1.0,0.0,0.0); // k
            distImuDist = -distImuProx;
            R_DistSeg_to_N = bioutils::get_R_Segment_to_N(R_distImu_to_N, distImuRight, distImuProx, distImuDist);
        }
        // get original joint angles
        Eigen::RowVector3d jointAnglesOrig = bioutils::consistent3DofJcsAngles(R_ProxSeg_to_N, R_DistSeg_to_N);
        // () -------------- test expected flex/ex response ----------------------
        // randomly create flex/ex rotations to apply to distal segment and test that expected angle is found
        for(uint j=0; j<subTests; j++) {
            double deltaFlexEx=testutils::dRand(-0.4,0.4); // delta alpha
            gtsam::Unit3 flexExAxisProxInDistalFrame=R_distImu_to_N.inverse()*R_proxImu_to_N*proxImuRight; // vA[B]= R_B_to_N'*R_A_to_N*vA[A]
            gtsam::Rot3 distImuDeltaRot=gtsam::Rot3::AxisAngle(flexExAxisProxInDistalFrame,-deltaFlexEx); // R[B->B2]. remember distal frame has to be rotated by negative delta_angle.
            gtsam::Rot3 R_DistImu_to_N_new=R_distImu_to_N*distImuDeltaRot.inverse(); // R[B2->N] = (R[B->N]*R[B->B2]')
            gtsam::Rot3 R_DistSeg_to_N_new=bioutils::get_R_Segment_to_N(R_DistImu_to_N_new,distImuRight,distImuProx,distImuDist);
            Eigen::RowVector3d jointAnglesNew = bioutils::consistent3DofJcsAngles(R_ProxSeg_to_N, R_DistSeg_to_N_new); // get new joint angles
            double expectedFlexExAngle=mathutils::wrapToPi(jointAnglesOrig(0) + deltaFlexEx);  // expected: new flex ex angle = old flexex ang + randFlexExSpin
            if( abs(expectedFlexExAngle-jointAnglesNew(0))>1.0e-10 ){
                std::cout<<"---- error debugging data ----"<<std::endl;
                std::cout <<"orig. joint angles: " << jointAnglesOrig << std::endl;
                std::cout<<"flexExAxis[proxImuFrame] = "<<proxImuRight.unitVector().transpose()<<", flexExAxis[distImuFrame] = "<<flexExAxisProxInDistalFrame.unitVector().transpose()<<std::endl;
                std::cout<<"new joint angles = "<<jointAnglesNew<<std::endl;
                throw std::runtime_error("unexpected angle (error = "+std::to_string(abs(jointAnglesOrig(0)+deltaFlexEx-jointAnglesNew(0)))+")"  );
            }
            if(abs(jointAnglesOrig(1)-jointAnglesNew(1))>1.0e-10){throw std::runtime_error("other angles should not have changed");} // make sure other joint angles haven't changed
            if(abs(jointAnglesOrig(2)-jointAnglesNew(2))>1.0e-10){throw std::runtime_error("other angles should not have changed");} // make sure other joint angles haven't changed
        }
        // () -------------- test expected int/ext rot response ----------------------
        // randomly create int/ext rotations to apply to distal segment and test that expected angle is found
        for(uint j=0; j<subTests; j++) {
            double deltaIntExtRot=testutils::dRand(-0.4,0.4); // delta gamma
            gtsam::Rot3 distImuDeltaRot=gtsam::Rot3::AxisAngle(distImuProx.normalized(),-deltaIntExtRot); // R[B->B2]. remember distal frame has to be rotated by negative angle. remember that rotation is about distal imu prox axis
            gtsam::Rot3 R_DistImu_to_N_new=R_distImu_to_N*distImuDeltaRot.inverse(); // R[B2->N] = (R[B->N]*R[B->B2]')
            gtsam::Rot3 R_DistSeg_to_N_new=bioutils::get_R_Segment_to_N(R_DistImu_to_N_new,distImuRight,distImuProx,distImuDist);
            Eigen::RowVector3d jointAnglesNew = bioutils::consistent3DofJcsAngles(R_ProxSeg_to_N, R_DistSeg_to_N_new); // get new joint angles
            double expectedIntExtRotAngle=mathutils::wrapToPi(jointAnglesOrig(2) + deltaIntExtRot);  // expected: new angle = old ang + randSpin
            if( abs(expectedIntExtRotAngle-jointAnglesNew(2))>1.0e-10 ){
                std::cout<<"---- error debugging data ----"<<std::endl;
                std::cout <<"orig. joint angles: " << jointAnglesOrig << std::endl;
                std::cout<<"rotation axis k (dist. imu prox vec) = "<<distImuProx.normalized().transpose()<<std::endl;
                std::cout<<"new joint angles = "<<jointAnglesNew<<std::endl;
                throw std::runtime_error("unexpected angle (error = "+std::to_string(expectedIntExtRotAngle-jointAnglesNew(2))+")"  );
            }
            if(abs(jointAnglesOrig(0)-jointAnglesNew(0))>1.0e-10){throw std::runtime_error("other angles should not have changed");} // make sure other joint angles haven't changed
            if(abs(jointAnglesOrig(1)-jointAnglesNew(1))>1.0e-10){throw std::runtime_error("other angles should not have changed");} // make sure other joint angles haven't changed
        }
        // () -------------- test expected ab/ad rot response ----------------------
        // randomly create ab/ad rotations to apply to distal segment and test that expected angle is found
        for(uint j=0; j<subTests; j++) {
            // there is an issue when trying to create an expected ab/ad angle with abs. value greater than pi/2. intuition:
            //     computed abad angle is actually an angle beta-pi/2. therefore beta = abad + pi/2.
            //     if abad>pi/2 => beta>pi || if abad<-pi/2 => beta<0
            //     BUT problem: neither of these situations are valid because beta is always an unsigned angle on [0,pi]
            //     *** fix: we need to make a check--we can never "request" an ab/ad angle with greater abs value than pi/2 ***
            double deltaAbAd=testutils::dRand(-0.4,0.4); // delta beta
            gtsam::Unit3 flexExAxisProxInDistalFrame=R_distImu_to_N.inverse()*R_proxImu_to_N*proxImuRight; // vA[B]= R_B_to_N'*R_A_to_N*vA[A]
            gtsam::Vector3 spinAxis=(distImuProx.cross(flexExAxisProxInDistalFrame.unitVector())).normalized(); // spin axis is cross(k,I') where k is distal IMU prox vec, and I' is proximal IMU flex/ex axis rotated into distal frame
            gtsam::Rot3 distImuDeltaRot=gtsam::Rot3::AxisAngle(spinAxis,deltaAbAd); // R[B->B2]
            gtsam::Rot3 R_DistImu_to_N_new=R_distImu_to_N*distImuDeltaRot.inverse(); // R[B2->N] = R[B->N]*R[B->B2]'
            gtsam::Rot3 R_DistSeg_to_N_new=bioutils::get_R_Segment_to_N(R_DistImu_to_N_new,distImuRight,distImuProx,distImuDist);
            Eigen::RowVector3d jointAnglesNew = bioutils::consistent3DofJcsAngles(R_ProxSeg_to_N, R_DistSeg_to_N_new); // get new joint angles
            double expectedAbAdAngle=mathutils::wrapToPi(jointAnglesOrig(1) + deltaAbAd);  // expected: new angle = old ang + randSpin
            if(! (abs(expectedAbAdAngle)>M_PI/2.0)){ // remember: we can't ever "request" (i.e., try to create) an expected ab/ad with abs value greater than pi/2
                if( abs(expectedAbAdAngle-jointAnglesNew(1))>1.0e-10 ){
                    std::cout<<"---- error debugging data ----"<<std::endl;
                    std::cout <<"orig. joint angles: [" << jointAnglesOrig << "], new joint angles = ["<<jointAnglesNew<<"]"<<std::endl;
                    std::cout<<"randAbAdSpin = "<<deltaAbAd<<", therefore expected new ab/ad angle = origAbAd + randAbAdSpin = "<<expectedAbAdAngle<<std::endl;
                    std::cout<<"flexExAxis[proxImu] (I)= ["<<proxImuRight.unitVector().transpose()<<"], flexExAxis[distImu]= ["<<flexExAxisProxInDistalFrame.unitVector().transpose()<<"]"<<std::endl;
                    std::cout<<"    distImuProxVec (k)=["<<distImuProx.transpose()<<"] ==> spinAxis=cross(distImuProxVec,flexExAxis[distImu])= ["<<spinAxis.transpose()<<"] (norm="<<spinAxis.norm()<<")"<<std::endl;
                    throw std::runtime_error("unexpected angle (error = "+std::to_string(expectedAbAdAngle-jointAnglesNew(1))+")"  );
                }
                if(abs(jointAnglesOrig(0)-jointAnglesNew(0))>1.0e-10){throw std::runtime_error("other angles should not have changed");} // make sure other joint angles haven't changed
                if(abs(jointAnglesOrig(2)-jointAnglesNew(2))>1.0e-10){throw std::runtime_error("other angles should not have changed");} // make sure other joint angles haven't changed
            }
        }
    }
    return 0;
}

int testZeroJointAngleCondition(uint nTests){
    // if R_DistalSeg_to_N and R_ProxSeg_to_N are identical, then joint angles should all be zero
    for(uint k=0; k<nTests; k++){
        gtsam::Rot3 R_proxSeg_to_N=testutils::randomRot3();
        gtsam::Rot3 R_distSeg_to_N=R_proxSeg_to_N;
        Eigen::RowVector3d jointAngles = bioutils::consistent3DofJcsAngles(R_proxSeg_to_N, R_distSeg_to_N);
        //std::cout<<"expected zero joint angles = "<<jointAngles<<std::endl;
        if(abs(jointAngles(0))>1.0e-10){throw std::runtime_error("expected zero joint angle but joint angle was "+std::to_string(jointAngles(0)));}
        if(abs(jointAngles(1))>1.0e-10){throw std::runtime_error("expected zero joint angle but joint angle was "+std::to_string(jointAngles(1)));}
        if(abs(jointAngles(2))>1.0e-10){throw std::runtime_error("expected zero joint angle but joint angle was "+std::to_string(jointAngles(2)));}
    }
    return 0;
}