// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// unit test rotation of an IMU state [pose,velocity,bias] about an axis angle

#include "mathutils.h"
#include "testutils.h"

bool test1();

int main(){
    test1();
    return 0;
}

bool test1(){
    // constructs a unit test that starts with zero rotation but position at [1 0 0]. Then rotates 135 degrees about +z axis in nav frame. should bring position to halfway between +y and -x: []
    // initial orientation is random--because it shouldn't matter.
    gtsam::Pose3 p(testutils::randomRot3(),gtsam::Point3(1.0,0.0,0.0)), pnew=p;
    // construct trivial rotation params
    gtsam::Point3 rotPointNav(0.0,0.0,testutils::dRand()); // should be able to have random Z component since this is a 2d problem
    gtsam::Vector3 rotAxisNav(0.0,0.0,1.0);
    double rotAngle=135.0*M_PI/180.0;
    // debug print statement
    p.print("p0= ");
    // perform rotation
    mathutils::rotateImuPoseAboutPointAxisAngle(pnew,rotPointNav,rotAxisNav,rotAngle);
    // check
    pnew.print("p1= ");
    if(!gtsam::assert_equal(pnew.translation(),gtsam::Point3(-1.0*sqrt(2.0)/2.0,sqrt(2.0)/2.0,0.0))){
        throw std::runtime_error("failed to rotate pose about point axis angle!");
    }
    return true;
}