// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include "imu/imu.h"
#include "testutils.h"
#include <iostream>

int main(){
    imu imu1=imu(testutils::getTestDataFile("20170411-154746-Y1_TUG_6.h5"), "Sacrum");
    imu1.verifyDataPackaging();
    // print some data out
    std::cout<<"gyros[0]=["<<imu1.gyroVec(0)<<"]"<<std::endl;
    return 0;
}