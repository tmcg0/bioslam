// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// A set of utilities to aid in testing

#pragma once

#include "imu/imu.h"
#include <random>
#include <gtsam/geometry/Pose3.h>
#include "gtsam/base/Vector.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsamutils.h"
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <iostream>
#include <cmath>

namespace testutils
{
    std::string getTestDataDir();
    std::string getBioslamRootDir();
    std::string getTestDataFile(const std::string& filename);
    Eigen::Vector4d randomQuat();
    Eigen::Vector4d randomDiffQuat();
    gtsam::Point3 randomPoint3();
    gtsam::Pose3 randomPose3();
    gtsam::Rot3 randomRot3();
    gtsam::Unit3 randomUnit3();
    gtsam::Vector3 randomVector3();
    double dRand(double lower_bound=-1.0, double upper_bound=1.0); // random double
    void runtime_assert(bool condition); // implementation of assert() that throws a runtime error if failed. not optimized out by release builds.
}