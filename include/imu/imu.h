// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#pragma once

#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>
#include <gtsam/base/Vector.h>
#include <highfive/H5DataSet.hpp>


class imu {
    public:
        // constructors
        imu()=default; // empty constructor
        imu(const std::string& filePath, const std::string& labelName); // construct a single imu from a label name in .h5 file
        // properties
        std::vector<double> gx, gy, gz, ax, ay, az, mx, my, mz, qs, qx, qy, qz; // IMU sensor data arrays
        std::vector<double> relTimeSec; // time of measurements, relative to zero in seconds
        std::vector<double> unixTimeUtc; // unix time stamp in seconds, utc (seconds since Jan 1, 1970 UTC)
        uint id; // unique id integer. in apdm sensors that looks like XI-000###
        std::vector<std::vector<double>> quaternion() const; // retrieve quaternion as vector<vector<double>>
        std::string label; // a label to uniquely id an imu in a set
        // other methods
        static std::map<std::string,imu> getImuMapFromDataFile(const std::string& filestr);
        void verifyDataPackaging(bool verbose=false);
        unsigned long length() const;
        static void printLabelsInFile(const std::string& datafilestr);
        gtsam::Vector3 gyroVec(const uint idx) const;
        gtsam::Vector3 accelVec(const uint idx) const;
        gtsam::Vector3 magVec(const uint idx) const;
        Eigen::MatrixXd gyroMat() const; // return Nx3 matrix of all gyro values
        Eigen::MatrixXd accelMat() const; // return Nx3 matrix of all accel values
        double getDeltaT() const;
        imu cutImuByIdx(uint startIdx, uint stopIdx);
};


// slice a vector of data into indexes [m,n]
template<typename T>
std::vector<T> slice(std::vector<T> const &v, uint m, uint n){
    auto first = v.cbegin() + m;
    auto last = v.cbegin() + n;
    std::vector<T> vec(first, last);
    return vec;
}

// read data of template type from h5 dataset into vector of that type
template<typename T>
std::vector<T> readDataFromDataset(const HighFive::DataSet& ds){
    std::vector<T> datavec;
    ds.read(datavec);
    return datavec;
}

struct imudata{
    // this is a simple struct to hold pure imu data for a *single* imu!
    std::vector<double> ax, ay, az, gx, gy, gz, mx, my, mz, relTimeSec, pressure, temperature;
    std::vector<double> qs, qx, qy, qz;
    std::vector<double> unixTimeUtcMicrosec;
    std::string label;
    int id;
};
