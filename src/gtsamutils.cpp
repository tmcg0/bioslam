// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <mathutils.h>
#include "gtsamutils.h"
#include <fstream>
#include <iomanip>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <factors/ConstrainedJointCenterPositionFactor.h>
#include <factors/ConstrainedJointCenterVelocityFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <factors/AngularVelocityFactor.h>
#include <factors/HingeJointFactors.h>
#include <factors/SegmentLengthMagnitudeFactor.h>
#include <factors/SegmentLengthDiscrepancyFactor.h>
#include <factors/AngleBetweenAxisAndSegmentFactor.h>
#include <factors/MagPose3Factor.h>
#include <factors/Pose3Priors.h>
#include <factors/Point3Priors.h>

void print(const gtsam::Matrix& A, const std::string &s, std::ostream& stream);

namespace gtsamutils{

    Eigen::MatrixXd Point3VectorToEigenMatrix(const std::vector<gtsam::Point3>& p){
        // vector<Rot3> => Nx4 Eigen::MatrixXd
        Eigen::MatrixXd M(p.size(),3);
        for(uint i=0; i<p.size(); i++){
            gtsam::Vector3 pos=p[i];
            M(i,0)=pos[0]; M(i,1)=pos[1]; M(i,2)=pos[2];
        }
        return M;
    }

    Eigen::MatrixXd Vector3VectorToEigenMatrix(const std::vector<gtsam::Vector3>& v){
        // vector<Rot3> => Nx4 Eigen::MatrixXd
        Eigen::MatrixXd M(v.size(),3);
        for(uint i=0; i<v.size(); i++){
            M(i,0)=v[i][0]; M(i,1)=v[i][1]; M(i,2)=v[i][2];
        }
        return M;
    }

    Eigen::MatrixXd vectorRot3ToFlattedEigenMatrixXd(const std::vector<gtsam::Rot3>& R){
        // make a Nx9 flatted Eigen::Matrix
        Eigen::MatrixXd M(R.size(),9);
        for(uint i=0; i<R.size(); i++){
            gtsam::Matrix33 m=R[i].matrix();
            M(i,0)=m(0,0); M(i,1)=m(0,1); M(i,2)=m(0,2); M(i,3)=m(1,0); M(i,4)=m(1,1); M(i,5)=m(1,2); M(i,6)=m(2,0); M(i,7)=m(2,1); M(i,8)=m(2,2);
        }
        return M;
    }

    void printErrorsInGraphByFactorType(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& vals){
        // this is less fun in C++ than MATLAB. You'll need to automatically know every possible in your graph a priori.
        // for each factor, get a count of how many there are in the graph. If count>0, then find error and print it.
        double totalGraphError=graph.error(vals), startTic=clock();
        std::cout<<"total error in graph = "<<totalGraphError<<std::endl;
        // now for each type of factor, create count, total error, and print results if some are found
        gtsamutils::printGraphErrorDueToFactorType<gtsam::PriorFactor<gtsam::Unit3>>(graph,vals,"    gtsam::PriorFactor<Unit3>:");
        gtsamutils::printGraphErrorDueToFactorType<gtsam::PriorFactor<gtsam::Pose3>>(graph,vals,"    gtsam::PriorFactor<Pose3>:");
        gtsamutils::printGraphErrorDueToFactorType<gtsam::PriorFactor<gtsam::Rot3>>(graph,vals,"    gtsam::PriorFactor<Rot3>:");
        gtsamutils::printGraphErrorDueToFactorType<gtsam::PriorFactor<gtsam::Point3>>(graph,vals,"    gtsam::PriorFactor<Point3>:");
        gtsamutils::printGraphErrorDueToFactorType<gtsam::PriorFactor<gtsam::Vector3>>(graph,vals,"    gtsam::PriorFactor<Vector3>:");
        gtsamutils::printGraphErrorDueToFactorType<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(graph,vals,"    gtsam::PriorFactor<imuBias::ConstantBias>:");
        gtsamutils::printGraphErrorDueToFactorType<gtsam::CombinedImuFactor>(graph,vals,"    gtsam::CombinedImuFactor:");
        gtsamutils::printGraphErrorDueToFactorType<gtsam::ImuFactor>(graph,vals,"    gtsam::ImuFactor:");
        gtsamutils::printGraphErrorDueToFactorType<bioslam::Pose3TranslationPrior>(graph,vals,"    bioslam::Pose3TranslationPrior:");
        gtsamutils::printGraphErrorDueToFactorType<bioslam::Pose3CompassPrior>(graph,vals,"    bioslam::Pose3CompassPrior:");
        gtsamutils::printGraphErrorDueToFactorType<bioslam::AngularVelocityFactor>(graph,vals,"    bioslam::AngularVelocityFactor:");
        gtsamutils::printGraphErrorDueToFactorType<bioslam::ConstrainedJointCenterPositionFactor>(graph, vals, "    bioslam::ConstrainedJointCenterPositionFactor:");
        gtsamutils::printGraphErrorDueToFactorType<bioslam::ConstrainedJointCenterNormPositionFactor>(graph, vals, "    bioslam::ConstrainedJointCenterNormPositionFactor:");
        gtsamutils::printGraphErrorDueToFactorType<bioslam::HingeJointConstraintVecErrEstAngVel>(graph, vals, "    bioslam::HingeJointConstraintVecErrEstAngVel:");
        gtsamutils::printGraphErrorDueToFactorType<bioslam::HingeJointConstraintNormErrEstAngVel>(graph, vals, "    bioslam::HingeJointConstraintNormErrEstAngVel:");
        gtsamutils::printGraphErrorDueToFactorType<bioslam::SegmentLengthMagnitudeFactor>(graph,vals,"    bioslam::SegmentLengthMagnitudeFactor:");
        gtsamutils::printGraphErrorDueToFactorType<bioslam::SegmentLengthMaxMagnitudeFactor>(graph,vals,"    bioslam::SegmentLengthMaxMagnitudeFactor:");
        gtsamutils::printGraphErrorDueToFactorType<bioslam::SegmentLengthMinMagnitudeFactor>(graph,vals,"    bioslam::SegmentLengthMinMagnitudeFactor:");
        gtsamutils::printGraphErrorDueToFactorType<bioslam::ConstrainedJointCenterVelocityFactor>(graph,vals,"    bioslam::ConstrainedJointCenterVelocityFactor:");
        gtsamutils::printGraphErrorDueToFactorType<bioslam::ConstrainedJointCenterNormVelocityFactor>(graph, vals, "    bioslam::ConstrainedJointCenterNormVelocityFactor:");
        gtsamutils::printGraphErrorDueToFactorType<bioslam::SegmentLengthDiscrepancyFactor>(graph,vals,"    bioslam::SegmentLengthDiscrepancyFactor:");
        gtsamutils::printGraphErrorDueToFactorType<bioslam::AngleBetweenAxisAndSegmentFactor>(graph,vals,"    bioslam::AngleBetweenAxisAndSegmentFactor:");
        gtsamutils::printGraphErrorDueToFactorType<bioslam::MinAngleBetweenAxisAndSegmentFactor>(graph,vals,"    bioslam::MinAngleBetweenAxisAndSegmentFactor:");
        gtsamutils::printGraphErrorDueToFactorType<bioslam::MaxAngleBetweenAxisAndSegmentFactor>(graph,vals,"    bioslam::MaxAngleBetweenAxisAndSegmentFactor:");
        gtsamutils::printGraphErrorDueToFactorType<bioslam::Point3MagnitudeDifferenceFactor>(graph,vals,"    bioslam::Point3MagnitudeDifferenceFactor:");
        gtsamutils::printGraphErrorDueToFactorType<bioslam::MagPose3Factor>(graph,vals,"    bioslam::MagPose3Factor:");
        std::cout<<"        graph error printing complete. ("<<(clock()-startTic)/CLOCKS_PER_SEC<<" seconds)"<<std::endl;
    }

    std::vector<gtsam::Rot3> imuOrientation(const imu& myImu){
        std::vector<std::vector<double>> q=myImu.quaternion();
        std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> qAPDM=mathutils::VectorVectorDoubleToVectorEigenVector(q);
        std::vector<gtsam::Rot3> orientation_Rot3=mathutils::QuaternionVectorToRot3Vector(qAPDM);
        return orientation_Rot3;
    }

    std::vector<gtsam::Rot3> Pose3VectorToRot3Vector(const std::vector<gtsam::Pose3>& poses){
        std::vector<gtsam::Rot3> rots(poses.size());
        for(uint k=0; k<poses.size();k++){
            rots[k]=poses[k].rotation();
        }
        return rots;
    }

    std::vector<gtsam::Point3> Pose3VectorToPoint3Vector(const std::vector<gtsam::Pose3>& poses){
        std::vector<gtsam::Point3> pos(poses.size());
        for(uint k=0; k<poses.size();k++){
            pos[k]=poses[k].translation();
        }
        return pos;
    }

    std::vector<double> vectorSetMagnitudes(const std::vector<Eigen::Vector3d>& v){
        std::vector<double> mags(v.size());
        for(uint i=0;i<v.size();i++){
            mags[i]=v[i].norm();
        }
        return mags;
    }

    gtsam::Vector3 accel_Vector3(const imu& myImu, const int& idx){
        gtsam::Vector3 x;
        x[0]=myImu.ax[idx]; x[1]=myImu.ay[idx]; x[2]=myImu.az[idx];
        return x;
    }

    gtsam::Vector3 mags_Vector3(const imu& myImu, const int& idx){
        gtsam::Vector3 x;
        x[0]=myImu.mx[idx]; x[1]=myImu.my[idx]; x[2]=myImu.mz[idx];
        return x;
    }

    gtsam::Vector3 gyros_Vector3(const imu& myImu, const int& idx){
        gtsam::Vector3 x;
        x[0]=myImu.gx[idx]; x[1]=myImu.gy[idx]; x[2]=myImu.gz[idx];
        return x;
    }

    Eigen::MatrixXd gyroMatrix(const imu& myImu){
        Eigen::MatrixXd gyros(myImu.length(),3);
        for(uint k=0; k<myImu.length();k++){
            gyros(k,0)=myImu.gx[k]; gyros(k,1)=myImu.gy[k]; gyros(k,2)=myImu.gz[k];
        }
        return gyros;
    }
    Eigen::MatrixXd accelMatrix(const imu& myImu){
        Eigen::MatrixXd accels(myImu.length(),3);
        for(uint k=0; k<myImu.length();k++){
            accels(k,0)=myImu.ax[k]; accels(k,1)=myImu.ay[k]; accels(k,2)=myImu.az[k];
        }
        return accels;
    }

    double median(std::vector<double> len){
        assert(!len.empty());
        if (len.size() % 2 == 0) {
            const auto median_it1 = len.begin() + len.size() / 2 - 1;
            const auto median_it2 = len.begin() + len.size() / 2;

            std::nth_element(len.begin(), median_it1 , len.end());
            const auto e1 = *median_it1;

            std::nth_element(len.begin(), median_it2 , len.end());
            const auto e2 = *median_it2;

            return (e1 + e2) / 2;

        } else {
            const auto median_it = len.begin() + len.size() / 2;
            std::nth_element(len.begin(), median_it , len.end());
            return *median_it;
        }
    }

    uint nearestIdxToVal(std::vector<double> v, double val){
        // this is gonna be ugly. copies entire vector and brute force searches for index nearest to value val.
        uint nearestIdx;
        double dist=9.0e9;
        for (uint k=0; k<v.size(); k++){
            if(abs(v[k]-val)<dist){ // update nearest index
                nearestIdx=k;
                dist=abs(v[k]-val); // update smallest found distance
            }
        }
        return nearestIdx;
    }

    std::vector<double> vectorizePoint3x(std::vector<gtsam::Point3> p){
        std::vector<double> x(p.size());
        for(uint i=0; i<p.size(); i++){
            x[i]=p[i].x();
        }
        return x;
    }
    std::vector<double> vectorizePoint3y(std::vector<gtsam::Point3> p){
        std::vector<double> y(p.size());
        for(uint i=0; i<p.size(); i++){
            y[i]=p[i].y();
        }
        return y;
    }
    std::vector<double> vectorizePoint3z(std::vector<gtsam::Point3> p){
        std::vector<double> z(p.size());
        for(uint i=0; i<p.size(); i++){
            z[i]=p[i].z();
        }
        return z;
    }
    std::vector<double> vectorizeVector3X(std::vector<gtsam::Vector3> v){
        std::vector<double> a(v.size());
        for(uint i=0; i<v.size(); i++){
            a[i]=v[i](0);
        }
        return a;
    }
    std::vector<double> vectorizeVector3Y(std::vector<gtsam::Vector3> v){
        std::vector<double> a(v.size());
        for(uint i=0; i<v.size(); i++){
            a[i]=v[i](1);
        }
        return a;
    }
    std::vector<double> vectorizeVector3Z(std::vector<gtsam::Vector3> v){
        std::vector<double> a(v.size());
        for(uint i=0; i<v.size(); i++){
            a[i]=v[i](2);
        }
        return a;
    }
    std::vector<double> vectorizeQuaternionS(std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> q){
        std::vector<double> a(q.size());
        for(uint i=0; i<q.size(); i++){
            a[i]=q[i](0);
        }
        return a;
    }
    std::vector<double> vectorizeQuaternionX(std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> q){
        std::vector<double> a(q.size());
        for(uint i=0; i<q.size(); i++){
            a[i]=q[i](1);
        }
        return a;
    }
    std::vector<double> vectorizeQuaternionY(std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> q){
        std::vector<double> a(q.size());
        for(uint i=0; i<q.size(); i++){
            a[i]=q[i](2);
        }
        return a;
    }
    std::vector<double> vectorizeQuaternionZ(std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> q){
        std::vector<double> a(q.size());
        for(uint i=0; i<q.size(); i++){
            a[i]=q[i](3);
        }
        return a;
    }
    void saveMatrixToFile(const gtsam::Matrix& A, const std::string &s, const std::string& filename) {
        std::fstream stream(filename.c_str(), std::fstream::out | std::fstream::app);
        print(A, s + "=", stream);
        stream.close();
    }
    void writeEigenMatrixToCsvFile(const std::string& name, const Eigen::MatrixXd& matrix, const Eigen::IOFormat& CSVFormat){
        std::ofstream file(name.c_str());
        file << matrix.format(CSVFormat);
        file.close();
    }
    Eigen::MatrixXd vectorRot3ToYprMatrix(const std::vector<gtsam::Rot3>& R){
        Eigen::MatrixXd yprMatrix(R.size(),3);
        for(uint k=0; k<R.size(); k++){
            gtsam::Vector3 ypr=R[k].ypr();
            yprMatrix(k,0)=ypr(0); yprMatrix(k,1)=ypr(1); yprMatrix(k,2)=ypr(2);
        }
        return yprMatrix;
    }


}

// helper functions

void print(const gtsam::Matrix& A, const std::string &s, std::ostream& stream) {
    size_t m = A.rows(), n = A.cols();

    // print out all elements
    stream << s << "[\n";
    for( size_t i = 0 ; i < m ; i++) {
        for( size_t j = 0 ; j < n ; j++) {
            double aij = A(i,j);
            if(aij != 0.0)
                stream << std::setw(12) << std::setprecision(9) << aij << ",\t";
            else
                stream << "         0.0,\t";
        }
        stream << std::endl;
    }
    stream << "];" << std::endl;
}