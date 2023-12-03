// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include "mathutils.h"
#include "gtsamutils.h"
#include <numeric>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <vector>

namespace mathutils
{
    std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> quatUnwrap(std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> quatVector) {
        // perform quaternion unwrapping. gtsam sometimes in the conversion to quaternions will switch sign of the quat.
        // unwrap to the manifold space of the first quaternion in the vector
        for (size_t i = 1; i < quatVector.size(); i++) {
            Eigen::Vector3d previousQuatAxis(quatVector[i - 1][1], quatVector[i - 1][2], quatVector[i - 1][3]);
            Eigen::Vector3d currentQuatAxis(quatVector[i][1], quatVector[i][2], quatVector[i][3]);
            double angBetween = unsignedAngle(previousQuatAxis, currentQuatAxis);
            if (angBetween > M_PI / 2) { // then flip the quaternion
                quatVector[i][0] = -1.0*quatVector[i][0]; quatVector[i][1] = -1.0*quatVector[i][1]; quatVector[i][2] = -1.0*quatVector[i][2]; quatVector[i][3] = -1.0*quatVector[i][3];
            }
        }
        return quatVector;
    }
    void printQuatVector(const std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>>& quatVector){
        int idx=0;
        for (auto q : quatVector){
            std::cout<<"q["<<idx<<"]: ["<<q[0]<<" "<<q[1]<<" "<<q[2]<<" "<<q[3]<<"]"<<std::endl;
            idx++;
        }
    }
    void printQuatVectorToFile(const std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>>& quatVector, std::string filepath){
        std::ofstream myfile;
        myfile.open(filepath);
        int idx=0;
        for (auto q : quatVector) {
            myfile<<"q["<<idx<<"]: ["<<q[0]<<" "<<q[1]<<" "<<q[2]<<" "<<q[3]<<"]"<<std::endl;
            idx++;
        }
        myfile.close();
    }

    double signedAngleBetweenVectorsInSamePlane(const Eigen::Vector3d& vA, const Eigen::Vector3d& vB, const Eigen::Vector3d& vN){
        // note: if possible, you should make this depend on signedAngle()
        // you have plane normal vN and two vectors in that plane vA and vB.
        // this function computes the signed angle between these vectors as atan2((Vb x Va) . Vn, Va . Vb) (with vN normalized)
        // should return an angle in [-pi,pi]
        // sign convention is: [insert here]
        // based on https://stackoverflow.com/a/33920320
        return atan2((vB.normalized().cross(vA.normalized())).dot(vN.normalized()), vA.normalized().dot(vB.normalized()));
    }

    double wrapToPi(const double& ang){
        // take in angle in radians and wrap it to interval [-pi,pi]
        return remainder(ang, 2.0*M_PI);
    }

    double unsignedAngle(const Eigen::Vector3d& av, const Eigen::Vector3d& bv, boost::optional<gtsam::Matrix13 &> H_a, boost::optional<gtsam::Matrix13 &> H_b){
        // returns unsigned angle on [0,pi] and optionally jacobians
        // note: below we cast some Eigen::Vector3d to gtsam::Point3 in order to get functions which return optional derivatives. better would be to namespace a dot() and cross() function of Eigen::Vector3d which did this.
        // assert(a.norm()>1.0e-5); assert(b.norm()>1.0e-5); // removed these asserts because when the problem is initialized to zero values this will fail.
        // conditions for atan2(y,x) to be well defined: (1) y != 0, (2) sqrt(x^2+y^2)+x != 0
        if(H_a || H_b){ // return optionally with jacobians
            // f(a,b)=f(y(c(a,b)),x(a,b)). use chain rule to put together jacobian.
            gtsam::Point3 a=static_cast<gtsam::Point3>(av);
            gtsam::Point3 b=static_cast<gtsam::Point3>(bv);
            gtsam::Matrix33 dc_da, dc_db;
            gtsam::Point3 c=gtsam::cross(a,b,dc_da,dc_db);
            gtsam::Matrix13 dy_dc;
            double y=gtsam::norm3(c, dy_dc); // y = norm(cross(a,b))
            gtsam::Matrix13 dx_da,dx_db;
            double x=gtsam::dot(a,b,dx_da,dx_db); // x = dot(a,b)
            // do the derivative of f(y,x)=atan2(y,x) manually: https://en.wikipedia.org/wiki/Atan2#Derivative
            double denom=pow(x,2.0)+pow(y,2.0); // common denominator to both derivatives
            gtsam::Matrix11 df_dy, df_dx;
            df_dy(0,0)=x/denom; // this is d(atan2(y,x))/dy (i.e., with respect to the first argument to atan2)
            df_dx(0,0)=-1.0*y/denom; // this is d(atan2(y,x))/dx (i.e., with respect to the second argument to atan2)
            // put it all together
            if(H_a){*H_a=df_dy*dy_dc*dc_da+df_dx*dx_da;}
            if(H_b){*H_b=df_dy*dy_dc*dc_db+df_dx*dx_db;}
            // std::cout<<"condition check: sqrt(x^2+y^2)+x =/= 0. LHS is "<<sqrt(pow(x,2)+pow(y,2))+x<<std::endl;
            return atan2(y,x);
        }else{ // just return the angle, no jacobian was requested.
            return atan2((av.cross(bv)).norm(),av.dot(bv));
        }
    }

    double signedAngle(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& refVec, boost::optional<gtsam::Matrix13 &> H_a, boost::optional<gtsam::Matrix13 &> H_b){
        // find the unsigned angle between a and b, and then use the input reference vector refVec to determine sign,
        // i.e., if cross(a,b) points in same direction as refVec, we say this is a positive (right-handed) rotation
        // returns signed angle on [-pi,pi]
        // this version also allows for optional computation of jacobians H_a and H_b
        // assert(refVec.norm()>1.0e-5); // removed these asserts because when the problem is initialized to zero values this will fail.
        if(H_a || H_b){
            // this function is f(a,b)=f(u(a,b)) where u is the unsigned angle function
            gtsam::Matrix13 df_da, df_db, du_da, du_db;
            double signedAng=unsignedAngle(a,b,du_da,du_db);
            df_da=du_da; df_db=du_db;
            if(unsignedAngle(refVec,a.cross(b))>M_PI/2){ // cross(a,b) and refVec point in opposite directions, this is a negative rotation
                signedAng=-1.0*signedAng;
                // also flip signs of jacobians
                df_da=-1.0*df_da; df_db=-1.0*df_db;
            }
            if(H_a){*H_a=df_da;}
            if(H_b){*H_b=df_db;}
            return signedAng;
        }else{ // just compute angle, no jacobian requested
            double signedAng=unsignedAngle(a,b);
            if(unsignedAngle(refVec,a.cross(b))>M_PI/2){ // cross(a,b) and refVec point in opposite directions, this is a negative rotation
                signedAng=-1.0*signedAng;
            }
            return signedAng;
        }
    }

    gtsam::Point3 sub(const gtsam::Point3& v1, const gtsam::Point3& v2, boost::optional<gtsam::Matrix33 &> H_v1, boost::optional<gtsam::Matrix33 &> H_v2){
        // subtract two vectors v(v1,v2)=(v1-v2) and return optional jacobians
        // see deprecated gtsam method Point3::sub
        if(H_v1){*H_v1=gtsam::Matrix33::Identity();}
        if(H_v2){*H_v2=-1.0*gtsam::Matrix33::Identity();}
        return v1-v2;
    }

    gtsam::Vector3 scalarTimesVector(const double& a, const gtsam::Vector3& b, boost::optional<gtsam::Matrix31&> H_a, boost::optional<gtsam::Matrix33&> H_b){
        // trivially multiple a scalar a by a vector b, and return optional jacobians
        if(H_a){*H_a=b;} // df/da
        if(H_b){*H_b=a*gtsam::Matrix33::Identity();} // df/db
        return a*b;
    }

    gtsam::Matrix33 skewSymmetric(const gtsam::Vector3 &v){
        // construct skew symmetric matrix
        gtsam::Matrix H = gtsam::Matrix::Zero(3, 3);
        H(0, 1) = -v[2];
        H(0, 2) = v[1];
        H(1, 0) = v[2];
        H(1, 2) = -v[0];
        H(2, 0) = -v[1];
        H(2, 1) = v[0];
        return H;
    }

    double errorFunScalarMax(double x, double xmax, double a, boost::optional<gtsam::Matrix11 &> H_x){
        // an error function e(x) which is approximately zero if x<xmax and nonzero increasing if x>xmax, and return optional jacobian
        // designed to be C^2
        double e=0.0;
        if(x>xmax){ e=(x-xmax)*pow(tanh(a*(x-xmax)),2.0); }
        if(H_x){
            if(x>xmax){
                gtsam::Matrix11 de_dx;
                de_dx(0,0)=pow(tanh(a*(x - xmax)),2.0) - 2.0*a*tanh(a*(x - xmax))*( pow(tanh(a*(x - xmax)),2.0) - 1.0)*(x - xmax);
                *H_x=de_dx; // de/dx
            }else{
                *H_x=gtsam::Matrix11::Zero();
            }
        }
        return e;
    }

    double errorFunScalarMin(double x, double xmin, double a, boost::optional<gtsam::Matrix11 &> H_x){
        // a piecewise error function e(x) which is approximately zero if x>xmin and nonzero increasing if x<xmin, and return optional jacobian
        // designed to be C^2
        double e=0.0;
        if(x<xmin){ e=(xmin-x)*pow(tanh(a*(xmin-x)),2.0); }
        if(H_x){
            if(x<xmin){
                gtsam::Matrix11 de_dx;
                de_dx(0,0)=2.0*a*tanh(a*(x - xmin))*(   pow(tanh(a*(x - xmin)),2.0) - 1.0)*(x - xmin) - pow(tanh(a*(x - xmin)),2.0);
                *H_x=de_dx; // de/dx
            }else{
                *H_x=gtsam::Matrix11::Zero();
            }
        }
        return e;
    }

    double errorFunScalarMaxMin(double x, double xmax, double xmin, double a, boost::optional<gtsam::Matrix11 &> H_x){
        // two-sided error function which is approximately zero when xmin<x<xmax and increasing positively otherwise.
        // also returns optional derivative de/dx. designed to be C^2.
        // this is simply the addition of max and min functions, e(x)=emax(x)+emin(x)
        gtsam::Matrix11 demax_dx,demin_dx;
        double emax=errorFunScalarMax(x,xmax,a,demax_dx);
        double emin=errorFunScalarMin(x,xmin,a,demin_dx);
        double e=emax+emin;
        if(H_x){ // de/dx = de_demax*demax_dx + de_demin*demin_dx, but de_demax,de_demin=1.0
            gtsam::Matrix11 de_dx=demax_dx+demin_dx;
            *H_x=de_dx;
        }
        return e;
    }

    std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> VectorVectorDoubleToVectorEigenVector(std::vector<std::vector<double>> vecIn){
        //todo: convert all of my vector<double> types to Eigen::VectorXd
        std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> vecOut(vecIn.size());
        for(std::size_t i=0;i<vecIn.size();i++){
            vecOut[i]=Eigen::Vector4d(vecIn[i][0],vecIn[i][1],vecIn[i][2],vecIn[i][3]);
        }
        return vecOut;
    }

    std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> Rot3VectorToQuaternionVector(std::vector<gtsam::Rot3> Rot3Vector){
       // transform a vector<gtsam::Rot3> to a vector<Vector4d> of quaternions
        std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> quatVector(Rot3Vector.size());
       for(uint i=0;i<Rot3Vector.size();i++){
           quatVector[i]=Rot3Vector[i].quaternion(); // [w x y z]
       }
       // unwrap it before you return it
       std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> quatVectorUnwrapped=quatUnwrap(quatVector);
       return quatVector;
   }

    Eigen::MatrixXd Rot3VectorToQuaternionMatrix(const std::vector<gtsam::Rot3>& Rot3Vector){
        // vector<Rot3> => Nx4 Eigen::MatrixXd
        Eigen::MatrixXd M(Rot3Vector.size(),4);
        for(uint i=0; i<Rot3Vector.size(); i++){
            gtsam::Vector4 q=Rot3Vector[i].quaternion();
            M(i,0)=q[0]; M(i,1)=q[1]; M(i,2)=q[2];M (i,3)=q[3];
        }
        return M;
    }

    std::vector<gtsam::Rot3> QuaternionVectorToRot3Vector(std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> quatVector){
        // convert a vector<Vector4d> of quaternions to a vector<Rot3>
        std::vector<gtsam::Rot3> Rot3Vector(quatVector.size()); // will output
        for(uint i=0;i<quatVector.size();i++){
            Rot3Vector[i]=gtsam::Rot3(quatVector[i][0],quatVector[i][1],quatVector[i][2],quatVector[i][3]);
        }
        return Rot3Vector;
    }

    Eigen::Vector3d rollPitchYawRmseBetweenRot3Arrays(const std::vector<gtsam::Rot3>& rA, const std::vector<gtsam::Rot3>& rB){
        // take in two sets of rotations and compute relative roll, pitch, yaw (RMSEs) of R[A->B]
        // this assumes both input orientations are body->nav
        assert(rA.size()==rB.size());
        std::vector<gtsam::Rot3> R_A_to_B(rA.size());
        std::vector<double> yawSqErr(rA.size()), pitchSqErr(rA.size()), rollSqErr(rA.size());
        for(uint k=0; k<rA.size(); k++){
            R_A_to_B[k]=rB[k].inverse()*rA[k]; // R[A->B]=R[B->N]'*R[A->N]
            gtsam::Vector3 rpyErr=R_A_to_B[k].rpy();
            yawSqErr[k]=pow(rpyErr[2],2);
            pitchSqErr[k]=pow(rpyErr[1],2);
            rollSqErr[k]=pow(rpyErr[0],2);
        }
        // now compute RMSEs
        double yawRMSE = sqrt((std::accumulate(yawSqErr.begin(), yawSqErr.end(), 0.0))/yawSqErr.size());
        double pitchRMSE = sqrt((std::accumulate(pitchSqErr.begin(), pitchSqErr.end(), 0.0))/pitchSqErr.size());
        double rollRMSE = sqrt((std::accumulate(rollSqErr.begin(), rollSqErr.end(), 0.0))/rollSqErr.size());
        // put in vector for output
        Eigen::Vector3d rpyRmse(rollRMSE,pitchRMSE,yawRMSE);
        return rpyRmse;
    }

    gtsam::Vector3 compute_wRel(gtsam::Vector4 qA, gtsam::Vector4 qB, gtsam::Vector3 wA, gtsam::Vector3 wB){
        double qA1=qA[0]; double qA2=qA[1]; double qA3=qA[2]; double qA4=qA[3];
        double qB1=qB[0]; double qB2=qB[1]; double qB3=qB[2]; double qB4=qB[3];
        double wA1=wA[0]; double wA2=wA[1]; double wA3=wA[2];
        double wB1=wB[0]; double wB2=wB[1]; double wB3=wB[2];
        // now compute wRel: wB-wA in frame A
        gtsam::Vector3 wRel;
        wRel[0]=-wA1+wB1*(pow(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4,2.0)+pow(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3,2.0)-pow(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2,2.0)-pow(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1,2.0))-wB2*((qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0)+wB3*((qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0);
        wRel[1]=-wA2+wB2*(pow(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4,2.0)-pow(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3,2.0)+pow(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2,2.0)-pow(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1,2.0))+wB1*((qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0)-wB3*((qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0-(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0);
        wRel[2]=-wA3+wB3*(pow(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4,2.0)-pow(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3,2.0)-pow(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2,2.0)+pow(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1,2.0))-wB1*((qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0)+wB2*((qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0);
        return wRel;
    }

    Eigen::MatrixXd d_errorVec_d_qA_func(gtsam::Vector4 qA, gtsam::Vector4 qB, gtsam::Vector3 wA, gtsam::Vector3 wB){
        // rename variables
        double qA1=qA[0]; double qA2=qA[1]; double qA3=qA[2]; double qA4=qA[3];
        double qB1=qB[0]; double qB2=qB[1]; double qB3=qB[2]; double qB4=qB[3];
        //double wA1=wA[0]; double wA2=wA[1]; double wA3=wA[2];
        double wB1=wB[0]; double wB2=wB[1]; double wB3=wB[2];

        Eigen::MatrixXd d_errorVec_d_qA_mat(3,4);
        d_errorVec_d_qA_mat(0,0) = -wB1*(qB1*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB2*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0-qB3*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qB4*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0)+wB2*(qB1*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB2*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qB4*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qB3*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)-wB3*(qB1*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qB3*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB2*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qB4*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0);
        d_errorVec_d_qA_mat(0,1) = -wB1*(qB2*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qB1*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qB3*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB4*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0)+wB2*(qB1*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qB3*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB2*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB4*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)+wB3*(qB1*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB2*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qB4*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB3*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0);
        d_errorVec_d_qA_mat(0,2) = -wB1*(qB1*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qB3*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qB2*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB4*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)+wB2*(qB2*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB1*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qB3*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qB4*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0)+wB3*(qB1*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qB2*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0-qB3*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qB4*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0);
        d_errorVec_d_qA_mat(0,3) = -wB1*(qB1*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qB2*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qB4*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB3*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)-wB2*(qB1*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qB2*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qB3*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qB4*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0)+wB3*(qB2*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB1*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0-qB3*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB4*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0);
        d_errorVec_d_qA_mat(1,0) = -wB1*(qB1*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qB2*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qB4*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB3*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)-wB2*(qB1*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qB2*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qB3*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qB4*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0)+wB3*(qB2*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB1*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0-qB3*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB4*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0);
        d_errorVec_d_qA_mat(1,1) = wB1*(qB1*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qB3*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qB2*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB4*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)-wB2*(qB2*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB1*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qB3*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qB4*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0)-wB3*(qB1*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qB2*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0-qB3*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qB4*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0);
        d_errorVec_d_qA_mat(1,2) = -wB1*(qB2*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qB1*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qB3*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB4*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0)+wB2*(qB1*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qB3*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB2*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB4*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)+wB3*(qB1*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB2*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qB4*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB3*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0);
        d_errorVec_d_qA_mat(1,3) = wB1*(qB1*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB2*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0-qB3*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qB4*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0)-wB2*(qB1*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB2*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qB4*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qB3*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)+wB3*(qB1*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qB3*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB2*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qB4*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0);
        d_errorVec_d_qA_mat(2,0) = wB1*(qB1*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qB3*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qB2*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB4*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)-wB2*(qB2*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB1*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qB3*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qB4*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0)-wB3*(qB1*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qB2*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0-qB3*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qB4*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0);
        d_errorVec_d_qA_mat(2,0) = wB1*(qB1*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qB2*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qB4*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB3*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)+wB2*(qB1*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qB2*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qB3*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qB4*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0)-wB3*(qB2*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB1*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0-qB3*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB4*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0);
        d_errorVec_d_qA_mat(2,2) = -wB1*(qB1*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB2*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0-qB3*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qB4*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0)+wB2*(qB1*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB2*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qB4*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qB3*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)-wB3*(qB1*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qB3*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB2*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qB4*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0);
        d_errorVec_d_qA_mat(2,3) = -wB1*(qB2*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qB1*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qB3*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB4*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0)+wB2*(qB1*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qB3*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB2*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB4*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)+wB3*(qB1*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB2*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qB4*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB3*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0);
        d_errorVec_d_qA_mat(2,3) = -wB1*(qB2*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qB1*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qB3*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB4*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0)+wB2*(qB1*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qB3*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB2*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB4*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)+wB3*(qB1*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qB2*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qB4*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qB3*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0);
        return d_errorVec_d_qA_mat;
    };

    Eigen::MatrixXd d_errorVec_d_qB_func(gtsam::Vector4 qA, gtsam::Vector4 qB, gtsam::Vector3 wA, gtsam::Vector3 wB) {
        // rename variables
        double qA1=qA[0]; double qA2=qA[1]; double qA3=qA[2]; double qA4=qA[3];
        double qB1=qB[0]; double qB2=qB[1]; double qB3=qB[2]; double qB4=qB[3];
        //double wA1=wA[0]; double wA2=wA[1]; double wA3=wA[2];
        double wB1=wB[0]; double wB2=wB[1]; double wB3=wB[2];

        Eigen::MatrixXd d_errorVec_d_qB(3,4);
        d_errorVec_d_qB(0,0) = -wB1*(qA1*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qA2*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qA3*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qA4*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0)+wB2*(qA1*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qA2*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qA4*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA3*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)+wB3*(qA1*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*-2.0+qA3*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA2*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qA4*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0);
        d_errorVec_d_qB(0,1) = -wB1*(qA2*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA1*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0-qA3*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qA4*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0)+wB2*(qA1*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*-2.0+qA3*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA2*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qA4*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)-wB3*(qA1*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qA2*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qA4*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA3*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0);
        d_errorVec_d_qB(0,2) = -wB1*(qA1*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*-2.0+qA3*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA2*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qA4*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)-wB2*(qA2*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA1*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0-qA3*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qA4*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0)-wB3*(qA1*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qA2*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qA3*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qA4*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0);
        d_errorVec_d_qB(0,3) = wB1*(qA1*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qA2*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qA4*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA3*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)+wB2*(qA1*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qA2*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qA3*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qA4*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0)-wB3*(qA2*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA1*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0-qA3*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qA4*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0);
        d_errorVec_d_qB(1,0) = wB1*(qA1*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*-2.0+qA2*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qA4*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA3*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)-wB2*(qA1*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA2*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0-qA3*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qA4*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0)+wB3*(qA2*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*-2.0+qA1*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qA3*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qA4*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0);
        d_errorVec_d_qB(1,1) = -wB1*(qA1*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qA3*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA2*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qA4*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)+wB2*(qA2*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*-2.0+qA1*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qA3*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qA4*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0)+wB3*(qA1*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA2*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0-qA3*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qA4*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0);
        d_errorVec_d_qB(1,2) = -wB1*(qA2*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*-2.0+qA1*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qA3*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qA4*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0)-wB2*(qA1*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qA3*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA2*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qA4*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)+wB3*(qA1*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*-2.0+qA2*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qA4*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA3*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0);
        d_errorVec_d_qB(1,3) = -wB1*(qA1*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA2*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0-qA3*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qA4*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0)-wB2*(qA1*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*-2.0+qA2*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qA4*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA3*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)-wB3*(qA1*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qA3*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA2*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qA4*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0);
        d_errorVec_d_qB(2,0) = wB1*(qA1*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qA3*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA2*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qA4*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)+wB2*(qA2*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qA1*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qA3*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qA4*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0)-wB3*(qA1*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA2*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qA3*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qA4*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0);
        d_errorVec_d_qB(2,1) = -wB1*(qA1*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qA2*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qA4*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA3*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)-wB2*(qA1*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA2*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qA3*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qA4*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0)-wB3*(qA2*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qA1*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qA3*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qA4*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0);
        d_errorVec_d_qB(2,2) = wB1*(qA1*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA2*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qA3*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qA4*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0)-wB2*(qA1*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qA2*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qA4*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA3*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)+wB3*(qA1*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qA3*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA2*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qA4*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0);
        d_errorVec_d_qB(2,3) = wB1*(qA2*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0-qA1*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0+qA3*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qA4*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0)-wB2*(qA1*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0-qA3*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA2*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0+qA4*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0)-wB3*(qA1*(qA1*qB4-qA2*qB3+qA3*qB2-qA4*qB1)*2.0-qA2*(qA1*qB3-qA3*qB1+qA2*qB4-qA4*qB2)*2.0+qA4*(qA1*qB1+qA2*qB2+qA3*qB3+qA4*qB4)*2.0+qA3*(qA1*qB2-qA2*qB1-qA3*qB4+qA4*qB3)*2.0);
        return d_errorVec_d_qB;
    };

    gtsam::Matrix33 quat2dcm(const Eigen::Vector4d& q){
        // from wikipedia https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Rotation_matrix_%E2%86%94_quaternion
        // converts quat->dcm
        gtsam::Matrix33 r;
        double qr=q[0]; double qi=q[1]; double qj=q[2]; double qk=q[3];
        r(0,0)=1.0-2.0*qj*qj-2.0*qk*qk;
        r(0,1)=2.0*(qi*qj-qk*qr);
        r(0,2)=2.0*(qi*qk+qj*qr);
        r(1,0)=2.0*(qi*qj+qk*qr);
        r(1,1)=1.0-2.0*qi*qi-2.0*qk*qk;
        r(1,2)=2.0*(qj*qk-qi*qr);
        r(2,0)=2.0*(qi*qk-qj*qr);
        r(2,1)=2.0*(qi*qr+qj*qk);
        r(2,2)=1.0-2.0*qi*qi-2.0*qj*qj;
        return r;
    }

    gtsam::Vector4 dcm2quat(const gtsam::Matrix33& r){
        // from wikipedia https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Rotation_matrix_%E2%86%94_quaternion
        // converts dcm->quat
        double qr=0.5*sqrt(1+r(0,0)+r(1,1)+r(2,2));
        double qi=0.25/qr*(r(2,1)-r(1,2));
        double qj=0.25/qr*(r(0,2)-r(2,0));
        double qk=0.25/qr*(r(1,0)-r(0,1));
        gtsam::Vector4 q; q<<qr,qi,qj,qk;
        return q;
    }

    bool are_quats_in_same_half_space(std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> qA, std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> qB){
        // evaluates if one of the quaternions needs to be "flipped" or not for comparison
        //todo: the robust way to do this is by the angle between the vector components of a quat set. implement this. for now, it's a dumb method because I don't wanna deal with the different sample rates between quats.
        if(qA.size()!=qB.size()) { // different sizes, do dumb method
            //dumb/incorrect method: take average of four components. find largest magnitude one. see if the other quat has the same sign on this component.
            // (0) find average of all components of quatA
            double qA_s_avg = 0.0, qA_x_avg = 0.0, qA_y_avg = 0.0, qA_z_avg = 0.0;
            for(uint i = 0; i < qA.size(); i++) {
                qA_s_avg += qA[i][0];
                qA_x_avg += qA[i][1];
                qA_y_avg += qA[i][2];
                qA_z_avg += qA[i][3];
            }
            qA_s_avg /= qA.size();
            qA_x_avg /= qA.size();
            qA_y_avg /= qA.size();
            qA_z_avg /= qA.size();
            double qB_s_avg = 0.0, qB_x_avg = 0.0, qB_y_avg = 0.0, qB_z_avg = 0.0;
            for(uint i = 0; i < qB.size(); i++) {
                qB_s_avg += qB[i][0];
                qB_x_avg += qB[i][1];
                qB_y_avg += qB[i][2];
                qB_z_avg += qB[i][3];
            }
            qB_s_avg /= qB.size();
            qB_x_avg /= qB.size();
            qB_y_avg /= qB.size();
            qB_z_avg /= qB.size();
            // now get largest magnitude component in qA averages
            gtsam::Vector4 qA_avg(qA_s_avg, qA_x_avg, qA_y_avg, qA_z_avg);
            gtsam::Vector4 qB_avg(qB_s_avg, qB_x_avg, qB_y_avg, qB_z_avg);
            std::ptrdiff_t idxMaxA;
            qA_avg.maxCoeff(&idxMaxA);
            //cout << "old method (mixed sizes): " << (abs(abs(qA_avg[idxMaxA]) - abs(qB_avg[idxMaxA])) < abs(qA_avg[idxMaxA] - qB_avg[idxMaxA])) << endl;
            if (abs(abs(qA_avg[idxMaxA]) - abs(qB_avg[idxMaxA])) < abs(qA_avg[idxMaxA] - qB_avg[idxMaxA])) {
                // different signs
                return false;
            } else { // same signs
                return true;
            }
        } else { // same size, do robust method
            // (0) robust method of this: calculate angle between vector components
            std::vector<double> ang_bw_quat_vector_components(qA.size());
            for(uint i = 0; i < qA.size(); i++) {
                Eigen::Vector3d qAVec(qA[i][1], qA[i][2], qA[i][3]);
                Eigen::Vector3d qBVec(qB[i][1], qB[i][2], qB[i][3]);
                ang_bw_quat_vector_components[i] = unsignedAngle(qAVec, qBVec);
            }
            double avgAngleBwQuatVecComponents = std::accumulate(ang_bw_quat_vector_components.begin(), ang_bw_quat_vector_components.end(), 0.0) / ang_bw_quat_vector_components.size();
            if(avgAngleBwQuatVecComponents>M_PI/2){ // it's over 180deg, not in same half space
                return false;
            }else{ // are in same half space
                return true;
            }
        }
    }

    Eigen::VectorXd angleUnwrapOnKnownDomain(const Eigen::VectorXd& angle, const double& domainMin, const double& domainMax, const double& tol){
        // domainMin and domainMax define the min and max of the domain the angle exists on
        // tol is the minimum size (abs) jump which must occur to be considered a jump (wrap). This is in same units as domainWidth.
        // check
        assert(domainMax>domainMin);
        // go
        Eigen::VectorXd out=angle;
        double domainWidth=domainMax-domainMin, domainCenter=(domainMax+domainMin)/2;
        for (uint i = 1; i < out.size(); i++) { // loop over all angles in array
            double d = out[i] - out[i-1]; // compute old difference
            if(abs(d)>tol){ // a jump (wrap) has been detected. if d positive, you need to reduce angle[i] (and v/v for d negative)
                // find number of multiples of domainWidth you need to go until d would be less than tol
                double m=ceil(abs(d)/domainWidth);
                double s=(d>0) ? -1.0 : 1.0;
                // update all angles i>=j
                for(uint j=i; j < angle.size(); j++){ // adjust all j>=i by this difference
                    out[j]=out[j]+m*s*domainWidth;
                }
            }
        }
        return out;
    }

    double slope(const std::vector<double>& x, const std::vector<double>& y) {
        // merely calls linearRegression()
        double m, b;
        linearRegression(StdVectorToEigenVector(x),StdVectorToEigenVector(y),m,b);
        return m;
    }

    void linearRegression(const Eigen::VectorXd& x, const Eigen::VectorXd& y, double& m, double& b){
        // least-squares linear regression implementation. given input data d, this function computes slope m and intercept for best-fit linear regression equation y=mx+b
        // this function writes the values to m and b to the input variable references
        // see: https://en.wikipedia.org/wiki/Simple_linear_regression
        double beta=((x.array()-x.mean())*(y.array()-y.mean())).sum()/( (x.array()-x.mean())*(x.array()-x.mean()) ).sum(); // compute slope beta
        double alpha=y.mean()-beta*x.mean(); // compute intercept alpha
        m=beta, b=alpha; // write to reference inputs
    }

    Eigen::VectorXd adjustRegressionSlopeToTargetSlope(const Eigen::VectorXd& x, const Eigen::VectorXd& y, const double& targetSlope){
        // takes in data x,y and performs regression (y=mx+b). Then, creates new ydata (ynew) which has new slope targetSlope but perserves old intercept b.
        // adjustment happens by perserving the original residuals.
        double oldSlope, oldIntercept;
        linearRegression(x,y,oldSlope,oldIntercept);
        // now find residuals
        Eigen::VectorXd e=y.array() - oldSlope*x.array() - oldIntercept*Eigen::VectorXd::Ones(y.size()).array(); // ybar=mx+b, e=(y-ybar) -> residuals e = y-mx-b
        // construct ynew which has targetSlope but perserves residuals e
        Eigen::VectorXd ynew = targetSlope*x.array() + oldIntercept*Eigen::VectorXd::Ones(y.size()).array() + e.array();
        // now test/debug
        double newSlope, newIntercept, tol=1.0e-10;
        linearRegression(x,ynew,newSlope,newIntercept);
        assert(abs(newSlope-targetSlope)<tol); // test new slope (should be targetSlope)
        assert(abs(newIntercept-oldIntercept)<tol); // test new intercept (should be same as old intercept)
        return ynew;
    }

    std::vector<double> unwrappedYawAngle(const std::vector<gtsam::Rot3>& R){
        // returns array of yaw angle, unwrapped, in radians. from testGtsamRot3 we know that the yaw domain is defined on [-pi,+pi]
        Eigen::MatrixXd yprMatrix=gtsamutils::vectorRot3ToYprMatrix(R);
        Eigen::VectorXd yaw=yprMatrix.col(0);
        Eigen::VectorXd yawUnwrapped=angleUnwrapOnKnownDomain(yaw, -1.0*M_PI, M_PI, 2.0*M_PI*0.9);
        return mathutils::EigenVectorToStdVector(yawUnwrapped);
    }

    std::vector<double> adjustSlopeToTargetSlope(const std::vector<double>& origX, const std::vector<double>& origY, const double& newSlope){
        // given a vector of data, adjust its linear slope so that the new slope = newSlope
        std::vector<double> origYShifted(origY.size());
        for(uint k=0; k<origY.size(); k++){
            origYShifted[k]=origY[k]-origY[0];
        }
        const double oldSlope=slope(origX,origYShifted); // old slope
        // now find the multiplicative factor to multiply to the original data in order to produce the new data
        const double multFactor=newSlope/oldSlope; // <--- note: this is a problem if newSlope=0!
        // now create new Y data
        std::vector<double> newY(origY.size());
        for(uint k=0; k<origY.size(); k++){
            newY[k]=(origYShifted[k]*multFactor)+origY[0];
        }
        // now test: make sure the slope of the new data = newSlope
        double tol=1.0e-5;
        double actualNewSlope=slope(origX,newY);
        if(abs(newSlope-actualNewSlope)>tol){
            std::cerr<<"actual new slope ("<<actualNewSlope<<") is different from target new slope ("<<newSlope<<")"<<std::endl;
        }
        return newY;
    }
    std::tuple<Eigen::MatrixXd,Eigen::Matrix4d> simpleImuOrientationForwardEkf(const Eigen::MatrixXd& gyros, const Eigen::MatrixXd& accels, const double& dt, const Eigen::RowVector4d& initX, const Eigen::Matrix4d& initP){
        // outputs orientations as R[N->B]
        // simple orientation EKF on quaternion x
        // --settings --
        const double noise_gyro=2.4e-3;
        const double noise_accel=2.83e-2;
        const double gravity=9.81007;
        Eigen::RowVector3d bias_w(0.0,0.0,0.0);
        Eigen::RowVector3d bias_a(0.0,0.0,0.0);
        // -------------
        // preallocate containers of results
        uint N=gyros.rows();
        Eigen::MatrixXd allX=Eigen::MatrixXd::Zero(N,4); // quaternion array of results
        allX.block<1,4>(0,0)=initX;
        Eigen::RowVector4d x=initX;
        Eigen::Matrix4d P=initP;
        Eigen::Matrix4d Omega=Eigen::Matrix4d::Zero(), F=Eigen::Matrix4d::Zero(), Q=Eigen::Matrix4d::Zero();
        Eigen::MatrixXd G(4,3);
        Eigen::MatrixXd H(3,4);
        Eigen::Matrix3d R_internal=Eigen::Matrix3d::Zero(), R_external=Eigen::Matrix3d::Zero(),R=Eigen::Matrix3d::Zero();
        for(uint k=1; k<N; k++){
            // 1. propogation
            Eigen::RowVector3d w=gyros.row(k) - bias_w;
            // compute F matrix
            Omega(0,0)=0; Omega(0,1)=-w[0]; Omega(0,2)=-w[1]; Omega(0,3)=-w[2];
            Omega(1,0)=w[0]; Omega(1,1)=0; Omega(1,2)=w[2]; Omega(1,3)=-w[1];
            Omega(2,0)=w[1]; Omega(2,1)=-w[2]; Omega(2,2)=0; Omega(2,3)=w[0];
            Omega(3,0)=w[2]; Omega(3,1)=w[1]; Omega(3,2)=-w[0]; Omega(3,3)=0;
            F=Eigen::Matrix4d::Identity()+Omega*dt*0.5;
            //std::cout<<"F="<<F<<std::endl;
            // compute process noise Q
            G(0,0)=-x[1]; G(0,1)=-x[2]; G(0,2)=-x[3];
            G(1,0)=x[0]; G(1,1)=-x[3]; G(1,2)=x[2];
            G(2,0)=x[3]; G(2,1)=x[0]; G(2,2)=-x[1];
            G(3,0)=-x[2]; G(3,1)=x[1]; G(3,2)=x[0];
            G=G*0.5;
            //std::cout<<"G="<<G<<std::endl;
            Q= (G*G.transpose()) * pow(noise_gyro*dt,2.0);
            //std::cout<<"Q="<<Q<<std::endl;
            // propgate state and covariance
            x=(F*x.transpose()).transpose();
            x.normalize();
            //std::cout<<"intermediate x= "<<x<<std::endl;
            P=F*P*F.transpose()+Q;
            //std::cout<<"intermediate P="<<P<<std::endl;
            // 2. update
            Eigen::RowVector3d a=accels.row(k) - bias_a;
            // use unit vector of accel as your observation (ignores magnitude differences)
            Eigen::RowVector3d ea=a.normalized();
            Eigen::RowVector3d ea_prediction(2*(x(1)*x(3)-x(0)*x(2)),2*(x(2)*x(3)+x(0)*x(1)),pow(x(0),2.0)-pow(x(1),2)-pow(x(2),2)+pow(x(3),2));
            Eigen::RowVector3d y=ea-ea_prediction; // residual
            //std::cout<<"accel residual: "<<y<<std::endl;
            // compute measurement matrix H
            H(0,0)=-x(2); H(0,1)=x(3); H(0,2)=-x(0); H(0,3)=x(1);
            H(1,0)=x(1); H(1,1)=x(0); H(1,2)=x(3); H(1,3)=x(2);
            H(2,0)=x(0); H(2,1)=-x(1); H(2,2)=-x(2); H(2,3)=x(3);
            H=H*2.0;
            //std::cout<<"H="<<H<<std::endl;
            // measurement noise matrix R
            R_internal=Eigen::Matrix3d::Identity()*pow(noise_accel/a.norm(),2.0);
            R_external=Eigen::Matrix3d::Identity()*pow(1.0-gravity/a.norm(),2.0);
            R=R_internal+R_external;
            //std::cout<<"R="<<R<<std::endl;
            // update
            Eigen::Matrix3d S=H*P*H.transpose()+R;
            //std::cout<<"S="<<S<<std::endl;
            Eigen::MatrixXd K=P*H.transpose() * S.inverse();
            //std::cout<<"K="<<K<<std::endl;
            x=(x.transpose()+(K*y.transpose())).transpose();
            P=(Eigen::Matrix4d::Identity()-K*H)*P;
            //std::cout<<"x[k="<<k<<"]="<<x<<std::endl;
            //std::cout<<"updated P="<<P<<std::endl;
            // cleanup
            x.normalize();
            P=(P+P.transpose())*0.5; // symmetricize P
            allX.block<1,4>(k,0)=x;
        }
        Eigen::Matrix4d finalP=P;
        return std::make_tuple(allX,finalP);
    }

    Eigen::MatrixXd simpleImuOrientationForwardBackwardEkfWrapper(const Eigen::MatrixXd& gyros, const Eigen::MatrixXd& accels, const double& dt, const uint& numFBPasses){
        // outputs orientations as R[N->B]
        // initialize x and P
        Eigen::RowVector4d initX(1.0,0.0,0.0,0.0);
        Eigen::Matrix4d initP=Eigen::Matrix4d::Identity();
        Eigen::MatrixXd currentX;
        Eigen::Matrix4d currentFinalP;
        Eigen::MatrixXd currentGyros=gyros;
        Eigen::MatrixXd currentAccels=accels;
        assert(currentGyros.isApprox(gyros)); // make sure asserts work
        assert(currentAccels.isApprox(accels)); // make sure asserts work
        // make sure that if you flip a matrix twice it returns to original matrix
        assert(((currentAccels.colwise().reverse()).colwise().reverse()).isApprox(accels));
        uint numPasses=0;
        while(numPasses<numFBPasses){
            // forward pass
            std::tie(currentX,currentFinalP)=simpleImuOrientationForwardEkf(currentGyros,currentAccels,dt,initX,initP);
            // backward pass
            // now take that original data, flip it (and neg the gyro data) and then run it backward
            // remember to initialize x and P to last index of the forward pass!
            currentGyros.colwise().reverseInPlace(); // flip gyros and -1
            currentGyros=-1.0*currentGyros;
            currentAccels.colwise().reverseInPlace(); // flip accels
            // note: right now asserts should not work when checking gyros and accels against original function inputs, i.e., the following two lines should fail:
            //assert(currentGyros.isApprox(gyros)); // should fail
            //assert(currentAccels.isApprox(accels)); // should fail
            std::tie(currentX,currentFinalP)=simpleImuOrientationForwardEkf(currentGyros,currentAccels,dt,currentX.block<1,4>(currentX.rows()-1,0),currentFinalP);
            // but now remember that your current X is upside down, and that currentFinalP is now your P at the first index
            currentX.colwise().reverseInPlace(); // flip back
            initP=currentFinalP;
            // flip back accels and gyros, and also multiply gyros by -1 to get back to where you were
            currentGyros.colwise().reverseInPlace(); // flip gyros and -1
            currentGyros=-1.0*currentGyros;
            currentAccels.colwise().reverseInPlace(); // flip accels
            // you can use assert to check that it's now like the original function input
            assert(currentGyros.isApprox(gyros));
            assert(currentAccels.isApprox(accels));
            numPasses++;
        }
        return currentX;
    }

    Eigen::VectorXd imuInternalPrecessionAboutStaticVector(const std::vector<gtsam::Rot3>& R_B_to_N, const gtsam::Point3& vproxB, const Eigen::Vector3d& seedStartVec){
        // you've input a set of orientations R[B->N] and a vector v which is constant in the body frame
        // compute the total angle of how much the IMU rotates about this vector in the nav frame
        // motivation: the yaw angle is hard to interpret, this is sorta like yaw, but a concept of yaw which is consistent to the internal skeleton of the human
        // ----------------------------------------------------------------------- //
        Eigen::VectorXd angles(R_B_to_N.size());
        // find an orthogonal vector to vB using the cross product and seedStartVec
        Eigen::Vector3d seedStartVecB=seedStartVec.normalized();
        Eigen::Vector3d vproxBnorm=vproxB.normalized();
        Eigen::Vector3d orthoVecB=vproxBnorm.cross(seedStartVecB); // so, orthoVec is orthogonal to vB
        orthoVecB.normalize(); // normalize! vproxB and seedStartVec are not in general orthogonal
        Eigen::Vector3d orthoVecNavInit=R_B_to_N[0]*orthoVecB;
        // ---- this is how the original algo looked, but I realized that if you just calculate the angle differential at each iteration and add them up at the end it takes care of unwrapping issues for you.
        /* now, we ask the question: what's the angle between orthoVec(initial) and vBnorm over time?
        Eigen::Vector3d orthoVecNavInit;
        for(uint k=0; k<R_B_to_N.size();k++){
            Eigen::Vector3d orthoVecNav=R_B_to_N[k]*orthoVecB;
            // () pull orthoVecNavInit back into the body frame
            Eigen::Vector3d orthoVecNavInitBody=R_B_to_N[k].inverse()*orthoVecNavInit;
            // () now correct it so that it's in the appropriate plane: orthogonal to vBnorm
            Eigen::Vector3d orthoVecNavInitBodyCorrected=(vproxBnorm.cross(orthoVecNavInitBody)).cross(vproxBnorm);
            // what's the angle between orthoVecNav and orthoVecNavInit?
            angles(k)=signedAngleBetweenVectorsInSamePlane(orthoVecB, orthoVecNavInitBodyCorrected, vproxBnorm);
            if(abs(unsignedAngle(vproxBnorm,orthoVecNavInitBodyCorrected)-M_PI/2)>1e-10) { // this angle should be 90 if your projections are correct
                throw std::runtime_error("angle was not 90 degrees");
            }
        }
        */
        // ----------------------------------------------------------------------- %
        angles(0)=0.0; // first index is zero, other are relative to it.
        // new algo: calculate delta angle from [k-1]->[k] and add up at end to naturally avoid wrapping issues
        for(uint k=1; k<R_B_to_N.size(); k++){ // start iteration at second index, you're finding [k]-[k-1]
            // () pull orthoVecNavInit back into the body frame
            Eigen::Vector3d orthoVecNavInitBodyKm1=R_B_to_N[k-1].inverse()*orthoVecNavInit; // @ k-1
            Eigen::Vector3d orthoVecNavInitBodyK=R_B_to_N[k].inverse()*orthoVecNavInit; // @ k
            // () now correct it so that it's in the appropriate plane: orthogonal to vBnorm
            Eigen::Vector3d orthoVecNavInitBodyCorrectedKm1=(vproxBnorm.cross(orthoVecNavInitBodyKm1)).cross(vproxBnorm);
            Eigen::Vector3d orthoVecNavInitBodyCorrectedK=(vproxBnorm.cross(orthoVecNavInitBodyK)).cross(vproxBnorm);
            // () find the change in angle from [k-1]->[k]
            double dIPAngle=signedAngleBetweenVectorsInSamePlane(orthoVecNavInitBodyCorrectedKm1,orthoVecNavInitBodyCorrectedK,vproxBnorm);
            // () now insert that into angles array. k = [k-1]+delta angle
            angles(k)=angles(k-1)+dIPAngle;
        }
        return angles;
    }

    std::vector<double> EigenVectorToStdVector(const Eigen::VectorXd& vec){
        std::vector<double> newvec(vec.data(), vec.data() + vec.rows() * vec.cols());
        return newvec;
    }

    Eigen::VectorXd StdVectorToEigenVector(const std::vector<double>& vec){
        return Eigen::VectorXd::Map(vec.data(),vec.size());
    }

    double meanAngBwAxesNavFrame(const std::vector<gtsam::Rot3>& rA, const gtsam::Unit3& kA, const std::vector<gtsam::Rot3>& rB, const gtsam::Unit3& kB){
        // you have rotations rA and rB which are B->N, and you have axes kA and kB in those local frames A, B
        // compute the mean (unsigned) angle between these axes when rotated into the navigation frame
        Eigen::Vector3d vA=kA.unitVector(), vB=kB.unitVector();
        std::vector<double> angBw(rA.size());
        for(uint k=0; k<rA.size(); k++){
            Eigen::Vector3d kAN=rA[k].rotate(vA), kBN=rB[k].rotate(vB);
            angBw[k]=unsignedAngle(kAN,kBN);
        }
        return mean(angBw);
    }

    double mean(const std::vector<double>& x){
        return std::accumulate(x.begin(), x.end(), 0.0)/x.size(); // return mean
    }

    double stdev(const std::vector<double>& x){
        double xmean=mean(x);
        double sq_sum = std::inner_product(x.begin(), x.end(), x.begin(), 0.0);
        return std::sqrt(sq_sum / x.size() - xmean * xmean); // standard deviation
    }

    double max(const std::vector<double>& x){
        return *std::max_element(x.begin(),x.end());
    }

    double min(const std::vector<double>& x){
        return *std::min_element(x.begin(),x.end());
    }

    double median(const std::vector<double>& x){
        std::vector<double> x_copy=x; // copy x to new array
        sort(x_copy.begin(), x_copy.end()); // sort
        double tmedian;
        if(x_copy.size() % 2 == 0){ // even
            tmedian = (x_copy[x_copy.size() / 2 - 1] + x_copy[x_copy.size() / 2]) / 2;
        }else { // odd
            tmedian = x_copy[x_copy.size() / 2];
        }
        return tmedian;
    }

    double atan2(double y, double x, boost::optional<gtsam::Matrix11 &> Hy, boost::optional<gtsam::Matrix11 &> Hx){
        // atan2 + optional derivatives (https://en.wikipedia.org/wiki/Atan2#Derivative)
        double f=std::atan2(y,x);
        if(Hy){ *Hy=gtsam::Matrix11(x/(pow(x,2.0)+pow(y,2.0))); } // df/dy
        if(Hx){ *Hx=gtsam::Matrix11(-1.0*y/(pow(x,2.0)+pow(y,2.0))); } // df/dx
        return f;
    }

    gtsam::Vector3 projmk(const gtsam::Vector3& m, const gtsam::Unit3& k, boost::optional<gtsam::Matrix33 &> H_m, boost::optional<gtsam::Matrix32 &> H_k){
        // m is a R(3) vector, k is a SU(2) vector. find the projection of m onto k and return optional jacobians.
        // since k is already a unit vector, we can drop the normalizations and use: p = m.dot(k) * k;
        gtsam::Matrix32 dr_dk;
        gtsam::Vector3 r=k.unitVector(dr_dk); // r := k transformed to R(3)
        gtsam::Matrix13 dmdr_dm, dmdr_dr;
        double mdr=gtsam::dot(m,r,dmdr_dm,dmdr_dr); // casting them as gtsam::Point3 to take advantage of the derivatives in gtsam's Point3::dot
        gtsam::Matrix31 dmdrtr_dmdr; gtsam::Matrix33 dmdrtr_dr;
        gtsam::Vector3 mdrtr=mathutils::scalarTimesVector(mdr, r, dmdrtr_dmdr, dmdrtr_dr); // mdr * r
        if(H_m){ // dmdrtr_dm = dmdrtr_dmdr*dmdr_dm
            *H_m= dmdrtr_dmdr*dmdr_dm;
        }
        if(H_k){ // dmdrtr_dmdr*dmdr_dr*dr_dk + dmdrtr_dr*dr_dk
            *H_k=dmdrtr_dmdr*dmdr_dr*dr_dk + dmdrtr_dr*dr_dk;
        }
        return mdrtr;
    }

    gtsam::Vector3 ptSeparation(const gtsam::Pose3& xA, const gtsam::Point3& sA, const gtsam::Pose3& xB, const gtsam::Point3& sB, boost::optional<gtsam::Matrix36 &> H_xA, boost::optional<gtsam::Matrix33 &> H_sA, boost::optional<gtsam::Matrix36 &> H_xB, boost::optional<gtsam::Matrix33 &> H_sB){
        // computes separation between points defined in local IMU frames A and B, in the nav frame. also returns optional jacobians.
        gtsam::Matrix36 dpA_dxA, dpB_dxB; gtsam::Matrix33 dpA_dsA, dpB_dsB;
        gtsam::Point3 pA=xA.transformFrom(sA,dpA_dxA,dpA_dsA);
        gtsam::Point3 pB=xB.transformFrom(sB,dpB_dxB,dpB_dsB);
        gtsam::Matrix33 de_dpA, de_dpB;
        gtsam::Point3 e=sub(pA,pB,de_dpA,de_dpB); // pA-pB
        if(H_xA){ *H_xA=de_dpA*dpA_dxA; }
        if(H_sA){ *H_sA=de_dpA*dpA_dsA; }
        if(H_xB){ *H_xB=de_dpB*dpB_dxB; }
        if(H_sB){ *H_sB=de_dpB*dpB_dsB; }
        return e;
    }

    double ptSeparationNorm(const gtsam::Pose3& xA, const gtsam::Point3& sA, const gtsam::Pose3& xB, const gtsam::Point3& sB, boost::optional<gtsam::Matrix16 &> H_xA, boost::optional<gtsam::Matrix13 &> H_sA, boost::optional<gtsam::Matrix16 &> H_xB, boost::optional<gtsam::Matrix13 &> H_sB){
        // computes norm of separation between points defined in local IMU frames A and B, in the nav frame. also returns optional jacobians.
        gtsam::Matrix36 de_dxA, de_dxB; gtsam::Matrix33 de_dsA, de_dsB;
        gtsam::Point3 e =ptSeparation(xA,sA,xB,sB,de_dxA,de_dsA,de_dxB,de_dsB);// point separation (vector)
        gtsam::Matrix13 dn_de;
        double n=gtsam::norm3(e,dn_de);
        if(H_xA){ *H_xA=dn_de*de_dxA; }
        if(H_sA){ *H_sA=dn_de*de_dsA; }
        if(H_xB){ *H_xB=dn_de*de_dxB; }
        if(H_sB){ *H_sB=dn_de*de_dsB; }
        return n;
    }

    std::vector<double> ptSeparationNorm(const std::vector<gtsam::Pose3>& xA, const gtsam::Point3& sA, const std::vector<gtsam::Pose3>& xB, const gtsam::Point3& sB){
        // vectorized version of function of the same name. for taking in a vector of poses.
        std::vector<double> n(xA.size());
        for(uint k=0; k<xA.size(); k++){
            n[k]=ptSeparationNorm(xA[k],sA,xB[k],sB);
        }
        return n;
    }

    gtsam::Vector3 projVecIntoPlane(const gtsam::Vector3& v, const gtsam::Vector3& n, boost::optional<gtsam::Matrix33 &> H_v, boost::optional<gtsam::Matrix33 &> H_n){
        // project vector v into plane whose normal is n, and optionally return derivative w.r.t. v and/or n
        // assumes v and n are in same coordinate frame
        gtsam::Matrix33 dt_dv, dt_dn;
        gtsam::Point3 t=gtsam::cross(v,n,dt_dv,dt_dn); // t = cross(n,v) => temp vector for cross product below
        gtsam::Matrix33 dh_dt ,dh_dn;
        gtsam::Point3 h=gtsam::cross(t,n,dh_dt,dh_dn); // h = cross(t,n) => the projection of v into the plane whose normal is n
        if(H_v){ *H_v=dh_dt*dt_dv; } // dh/dv
        if(H_n){ *H_n=dh_dn + dh_dt*dt_dn;} // dh/dn
        return h;
    }

    std::string distributionInfoString(const std::vector<double>& x) {
        // generates a string of information about a distribution for printing
        std::string info;
        try{
            info="mean="+std::to_string(mean(x))+", median="+std::to_string(median(x))+", std="+std::to_string(stdev(x))+", max="+std::to_string(max(x))+", min="+std::to_string(min(x));
        }catch(std::exception& e){
            info="error: could not print joint angle distribution. this is a known bug on some architectures."; //todo: fix this bug.
        }
        return info;
    }

    std::string distributionInfoString(const Eigen::VectorXd& x){
        return distributionInfoString(EigenVectorToStdVector(x));
    }

    uint findIdxOfNearestValInArray(const std::vector<double>& inputArray, double findVal){
        // take in a vector and find the closest value to findVal and return associated index
        std::vector<double> shiftedAbsValArray(inputArray.size());
        uint minIdx=999999; double foundMin=9.0e9;
        for(uint i=0;i<inputArray.size();i++){
            shiftedAbsValArray[i]=abs(inputArray[i]-findVal);
            if (shiftedAbsValArray[i]<foundMin){
                minIdx=i; foundMin=shiftedAbsValArray[i];
            }
        }
        return minIdx;
    }

    gtsam::Vector3 vectorFromPointToAxis(const gtsam::Vector3& p, const gtsam::Vector3& a, const gtsam::Unit3& k) {
        // you have an infinitely-long line defined by axis k and a point in 3D space p. This function computes the vector from p to the nearest point on the line defined by k.
        // assumes all quantities are in same coordinate frame
        /* from stackoverflow: https://stackoverflow.com/a/5227626
         *  p - point
            k - direction of line (unit length)
            a - point in line
            X - base of the perpendicular line
                p
               /|
              / |
             /  v
            a---X-----> k

            (p-a).k == |X-a|
            X == a + ((p-a).k)k
            Desired perpendicular: X-p = a + ((p-a).k)k - p
         */
        return a+((p-a).dot(k.unitVector()))*k - p;
    }

    void rotateImuPoseAboutPointAxisAngle(gtsam::Pose3& x0, const gtsam::Point3& rotPointNav, const gtsam::Vector3& rotVecNav, const double& rotAngle){
        // takes in a GTSAM IMU pose and rotates it about a point in the navigation frame specified by (point, axis, rotation angle)
        // remember: velocity will not change. Since velocity is specified (similar to position) as a vector in the nav frame, if the nav frame doesn't change the velocity will not change!
        // old IMU frame is denoted by B0, new is B1
        // todo: can you find the compact statement for the delta pose and just apply it directly?
        // --- old, long way for clarity --- //
        // () manually pull out old state
        gtsam::Point3 p0=x0.translation(); // this is a vector *from* the nav origin *to* the body frame center
        gtsam::Rot3 R_B0_to_N=x0.rotation(); // this is R[B->N]
        // () get R[B0->B1] and R[B1->N]
        gtsam::Vector3 rotVecB0=R_B0_to_N.inverse().rotate(rotVecNav);
        gtsam::Rot3 R_B0_to_B1=gtsam::Rot3::AxisAngle(gtsam::Unit3(rotVecB0),rotAngle);
        gtsam::Rot3 R_B1_to_N=R_B0_to_N*R_B0_to_B1.inverse(); // R[B1->N] = (R[B0->B1]*R[B0->N]')' = R[B0->N]*R[B0->B1]'
        // () figure out change in position
        //    remember that we have to do this rotation about the point *in the plane normal to rotVecNav*
        // find vector from rotation line -> p0. call that initial rotation radius a0.
        gtsam::Vector a0=-1.0*vectorFromPointToAxis(p0,rotPointNav,gtsam::Unit3(rotVecNav));
        gtsam::Point3 c0=p0-a0; // c is the intersection point on the line
        gtsam::Rot3 deltaRotNav=gtsam::Rot3::AxisAngle(gtsam::Unit3(rotVecNav),rotAngle);
        gtsam::Vector3 a1=deltaRotNav.rotate(a0); // rotated, but still from line to point. note that c0 should stay same--it's on line so it doesn't rotate.
        gtsam::Point3 p1=c0+a1; // new point is the rotation of the original radius about rotPointNav
        // () construct new pose, x1, and new velocity, v1
        gtsam::Pose3 x1(R_B1_to_N,p1);
        // ---------------------------------- //
        // () set variables and exit
        x0=x1;
    }

} // namespace mathutils

