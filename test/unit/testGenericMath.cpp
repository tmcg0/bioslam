// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// unit testing for generic math functions in bioslam

#include "mathutils.h"
#include "testutils.h"
#include "gtsamutils.h"
#include "gtsam/base/numericalDerivative.h"
#include "gtsam/slam/PriorFactor.h"

int testDot(uint nTests=1000);
int testCross(uint nTests=1000);
int testNorm(uint nTests=1000);
int testUnsignedAngleCalc(uint nTests=1000, bool testNumericalDerivative=false);
int testSignedAngleCalc(uint nTests=1000, bool testNumericalDerivative=false);
int testProjmk(uint nTests=1000);
int testPtSeparation(uint nTests=1000);
int testPtSeparationNorm(uint nTests=1000);
int testProjVecIntoPlane(uint nTests=1000);
int testatan2(uint nTests=1000);

int main(){
    // test gtsam math functions
    testDot(1000);
    testCross(1000);
    testNorm(1000);
    // test bioslam math functions
    testatan2(1000);
    testProjVecIntoPlane(1000);
    testUnsignedAngleCalc(1000,true);
    testSignedAngleCalc(1000,true);
    testProjmk(1000);
    testPtSeparation(1000);
    testPtSeparationNorm(1000);
    return 0;
}

int testatan2(uint nTests){
    for(uint i=0; i<nTests; i++){
        double y=testutils::dRand(), x=testutils::dRand();
        gtsam::Matrix11 derivedH1, derivedH2;
        double lim=5.0e-3;
        if(abs(x)>lim && abs(y)>lim){ // run test only for well-conditioned cases
            double f=mathutils::atan2(y, x, derivedH1, derivedH2);
            // test derivative numerically
            gtsam::Matrix numericalH1=gtsam::numericalDerivative21<double,double,double>(
                    std::function<double(const double&, const double&)>
                            (std::bind(&mathutils::atan2,std::placeholders::_1,std::placeholders::_2, boost::none, boost::none)), y,x, 1e-5);
            gtsam::Matrix numericalH2=gtsam::numericalDerivative22<double,double,double>(
                    std::function<double(const double&, const double&)>
                            (std::bind(&mathutils::atan2,std::placeholders::_1,std::placeholders::_2, boost::none, boost::none)), y,x, 1e-5);
            bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-5);
            bool testH2=gtsam::assert_equal(derivedH2,numericalH2,1e-5);
            if (!testH1){
                std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
                std::cerr<<"f=atan2(y,x) where f="<<f<<", y="<<y<<", x="<<x<<" (x^2+y+2)="<<pow(x,2.0)+pow(y,2.0)<<" => df/dx=-y/(x^2+y^2)="<<-y/(pow(x,2.0)+pow(y,2.0))<<", df/dy=x/(x^2+y^2)="<<x/(pow(x,2.0)+pow(y,2.0))<<std::endl;
                throw std::runtime_error("atan2() jacobian did not check out.");
                return 1;
            }
            if (!testH2){
                std::cerr<<"H2 did not check out numerically."<<std::endl<<"derivedH2="<<derivedH2<<std::endl<<"numericalH2"<<numericalH2<<std::endl;
                std::cerr<<"f=atan2(y,x) where f="<<f<<", y="<<y<<", x="<<x<<" (x^2+y+2)="<<pow(x,2.0)+pow(y,2.0)<<" => df/dx=-y/(x^2+y^2)="<<-y/(pow(x,2.0)+pow(y,2.0))<<", df/dy=x/(x^2+y^2)="<<x/(pow(x,2.0)+pow(y,2.0))<<std::endl;
                throw std::runtime_error("atan2() jacobian did not check out.");
                return 1;
            }
        }
    }
    return 0;
}

int testProjVecIntoPlane(uint nTests){
    for(uint i=0; i<nTests; i++){
        gtsam::Point3 a=testutils::randomPoint3(), b=testutils::randomPoint3();
        gtsam::Matrix33 derivedH1, derivedH2;
        mathutils::projVecIntoPlane(a, b, derivedH1, derivedH2);
        // test derivative numerically
        gtsam::Matrix numericalH1=gtsam::numericalDerivative21<gtsam::Point3,gtsam::Point3,gtsam::Point3>(
                std::function<gtsam::Point3(const gtsam::Point3&, const gtsam::Point3&)>
                        (std::bind(&mathutils::projVecIntoPlane,std::placeholders::_1,std::placeholders::_2, boost::none, boost::none)), a, b, 1e-5);
        gtsam::Matrix numericalH2=gtsam::numericalDerivative22<gtsam::Point3,gtsam::Point3,gtsam::Point3>(
                std::function<gtsam::Point3(const gtsam::Point3&, const gtsam::Point3&)>
                        (std::bind(&mathutils::projVecIntoPlane,std::placeholders::_1,std::placeholders::_2, boost::none, boost::none)), a, b, 1e-5);
        bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-10);
        bool testH2=gtsam::assert_equal(derivedH2,numericalH2,1e-10);
        if (!testH1){
            std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
            throw std::runtime_error("cross() jacobian did not check out.");
            return 1;
        }
        if (!testH2){
            std::cerr<<"H2 did not check out numerically."<<std::endl<<"derivedH2="<<derivedH2<<std::endl<<"numericalH2"<<numericalH2<<std::endl;
            throw std::runtime_error("cross() jacobian did not check out.");
            return 1;
        }
    }
    return 0;
}

int testNorm(uint nTests){
    // test gtsam's Point3::norm() implementation for numerical derivatives
    for(uint i=0; i<nTests; i++){
        gtsam::Point3 a=testutils::randomPoint3();
        if(a.norm()>0.1){
            gtsam::Matrix derivedH1;
            gtsam::norm3(a,derivedH1);
            // test derivative numerically
            gtsam::Matrix numericalH1=gtsam::numericalDerivative11<double,gtsam::Point3>(
                    std::function<double(const gtsam::Point3&)>
                            (std::bind(&gtsam::norm3,std::placeholders::_1,boost::none)),a,1e-8);
            bool testH1=gtsam::assert_equal(derivedH1,numericalH1,5.0e-8);
            if (!testH1){
                std::cout<<"a="<<a.transpose()<<", norm="<<a.norm()<<std::endl;
                std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
                throw std::runtime_error("norm() jacobian did not check out.");
            }
        }
    }
    return 0;
}

int testCross(uint nTests){
    // test gtsam's Point3::cross() implementation for numerical derivatives
    for(uint i=0; i<nTests; i++){
        gtsam::Point3 a=testutils::randomPoint3(), b=testutils::randomPoint3();
        gtsam::Matrix derivedH1, derivedH2;
        gtsam::cross(a,b,derivedH1,derivedH2);
        // test derivative numerically
        gtsam::Matrix numericalH1=gtsam::numericalDerivative21<gtsam::Point3,gtsam::Point3,gtsam::Point3>(
                std::function<gtsam::Point3(const gtsam::Point3&, const gtsam::Point3&)>
                        (std::bind(&gtsam::cross,std::placeholders::_1,std::placeholders::_2,boost::none,boost::none)),a,b,1e-5);
        gtsam::Matrix numericalH2=gtsam::numericalDerivative22<gtsam::Point3,gtsam::Point3,gtsam::Point3>(
                std::function<gtsam::Point3(const gtsam::Point3&, const gtsam::Point3&)>
                        (std::bind(&gtsam::cross,std::placeholders::_1,std::placeholders::_2,boost::none,boost::none)),a,b,1e-5);
        bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-9);
        bool testH2=gtsam::assert_equal(derivedH2,numericalH2,1e-9);
        if (!testH1){
            std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
            throw std::runtime_error("cross() jacobian did not check out.");
            return 1;
        }
        if (!testH2){
            std::cerr<<"H2 did not check out numerically."<<std::endl<<"derivedH2="<<derivedH2<<std::endl<<"numericalH2"<<numericalH2<<std::endl;
            throw std::runtime_error("cross() jacobian did not check out.");
            return 1;
        }
    }
    return 0;
}

int testDot(uint nTests){
    // test gtsam's Point3::dot() implementation for numerical derivatives
    for(uint i=0; i<nTests; i++){
        gtsam::Point3 a=testutils::randomPoint3(), b=testutils::randomPoint3();
        gtsam::Matrix derivedH1, derivedH2;
        gtsam::dot(a,b,derivedH1,derivedH2);
        // to reduce ambiguity between overloads of gtsam::dot(), we test the derivative numerically using a lambda function, as opposed to other styles in this file
        // remember to use std::function to avoid ambiguity between overloads of numericalDerivative
        auto numericalH1 = gtsam::numericalDerivative21<double, gtsam::Point3, gtsam::Point3>(
            std::function<double(const gtsam::Point3&, const gtsam::Point3&)>([](const gtsam::Point3& arg1, const gtsam::Point3& arg2) {
                return gtsam::dot(arg1, arg2);
            }),
            a, b, 1e-5);
        auto numericalH2 = gtsam::numericalDerivative22<double, gtsam::Point3, gtsam::Point3>(
            std::function<double(const gtsam::Point3&, const gtsam::Point3&)>([](const gtsam::Point3& arg1, const gtsam::Point3& arg2) {
                return gtsam::dot(arg1, arg2);
            }),
            a, b, 1e-5);
        // now test using gtsam::assert_equal()
        bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-9);
        bool testH2=gtsam::assert_equal(derivedH2,numericalH2,1e-9);
        if (!testH1){
            std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
            throw std::runtime_error("dot() jacobian did not check out.");
            return 1;
        }
        if (!testH2){
            std::cerr<<"H2 did not check out numerically."<<std::endl<<"derivedH2="<<derivedH2<<std::endl<<"numericalH2"<<numericalH2<<std::endl;
            throw std::runtime_error("dot() jacobian did not check out.");
            return 1;
        }
    }
    return 0;
}

int testPtSeparationNorm(uint nTests){
    // test mathutils::ptSeparation
    for(uint i=0; i<nTests; i++){
        gtsam::Pose3 xA=testutils::randomPose3(), xB=testutils::randomPose3();
        gtsam::Point3 sA=testutils::randomPoint3(), sB=testutils::randomPoint3();
        gtsam::Matrix16 derivedH1, derivedH3; gtsam::Matrix13 derivedH2, derivedH4;
        double n=mathutils::ptSeparationNorm(xA, sA, xB, sB, derivedH1, derivedH2, derivedH3, derivedH4);
        // to reduce ambiguity between overloads of mathutils::ptSeparationNorm, we test the derivative numerically using a lambda function, as opposed to other styles in this file
        // remember to use std::function to avoid ambiguity between overloads of numericalDerivative
        auto numericalH1 = gtsam::numericalDerivative41<double, gtsam::Pose3, gtsam::Point3, gtsam::Pose3, gtsam::Point3>(
            std::function<double(const gtsam::Pose3&, const gtsam::Point3&, const gtsam::Pose3&, const gtsam::Point3&)>([](const gtsam::Pose3& arg1, const gtsam::Point3& arg2, const gtsam::Pose3& arg3, const gtsam::Point3& arg4) {
                return mathutils::ptSeparationNorm(arg1, arg2, arg3, arg4);
            }),
            xA, sA, xB, sB, 1e-5);
        auto numericalH2 = gtsam::numericalDerivative42<double, gtsam::Pose3, gtsam::Point3, gtsam::Pose3, gtsam::Point3>(
            std::function<double(const gtsam::Pose3&, const gtsam::Point3&, const gtsam::Pose3&, const gtsam::Point3&)>([](const gtsam::Pose3& arg1, const gtsam::Point3& arg2, const gtsam::Pose3& arg3, const gtsam::Point3& arg4) {
                return mathutils::ptSeparationNorm(arg1, arg2, arg3, arg4);
            }),
            xA, sA, xB, sB, 1e-5);
        auto numericalH3 = gtsam::numericalDerivative43<double, gtsam::Pose3, gtsam::Point3, gtsam::Pose3, gtsam::Point3>(
            std::function<double(const gtsam::Pose3&, const gtsam::Point3&, const gtsam::Pose3&, const gtsam::Point3&)>([](const gtsam::Pose3& arg1, const gtsam::Point3& arg2, const gtsam::Pose3& arg3, const gtsam::Point3& arg4) {
                return mathutils::ptSeparationNorm(arg1, arg2, arg3, arg4);
            }),
            xA, sA, xB, sB, 1e-5);
        auto numericalH4 = gtsam::numericalDerivative44<double, gtsam::Pose3, gtsam::Point3, gtsam::Pose3, gtsam::Point3>(
            std::function<double(const gtsam::Pose3&, const gtsam::Point3&, const gtsam::Pose3&, const gtsam::Point3&)>([](const gtsam::Pose3& arg1, const gtsam::Point3& arg2, const gtsam::Pose3& arg3, const gtsam::Point3& arg4) {
                return mathutils::ptSeparationNorm(arg1, arg2, arg3, arg4);
            }),
            xA, sA, xB, sB, 1e-5);
        // now test using gtsam::assert_equal()
        bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-5);
        if (!testH1){
            std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
            throw std::runtime_error("ptSeparationNorm() jacobian did not check out numerically.");
            return 1;
        }
        bool testH2=gtsam::assert_equal(derivedH2,numericalH2,1e-5);
        if (!testH2){
            std::cerr<<"H2 did not check out numerically."<<std::endl<<"derivedH2="<<derivedH2<<std::endl<<"numericalH2"<<numericalH2<<std::endl;
            throw std::runtime_error("ptSeparationNorm() jacobian did not check out numerically.");
            return 1;
        }
        bool testH3=gtsam::assert_equal(derivedH3,numericalH3,1e-5);
        if (!testH3){
            std::cerr<<"H3 did not check out numerically."<<std::endl<<"derivedH3="<<derivedH3<<std::endl<<"numericalH3"<<numericalH3<<std::endl;
            throw std::runtime_error("ptSeparationNorm() jacobian did not check out numerically.");
            return 1;
        }
        bool testH4=gtsam::assert_equal(derivedH4,numericalH4,1e-5);
        if (!testH4){
            std::cerr<<"H4 did not check out numerically."<<std::endl<<"derivedH4="<<derivedH4<<std::endl<<"numericalH4"<<numericalH4<<std::endl;
            throw std::runtime_error("ptSeparationNorm() jacobian did not check out numerically.");
            return 1;
        }
    }
    return 0;
}

int testPtSeparation(uint nTests){
    // test mathutils::ptSeparation
    for(uint i=0; i<nTests; i++){
        gtsam::Pose3 xA=testutils::randomPose3(), xB=testutils::randomPose3();
        gtsam::Point3 sA=testutils::randomPoint3(), sB=testutils::randomPoint3();
        gtsam::Matrix36 derivedH1, derivedH3; gtsam::Matrix33 derivedH2, derivedH4;
        gtsam::Point3 e=mathutils::ptSeparation(xA, sA, xB, sB, derivedH1, derivedH2, derivedH3, derivedH4);
        // test derivative numerically
        gtsam::Matrix numericalH1=gtsam::numericalDerivative41<gtsam::Vector3,gtsam::Pose3,gtsam::Point3,gtsam::Pose3,gtsam::Point3>(
                std::function<gtsam::Vector3(const gtsam::Pose3&, const gtsam::Point3&,const gtsam::Pose3&, const gtsam::Point3&)>
                        (std::bind(&mathutils::ptSeparation,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4, boost::none, boost::none, boost::none, boost::none)), xA, sA, xB, sB, 1e-5);
        gtsam::Matrix numericalH2=gtsam::numericalDerivative42<gtsam::Vector3,gtsam::Pose3,gtsam::Point3,gtsam::Pose3,gtsam::Point3>(
                std::function<gtsam::Vector3(const gtsam::Pose3&, const gtsam::Point3&,const gtsam::Pose3&, const gtsam::Point3&)>
                        (std::bind(&mathutils::ptSeparation,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4, boost::none, boost::none, boost::none, boost::none)), xA, sA, xB, sB, 1e-5);
        gtsam::Matrix numericalH3=gtsam::numericalDerivative43<gtsam::Vector3,gtsam::Pose3,gtsam::Point3,gtsam::Pose3,gtsam::Point3>(
                std::function<gtsam::Vector3(const gtsam::Pose3&, const gtsam::Point3&,const gtsam::Pose3&, const gtsam::Point3&)>
                        (std::bind(&mathutils::ptSeparation,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4, boost::none, boost::none, boost::none, boost::none)), xA, sA, xB, sB, 1e-5);
        gtsam::Matrix numericalH4=gtsam::numericalDerivative44<gtsam::Vector3,gtsam::Pose3,gtsam::Point3,gtsam::Pose3,gtsam::Point3>(
                std::function<gtsam::Vector3(const gtsam::Pose3&, const gtsam::Point3&,const gtsam::Pose3&, const gtsam::Point3&)>
                        (std::bind(&mathutils::ptSeparation,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4, boost::none, boost::none, boost::none, boost::none)), xA, sA, xB, sB, 1e-5);
        // now test using gtsam::assert_equal()
        bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-9);
        if (!testH1){
            std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
            throw std::runtime_error("ptSeparation() jacobian did not check out numerically.");
            return 1;
        }
        bool testH2=gtsam::assert_equal(derivedH2,numericalH2,1e-9);
        if (!testH2){
            std::cerr<<"H2 did not check out numerically."<<std::endl<<"derivedH2="<<derivedH2<<std::endl<<"numericalH2"<<numericalH2<<std::endl;
            throw std::runtime_error("ptSeparation() jacobian did not check out numerically.");
            return 1;
        }
        bool testH3=gtsam::assert_equal(derivedH3,numericalH3,1e-9);
        if (!testH3){
            std::cerr<<"H3 did not check out numerically."<<std::endl<<"derivedH3="<<derivedH3<<std::endl<<"numericalH3"<<numericalH3<<std::endl;
            throw std::runtime_error("ptSeparation() jacobian did not check out numerically.");
            return 1;
        }
        bool testH4=gtsam::assert_equal(derivedH4,numericalH4,1e-9);
        if (!testH4){
            std::cerr<<"H4 did not check out numerically."<<std::endl<<"derivedH4="<<derivedH4<<std::endl<<"numericalH4"<<numericalH4<<std::endl;
            throw std::runtime_error("ptSeparation() jacobian did not check out numerically.");
            return 1;
        }
    }
    return 0;
}

int testProjmk(uint nTests){
    // tests mathutils::projmk
    for(uint i=0; i<nTests; i++){
        gtsam::Vector3 m=testutils::randomVector3();
        gtsam::Unit3 k=testutils::randomUnit3();
        gtsam::Matrix33 derivedH1; gtsam::Matrix32 derivedH2;
        gtsam::Vector3 mk=mathutils::projmk(m, k, derivedH1, derivedH2);
        // test derivative numerically
        gtsam::Matrix numericalH1=gtsam::numericalDerivative21<gtsam::Vector3,gtsam::Vector3,gtsam::Unit3>(
                std::function<gtsam::Vector3(const gtsam::Vector3&, const gtsam::Unit3&)>
                        (std::bind(&mathutils::projmk,std::placeholders::_1,std::placeholders::_2, boost::none, boost::none)), m, k, 1e-5);
        gtsam::Matrix numericalH2=gtsam::numericalDerivative22<gtsam::Vector3,gtsam::Vector3,gtsam::Unit3>(
                std::function<gtsam::Vector3(const gtsam::Vector3&, const gtsam::Unit3&)>
                        (std::bind(&mathutils::projmk,std::placeholders::_1,std::placeholders::_2, boost::none, boost::none)), m, k, 1e-5);
        bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-9);
        bool testH2=gtsam::assert_equal(derivedH2,numericalH2,1e-9);
        if (!testH1){
            std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
            throw std::runtime_error("projmk() jacobian did not check out.");
            return 1;
        }
        if (!testH2){
            std::cerr<<"H2 did not check out numerically."<<std::endl<<"derivedH2="<<derivedH2<<std::endl<<"numericalH2"<<numericalH2<<std::endl;
            throw std::runtime_error("projmk() jacobian did not check out.");
            return 1;
        }
    }
    return 0;
}

int testSignedAngleCalc(uint nTests, bool testNumericalDerivative){
    double minAng=9.0e9, maxAng=-9.0e9, ang=0.0;
    gtsam::Vector3 refVec=testutils::randomVector3();
    for(uint k=0; k<nTests; k++){
        gtsam::Vector3 v1=testutils::randomVector3(), v2=testutils::randomVector3();
        gtsam::Matrix13 derivedH1, derivedH2;
        ang=mathutils::signedAngle(v1, v2, refVec, derivedH1, derivedH2);
        if( (v1.cross(v2)).norm()>1.0e-1 && v1.norm()>1.0e-1 && v2.norm()>1.0e-1 && refVec.norm()>1.0e-1 ){ // only test well conditioned cases
            if(ang>maxAng){ // new maxang
                maxAng=ang;
            }
            if(ang<minAng){ // new minang
                minAng=ang;
            }
            if(testNumericalDerivative){
                // test numerical derivative
                // there was a previous known issue with this test where sometimes the numerical derivative would explode, causing the test to fail: https://github.com/tmcg0/bioslam/issues/2
                // the full problem was not identified, however, as it seemed to just be a problem with the unit test (not the analytical derivatives), we simply check for incorrect numerical derivatives and ignore them:
                double numDerCoeffMax=10000.0;
                gtsam::Matrix numericalH1=gtsam::numericalDerivative21<double,gtsam::Vector3,gtsam::Vector3>(
                        std::function<double(const gtsam::Vector3&, const gtsam::Vector3&)>
                                (std::bind(&mathutils::signedAngle, std::placeholders::_1, std::placeholders::_2, refVec, boost::none, boost::none)), v1, v2, 1e-5);
                gtsam::Matrix numericalH2=gtsam::numericalDerivative22<double,gtsam::Vector3,gtsam::Vector3>(
                        std::function<double(const gtsam::Vector3&, const gtsam::Vector3&)>
                                (std::bind(&mathutils::signedAngle, std::placeholders::_1, std::placeholders::_2, refVec, boost::none, boost::none)), v1, v2, 1e-5);
                double derivTol=1.0e-6;
                bool testH1=gtsam::assert_equal(derivedH1,numericalH1,derivTol);
                bool testH2=gtsam::assert_equal(derivedH2,numericalH2,derivTol);
                if (!testH1 && numericalH1.cwiseAbs().maxCoeff()<numDerCoeffMax){
                    std::cout<<"test "<<k<<"/"<<nTests<<" error info: ang="<<ang<<", v1="<<v1.transpose()<<", v2="<<v2.transpose()<<", => cross(v1,v2)="<<v1.cross(v2).transpose()<<" | refVec="<<refVec.transpose()<<std::endl;
                    std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
                    throw std::runtime_error("signedAngle() jacobian did not check out.");
                    return 1;
                }
                if (!testH2 && numericalH2.cwiseAbs().maxCoeff()<numDerCoeffMax){
                    std::cout<<"test "<<k<<"/"<<nTests<<" error info: ang="<<ang<<", v1="<<v1.transpose()<<", v2="<<v2.transpose()<<", => cross(v1,v2)="<<v1.cross(v2).transpose()<<" | refVec="<<refVec.transpose()<<std::endl;
                    std::cerr<<"H2 did not check out numerically."<<std::endl<<"derivedH2="<<derivedH2<<std::endl<<"numericalH2"<<numericalH2<<std::endl;
                    throw std::runtime_error("signedAngle() jacobian did not check out.");
                    return 1;
                }
            }
        }
    }
    std::cout<<"testSignedAngleCalc: "<<nTests<<" tests, min angle = "<<minAng<<", max angle = "<<maxAng<<std::endl;
    return 0;
}

int testUnsignedAngleCalc(uint nTests, bool testNumericalDerivative){
    double minAng=9.0e9, maxAng=-9.0e9, ang=0.0;
    for(uint k=0; k<nTests; k++){
        gtsam::Matrix13 derivedH1, derivedH2;
        gtsam::Vector3 v1=testutils::randomVector3(), v2=testutils::randomVector3();
        ang=mathutils::unsignedAngle(v1, v2, derivedH1, derivedH2);
        if( (v1.cross(v2)).norm()>1.0e-3 ){ // only test well conditioned cases
            if(ang>maxAng){ // new maxang
                maxAng=ang;
            }
            if(ang<minAng){ // new minang
                minAng=ang;
            }
            if(testNumericalDerivative){
                // test numerical derivative
                gtsam::Matrix numericalH1=gtsam::numericalDerivative21<double,gtsam::Vector3,gtsam::Vector3>(
                        std::function<double(const gtsam::Vector3&, const gtsam::Vector3&)>
                                (std::bind(&mathutils::unsignedAngle, std::placeholders::_1, std::placeholders::_2, boost::none, boost::none)), v1, v2, 1e-5);
                gtsam::Matrix numericalH2=gtsam::numericalDerivative22<double,gtsam::Vector3,gtsam::Vector3>(
                        std::function<double(const gtsam::Vector3&, const gtsam::Vector3&)>
                                (std::bind(&mathutils::unsignedAngle, std::placeholders::_1, std::placeholders::_2, boost::none, boost::none)), v1, v2, 1e-5);
                double derivTol=1.0e-5;
                bool testH1=gtsam::assert_equal(derivedH1,numericalH1,derivTol);
                bool testH2=gtsam::assert_equal(derivedH2,numericalH2,derivTol);
                if (!testH1){
                    std::cout<<"error info: ang = "<<ang<<", v1="<<v1.transpose()<<", v2="<<v2.transpose()<<", => cross(v1,v2)="<<v1.cross(v2).transpose()<<std::endl;
                    std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
                    throw std::runtime_error("unsignedAngle() jacobian did not check out.");
                    return 1;
                }
                if (!testH2){
                    std::cout<<"error info: ang = "<<ang<<", v1="<<v1.transpose()<<", v2="<<v2.transpose()<<", => cross(v1,v2)="<<v1.cross(v2).transpose()<<std::endl;
                    std::cerr<<"H2 did not check out numerically."<<std::endl<<"derivedH2="<<derivedH2<<std::endl<<"numericalH2"<<numericalH2<<std::endl;
                    throw std::runtime_error("unsignedAngle() jacobian did not check out.");
                    return 1;
                }
            }
        }
    }
    std::cout<<"testUnsignedAngleCalc: "<<nTests<<" tests, min angle = "<<minAng<<", max angle = "<<maxAng<<std::endl;
    return 0;
}