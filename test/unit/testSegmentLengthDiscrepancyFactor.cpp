// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// unit test of SegmentLengthDiscrepancyFactor

#include "testutils.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/numericalDerivative.h>
#include <factors/SegmentLengthDiscrepancyFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

int test_derivative_numerically(const bioslam::SegmentLengthDiscrepancyFactor& fac,const gtsam::Point3& imuAToProximalJointCtr, const gtsam::Point3& imuAToDistalJointCtr, const gtsam::Point3& imuBToProximalJointCtr, const gtsam::Point3& imuBToDistalJointCtr);
int random_factor_tests(uint nTests=100);

int main(){
    random_factor_tests();
    return 0;
}

int random_factor_tests(uint nTests){
    // in a loop, randomly generate inputs to the max factor and test derivatives numerically.
    gtsam::SharedNoiseModel myNoiseModel=gtsam::noiseModel::Isotropic::Sigmas(gtsam::Vector1(1.0e-4));
    gtsam::Values myVals;
    gtsam::NonlinearFactorGraph mygraph;
    gtsam::Key imuAToProximalJointCtrKey=gtsam::Symbol('a',0);
    gtsam::Key imuAToDistalJointCtrKey=gtsam::Symbol('b',0);
    gtsam::Key imuBToProximalJointCtrKey=gtsam::Symbol('c',0);
    gtsam::Key imuBToDistalJointCtrKey=gtsam::Symbol('d',0);
    // (note: when sigma=1, then optimizer error = 0.5*(factor error^2)
    for(uint i=0;i<nTests;i++) {
        std::cout<<"------ test "<<i<<"/"<<nTests<<" ------"<<std::endl;
        // generate random vectors and then put in keys
        gtsam::Point3 imuAToProximalJointCtr=testutils::randomPoint3(), imuAToDistalJointCtr=testutils::randomPoint3(), imuBToProximalJointCtr=testutils::randomPoint3(), imuBToDistalJointCtr=testutils::randomPoint3();
        if (!myVals.exists(imuAToProximalJointCtrKey)) {
            myVals.insert(imuAToProximalJointCtrKey, imuAToProximalJointCtr);
            myVals.insert(imuAToDistalJointCtrKey, imuAToDistalJointCtr);
            myVals.insert(imuBToProximalJointCtrKey, imuBToProximalJointCtr);
            myVals.insert(imuBToDistalJointCtrKey, imuBToDistalJointCtr);
        } else {
            myVals.update(imuAToProximalJointCtrKey, imuAToProximalJointCtr);
            myVals.update(imuAToDistalJointCtrKey, imuAToDistalJointCtr);
            myVals.update(imuBToProximalJointCtrKey, imuBToProximalJointCtr);
            myVals.update(imuBToDistalJointCtrKey, imuBToDistalJointCtr);
        }
        bioslam::SegmentLengthDiscrepancyFactor testFac=bioslam::SegmentLengthDiscrepancyFactor(imuAToProximalJointCtrKey, imuAToDistalJointCtrKey, imuBToProximalJointCtrKey, imuBToDistalJointCtrKey, myNoiseModel);
        mygraph.add(testFac);
        // test derivative numerically
        test_derivative_numerically(testFac, imuAToProximalJointCtr, imuAToDistalJointCtr, imuBToProximalJointCtr, imuBToDistalJointCtr);
        // optimize
        gtsam::LevenbergMarquardtOptimizer optimizer(mygraph,myVals);
        gtsam::Values estimate=optimizer.optimize();
        gtsam::Point3 vAProx=estimate.at<gtsam::Point3>(imuAToProximalJointCtrKey);
        gtsam::Point3 vADist=estimate.at<gtsam::Point3>(imuAToDistalJointCtrKey);
        gtsam::Point3 vBProx=estimate.at<gtsam::Point3>(imuBToProximalJointCtrKey);
        gtsam::Point3 vBDist=estimate.at<gtsam::Point3>(imuBToDistalJointCtrKey);
        // calc discrepancy
        double discrepancy=(vAProx-vADist).norm()-(vBProx-vBDist).norm();
        std::cout<<"    discrepancy="<<discrepancy<<", final optimizer error = "<<optimizer.error()<<std::endl;
        std::cout<<"    vAProx=["<<vAProx.transpose()<<"], vADist=["<<vADist.transpose()<<"]"<<std::endl;
        std::cout<<"    vBProx=["<<vBProx.transpose()<<"], vBDist=["<<vBDist.transpose()<<"]"<<std::endl;
        // test
        if(discrepancy>1.0e-6){
            throw std::runtime_error("discrepancy is too large! discprenacy="+std::to_string(discrepancy));
        }
        if(optimizer.error()>1.0e-12){
            throw std::runtime_error("global converged error is too large. error = "+std::to_string(optimizer.error()));
        }
    }
    return 0;
}

int test_derivative_numerically(const bioslam::SegmentLengthDiscrepancyFactor& fac,const gtsam::Point3& imuAToProximalJointCtr, const gtsam::Point3& imuAToDistalJointCtr, const gtsam::Point3& imuBToProximalJointCtr, const gtsam::Point3& imuBToDistalJointCtr){
    // unit test the Jacobian against GTSAM's numerical derivative
    // this may seem silly for such a simple factor, but it's important to do.
    // () get derived error and jacobians
    gtsam::Matrix derivedH1, derivedH2, derivedH3, derivedH4;
    gtsam::Vector derivedErr=fac.evaluateError(imuAToProximalJointCtr,imuAToDistalJointCtr, imuBToProximalJointCtr, imuBToDistalJointCtr,derivedH1, derivedH2, derivedH3, derivedH4);
    // () get numerical jacobians
    //    I think to call it it's numericalDerivativeXY where X=number of input variables and Y=which Jacobian you want to test
    //    templates are: <output type (typically gtsam::Vector), then the input argument types in order)
    gtsam::Matrix numericalH1=gtsam::numericalDerivative41<gtsam::Vector,gtsam::Point3,gtsam::Point3,gtsam::Point3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Point3&,const gtsam::Point3&,const gtsam::Point3&,const gtsam::Point3&)>
                    (std::bind(&bioslam::SegmentLengthDiscrepancyFactor::evaluateError,fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,boost::none,boost::none,boost::none,boost::none)),imuAToProximalJointCtr,imuAToDistalJointCtr, imuBToProximalJointCtr, imuBToDistalJointCtr,1e-5);
    gtsam::Matrix numericalH2=gtsam::numericalDerivative42<gtsam::Vector,gtsam::Point3,gtsam::Point3,gtsam::Point3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Point3&,const gtsam::Point3&,const gtsam::Point3&,const gtsam::Point3&)>
                    (std::bind(&bioslam::SegmentLengthDiscrepancyFactor::evaluateError,fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,boost::none,boost::none,boost::none,boost::none)),imuAToProximalJointCtr,imuAToDistalJointCtr, imuBToProximalJointCtr, imuBToDistalJointCtr,1e-5);
    gtsam::Matrix numericalH3=gtsam::numericalDerivative43<gtsam::Vector,gtsam::Point3,gtsam::Point3,gtsam::Point3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Point3&,const gtsam::Point3&,const gtsam::Point3&,const gtsam::Point3&)>
                    (std::bind(&bioslam::SegmentLengthDiscrepancyFactor::evaluateError,fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,boost::none,boost::none,boost::none,boost::none)),imuAToProximalJointCtr,imuAToDistalJointCtr, imuBToProximalJointCtr, imuBToDistalJointCtr,1e-5);
    gtsam::Matrix numericalH4=gtsam::numericalDerivative44<gtsam::Vector,gtsam::Point3,gtsam::Point3,gtsam::Point3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Point3&,const gtsam::Point3&,const gtsam::Point3&,const gtsam::Point3&)>
                    (std::bind(&bioslam::SegmentLengthDiscrepancyFactor::evaluateError,fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,boost::none,boost::none,boost::none,boost::none)),imuAToProximalJointCtr,imuAToDistalJointCtr, imuBToProximalJointCtr, imuBToDistalJointCtr,1e-5);

    // now test using gtsam::assert_equal()
    bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-7);
    bool testH2=gtsam::assert_equal(derivedH2,numericalH2,1e-7);
    bool testH3=gtsam::assert_equal(derivedH3,numericalH3,1e-7);
    bool testH4=gtsam::assert_equal(derivedH4,numericalH4,1e-7);
    if (!testH1){
        std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
        return 1;
    }
    if (!testH2){
        std::cerr<<"H2 did not check out numerically."<<std::endl<<"derivedH2="<<derivedH2<<std::endl<<"numericalH2"<<numericalH2<<std::endl;
        return 1;
    }
    if (!testH3){
        std::cerr<<"H3 did not check out numerically."<<std::endl<<"derivedH3="<<derivedH3<<std::endl<<"numericalH3"<<numericalH3<<std::endl;
        return 1;
    }
    if (!testH4){
        std::cerr<<"H4 did not check out numerically."<<std::endl<<"derivedH4="<<derivedH4<<std::endl<<"numericalH4"<<numericalH4<<std::endl;
        return 1;
    }
    return 0;
}
