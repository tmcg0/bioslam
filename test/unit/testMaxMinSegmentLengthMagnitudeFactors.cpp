// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// unit test of max and min constraints of the segment length magnitude factors (SegmentLengthMaxMagnitudeFactor and SegmentLengthMinMagnitudeFactor)

#include "testutils.h"
#include <gtsam/geometry/Pose3.h>
#include <factors/SegmentLengthMagnitudeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/base/numericalDerivative.h>

int test_derivative_numerically(const bioslam::SegmentLengthMaxMagnitudeFactor& fac, const gtsam::Point3 &v1, const gtsam::Point3 &v2);
int test_derivative_numerically(const bioslam::SegmentLengthMinMagnitudeFactor& fac, const gtsam::Point3 &v1, const gtsam::Point3 &v2);
int random_factor_tests(uint nTests=10000);
int solver_test(uint tests=100, double errorTol=5.0e-3);

int main(){
    random_factor_tests();
    solver_test();
}

int solver_test(uint tests, double errorTol){
    // note: this solver test really only tests the convergence of the normal anthro factor. adapt it to test max/min factors.
    // generate two random vectors, v1 and v2
    gtsam::SharedNoiseModel myNoiseModel=gtsam::noiseModel::Isotropic::Sigmas(gtsam::Vector1(1.0e-3));
    gtsam::Values myVals;
    gtsam::Key v1Key=gtsam::Symbol('a',0);
    gtsam::Key v2Key=gtsam::Symbol('b',0);
    double segLengthMin=5.0, segLengthMax=10.0;

    std::cout<<"--- testing SegmentLength[Max,Min]MagnitudeFactor. sigma="<<myNoiseModel->sigmas()<<" | seg length [max,min] = ["<<segLengthMax<<","<<segLengthMin<<"]"<<std::endl;
    // (note: when sigma=1, then optimizer error = 0.5*(factor error^2)
    for(uint i=0;i<tests;i++){
        gtsam::Unit3 u1=testutils::randomUnit3(), u2=testutils::randomUnit3();
        double l1=testutils::dRand(1.0, 15.0), l2=testutils::dRand(1.0, 15.0);
        gtsam::Point3 v1=u1.unitVector()*l1, v2=u2.unitVector()*l2;
        if(!myVals.exists(v1Key)){
            myVals.insert(v1Key,v1); myVals.insert(v2Key,v2);
        }else{
            myVals.update(v1Key,v1); myVals.update(v2Key,v2);
        }
        // setup graph and factors
        gtsam::NonlinearFactorGraph mygraph;
        bioslam::SegmentLengthMaxMagnitudeFactor testFacMax=bioslam::SegmentLengthMaxMagnitudeFactor(v1Key, v2Key, segLengthMax, myNoiseModel);
        bioslam::SegmentLengthMinMagnitudeFactor testFacMin=bioslam::SegmentLengthMinMagnitudeFactor(v1Key, v2Key, segLengthMin, myNoiseModel);
        mygraph.add(testFacMax); mygraph.add(testFacMin);
        // test derivative numerically
        test_derivative_numerically(testFacMax, v1, v2);
        test_derivative_numerically(testFacMin, v1, v2);
        gtsam::LevenbergMarquardtOptimizer optimizer(mygraph,myVals);
        double initialError=optimizer.error();
        double initialSegLength=(v1-v2).norm();
        gtsam::Values estimate=optimizer.optimize();
        double finalError=optimizer.error();
        gtsam::Point3 est_v1=estimate.at<gtsam::Point3>(v1Key);
        gtsam::Point3 est_v2=estimate.at<gtsam::Point3>(v2Key);
        double estSegLength=(est_v1-est_v2).norm();
        std::cout<<"initial: v1=["<<v1.transpose()<<"], v2=["<<v2.transpose()<<"] (seg length="<<initialSegLength<<") -->  ";
        std::cout<<"optimizec: v1=["<<est_v1.transpose()<<"], v2=["<<est_v2.transpose()<<"] (seg length="<<estSegLength<<"), optimizer error: "<<initialError<<" --> "<<finalError<<")  iterations="<<optimizer.iterations()<<std::endl;
        if((estSegLength-segLengthMax)>errorTol){ // if greater than 1% more than maxnorm, throw error
            throw std::runtime_error("error: estimated seg length is too large");
            return 1;
        }
        if((segLengthMin-estSegLength)>errorTol){
            std::cerr<<"error: estimated seg length is too small"<<std::endl;
            throw std::runtime_error("error: estimated seg length is too small");
            return 2;
        }
    }
    return 0;
}

int random_factor_tests(uint nTests){
    // in a loop, randomly generate inputs to the max factor and test derivatives numerically.
    gtsam::Vector velNoiseVec(1); velNoiseVec<<1.0;
    gtsam::SharedNoiseModel myNoiseModel=gtsam::noiseModel::Isotropic::Sigmas(velNoiseVec);
    gtsam::Values myVals;
    gtsam::NonlinearFactorGraph mygraph;
    gtsam::Key v1Key=gtsam::Symbol('a',0);
    gtsam::Key v2Key=gtsam::Symbol('b',0);
    // (note: when sigma=1, then optimizer error = 0.5*(factor error^2)
    for(uint i=0;i<nTests;i++) {
        gtsam::Point3 v1 = testutils::randomPoint3(), v2 = testutils::randomPoint3();
        double maxLength=abs(testutils::dRand()), minLength=abs(testutils::dRand())/10.0;
        if (!myVals.exists(v1Key)) {
            myVals.insert(v1Key, v1);
            myVals.insert(v2Key, v2);
        } else {
            myVals.update(v1Key, v1);
            myVals.update(v2Key, v2);
        }
        bioslam::SegmentLengthMaxMagnitudeFactor testFacMax=bioslam::SegmentLengthMaxMagnitudeFactor(v1Key, v2Key, maxLength, myNoiseModel);
        bioslam::SegmentLengthMinMagnitudeFactor testFacMin=bioslam::SegmentLengthMinMagnitudeFactor(v1Key, v2Key, minLength, myNoiseModel);
        // test derivative numerically
        test_derivative_numerically(testFacMax, v1, v2);
        test_derivative_numerically(testFacMin, v1, v2);
    }
    return 0;
}

int test_derivative_numerically(const bioslam::SegmentLengthMaxMagnitudeFactor& fac, const gtsam::Point3 &v1, const gtsam::Point3 &v2){
    // unit test the Jacobian against GTSAM's numerical derivative
    // this may seem silly for such a simple factor, but it's important to do.
    // () get derived error and jacobians
    gtsam::Matrix derivedH1, derivedH2;
    gtsam::Vector derivedErr=fac.evaluateError(v1,v2,derivedH1, derivedH2);
    // () get numerical jacobians
    //    I think to call it it's numericalDerivativeXY where X=number of input variables and Y=which Jacobian you want to test
    //    templates are: <output type (typically gtsam::Vector), then the input argument types in order)
    gtsam::Matrix numericalH1=gtsam::numericalDerivative21<gtsam::Vector,gtsam::Point3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Point3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::SegmentLengthMaxMagnitudeFactor::evaluateError,fac,std::placeholders::_1,std::placeholders::_2,boost::none,boost::none)),v1,v2,1e-5);
    gtsam::Matrix numericalH2=gtsam::numericalDerivative22<gtsam::Vector,gtsam::Point3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Point3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::SegmentLengthMaxMagnitudeFactor::evaluateError,fac,std::placeholders::_1,std::placeholders::_2,boost::none,boost::none)),v1,v2,1e-5);
    // now test using gtsam::assert_equal()
    bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-5);
    bool testH2=gtsam::assert_equal(derivedH2,numericalH2,1e-5);
    if (!testH1){
        std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
        throw std::runtime_error("jacobian numerical check failed.");
        return 1;
    }
    if (!testH2){
        std::cerr<<"H2 did not check out numerically."<<std::endl<<"derivedH2="<<derivedH2<<std::endl<<"numericalH2"<<numericalH2<<std::endl;
        throw std::runtime_error("jacobian numerical check failed.");
        return 1;
    }
    return 0;
}

int test_derivative_numerically(const bioslam::SegmentLengthMinMagnitudeFactor& fac, const gtsam::Point3 &v1, const gtsam::Point3 &v2){
    // unit test the Jacobian against GTSAM's numerical derivative
    // this may seem silly for such a simple factor, but it's important to do.
    // () get derived error and jacobians
    gtsam::Matrix derivedH1, derivedH2;
    gtsam::Vector derivedErr=fac.evaluateError(v1,v2,derivedH1, derivedH2);
    // () get numerical jacobians
    //    I think to call it it's numericalDerivativeXY where X=number of input variables and Y=which Jacobian you want to test
    //    templates are: <output type (typically gtsam::Vector), then the input argument types in order)
    gtsam::Matrix numericalH1=gtsam::numericalDerivative21<gtsam::Vector,gtsam::Point3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Point3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::SegmentLengthMinMagnitudeFactor::evaluateError,fac,std::placeholders::_1,std::placeholders::_2,boost::none,boost::none)),v1,v2,1e-5);
    gtsam::Matrix numericalH2=gtsam::numericalDerivative22<gtsam::Vector,gtsam::Point3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Point3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::SegmentLengthMinMagnitudeFactor::evaluateError,fac,std::placeholders::_1,std::placeholders::_2,boost::none,boost::none)),v1,v2,1e-5);
    // now test using gtsam::assert_equal()
    bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-7);
    bool testH2=gtsam::assert_equal(derivedH2,numericalH2,1e-7);
    if (!testH1){
        std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
        throw std::runtime_error("jacobian numerical check failed.");
        return 1;
    }
    if (!testH2){
        std::cerr<<"H2 did not check out numerically."<<std::endl<<"derivedH2="<<derivedH2<<std::endl<<"numericalH2"<<numericalH2<<std::endl;
        throw std::runtime_error("jacobian numerical check failed.");
        return 1;
    }
    return 0;
}