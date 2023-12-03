// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// unit test of SegmentLengthMagnitudeFactor

#include "testutils.h"
#include <gtsam/geometry/Pose3.h>
#include <factors/SegmentLengthMagnitudeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/base/numericalDerivative.h>

int test_derivative_numerically(const bioslam::SegmentLengthMagnitudeFactor& fac, const gtsam::Point3 &v1, const gtsam::Point3 &v2);

int main(){
    uint tests=50;
    // generate two random vectors, v1 and v2
    gtsam::Vector velNoiseVec(1); velNoiseVec<<1.0;
    gtsam::SharedNoiseModel myNoiseModel=gtsam::noiseModel::Isotropic::Sigmas(velNoiseVec);
    gtsam::Values myVals;
    gtsam::NonlinearFactorGraph mygraph;
    gtsam::Key v1Key=gtsam::Symbol('a',0);
    gtsam::Key v2Key=gtsam::Symbol('b',0);
    double segLengthMean=5.0;
    bioslam::SegmentLengthMagnitudeFactor testFac=bioslam::SegmentLengthMagnitudeFactor(v1Key, v2Key, segLengthMean, myNoiseModel);
    mygraph.add(testFac);

    std::cout<<"--- testing SegmentLengthMagnitudeFactor. sigma="<<velNoiseVec<<" | ideal segment length = "<<segLengthMean<<std::endl;
    // (note: when sigma=1, then optimizer error = 0.5*(factor error^2)
    for(uint i=0;i<tests;i++){
        gtsam::Point3 v1=testutils::randomPoint3(), v2=testutils::randomPoint3();
        if(!myVals.exists(v1Key)){
            myVals.insert(v1Key,v1); myVals.insert(v2Key,v2);
        }else{
            myVals.update(v1Key,v1); myVals.update(v2Key,v2);
        }
        // test derivative numerically
        test_derivative_numerically(testFac, v1, v2);
        gtsam::LevenbergMarquardtOptimizer optimizer(mygraph,myVals);
        double initialError=optimizer.error();
        double initialSegLength=(v1-v2).norm();
        gtsam::Values estimate=optimizer.optimize();
        double finalError=optimizer.error();
        gtsam::Point3 est_v1=estimate.at<gtsam::Point3>(v1Key);
        gtsam::Point3 est_v2=estimate.at<gtsam::Point3>(v2Key);
        double estSegLength=(est_v1-est_v2).norm();
        std::cout<<"initial: v1=["<<v1.transpose()<<"], v2=["<<v2.transpose()<<"] (seg length="<<initialSegLength<<") -->  ";
        std::cout<<"optimized: v1=["<<est_v1.transpose()<<"], v2=["<<est_v2.transpose()<<"] (seg length="<<estSegLength<<", target length = "<<segLengthMean<<", error = "<<abs(estSegLength-segLengthMean)<<"), optimizer error: "<<initialError<<" --> "<<finalError<<")  iterations="<<optimizer.iterations()<<std::endl;
        if(abs(segLengthMean-estSegLength)>segLengthMean*0.01){ // if greater than 10% more than maxnorm, throw error
            std::cerr<<"error: estimated seg length is greater than 1% larger than the target length"<<std::endl;
            return 1;
        }
    }
    return 0;
}

int test_derivative_numerically(const bioslam::SegmentLengthMagnitudeFactor& fac, const gtsam::Point3 &v1, const gtsam::Point3 &v2){
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
                    (std::bind(&bioslam::SegmentLengthMagnitudeFactor::evaluateError,fac,std::placeholders::_1,std::placeholders::_2,boost::none,boost::none)),v1,v2,1e-5);
    gtsam::Matrix numericalH2=gtsam::numericalDerivative22<gtsam::Vector,gtsam::Point3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Point3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::SegmentLengthMagnitudeFactor::evaluateError,fac,std::placeholders::_1,std::placeholders::_2,boost::none,boost::none)),v1,v2,1e-5);
    // now test using gtsam::assert_equal()
    bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-7);
    bool testH2=gtsam::assert_equal(derivedH2,numericalH2,1e-7);
    if (!testH1){
        std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
        return 1;
    }
    if (!testH2){
        std::cerr<<"H2 did not check out numerically."<<std::endl<<"derivedH2="<<derivedH2<<std::endl<<"numericalH2"<<numericalH2<<std::endl;
        return 1;
    }
    return 0;
}