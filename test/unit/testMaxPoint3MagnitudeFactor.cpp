// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// unit test of MaxPoint3MagnitudeFactor

#include <factors/Point3Priors.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/numericalDerivative.h>
#include <random>
#include "testutils.h"

int test_derivative_numerically(const bioslam::MaxPoint3MagnitudeFactor& fac, const gtsam::Point3 &p);

int main(){
    // generate a bunch of random Point3's. run them through an optimizer and make sure that their mangitude comes out to whatever you set the limit is in the factor header file.
    uint tests=50;
    double maxNorm=0.5;
    gtsam::Vector velNoiseVec(1); velNoiseVec<<1.0;
    gtsam::SharedNoiseModel myNoiseModel=gtsam::noiseModel::Isotropic::Sigmas(velNoiseVec);
    gtsam::Values myVals;
    gtsam::NonlinearFactorGraph mygraph;
    gtsam::Key mykey=gtsam::Symbol('a',0);
    bioslam::MaxPoint3MagnitudeFactor testfac=bioslam::MaxPoint3MagnitudeFactor(mykey, maxNorm, myNoiseModel);
    mygraph.add(testfac);
    for(uint i=0;i<tests;i++){
        gtsam::Point3 testPoint=testutils::randomPoint3();
        if(!myVals.exists(mykey)){
            myVals.insert(mykey,testPoint);
        }else{
            myVals.update(mykey,testPoint);
        }
        testutils::runtime_assert(test_derivative_numerically(testfac,testPoint)==0);
        gtsam::LevenbergMarquardtOptimizer optimizer(mygraph,myVals);
        gtsam::Values estimate=optimizer.optimize();
        gtsam::Point3 optimizedPoint=estimate.at<gtsam::Point3>(mykey);
        std::cout<<"initial Point3: ["<<testPoint.transpose()<<"] (norm="<<testPoint.norm()<<")  -->  ";
        std::cout<<"optimized Point3:"<<optimizedPoint.transpose()<<" (norm="<<optimizedPoint.norm()<<"). iterations="<<optimizer.iterations()<<std::endl;
        if(optimizedPoint.norm()>maxNorm*1.1){ // if greater than 10% more than maxnorm, throw error
            std::cerr<<"error: estimated norm is greater than 10% larger than set max norm!"<<std::endl;
            return 1;
        }
    }
    return 0;
}

int test_derivative_numerically(const bioslam::MaxPoint3MagnitudeFactor& fac, const gtsam::Point3 &p){
    // unit test the Jacobian against GTSAM's numerical derivative
    // this may seem silly for such a simple factor, but it's important to do.
    // () get derived error and jacobians
    gtsam::Matrix derivedH1;
    gtsam::Vector derivedErr=fac.evaluateError(p,derivedH1);
    // () get numerical jacobians
    //    I think to call it it's numericalDerivativeXY where X=number of input variables and Y=which Jacobian you want to test
    //    templates are: <output type (typically gtsam::Vector), then the input argument types in order)
    gtsam::Matrix numericalH1=gtsam::numericalDerivative11<gtsam::Vector,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Point3&)>
                    (std::bind(&bioslam::MaxPoint3MagnitudeFactor::evaluateError,fac,std::placeholders::_1,boost::none)),p,1e-5);
    // now test using gtsam::assert_equal()
    bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-7);
    if (!testH1){
        std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
        return 1;
    }
    return 0;
}