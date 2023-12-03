// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// unit test of Point3MagnitudeDifferenceFactor

#include <factors/Point3Priors.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/numericalDerivative.h>
#include <random>
#include "testutils.h"

int test_derivative_numerically(const bioslam::Point3MagnitudeDifferenceFactor& fac, const gtsam::Point3 &v1, const gtsam::Point3 &v2);

int main(){
    // generate a bunch of random Point3's. run them through an optimizer and make sure that their mangitude is the same.
    uint tests=50;
    double tol = 1.0e-4;
    gtsam::Vector velNoiseVec(1); velNoiseVec<<1.0;
    gtsam::SharedNoiseModel myNoiseModel=gtsam::noiseModel::Isotropic::Sigmas(velNoiseVec);
    gtsam::Values myVals;
    gtsam::NonlinearFactorGraph mygraph;
    gtsam::Key v1key=gtsam::Symbol('a', 0);
    gtsam::Key v2key=gtsam::Symbol('b', 0);
    bioslam::Point3MagnitudeDifferenceFactor testfac=bioslam::Point3MagnitudeDifferenceFactor(v1key, v2key, myNoiseModel);
    mygraph.add(testfac);
    for(uint i=0;i<tests;i++){
        gtsam::Point3 v1init=testutils::randomPoint3();
        gtsam::Point3 v2init=testutils::randomPoint3();
        if(!myVals.exists(v1key)){
            myVals.insert(v1key, v1init);
            myVals.insert(v2key, v2init);
        }else{
            myVals.update(v1key, v1init);
            myVals.update(v2key, v2init);
        }
        assert(test_derivative_numerically(testfac, v1init, v2init) == 0);
        gtsam::LevenbergMarquardtOptimizer optimizer(mygraph,myVals);
        gtsam::Values estimate=optimizer.optimize();
        gtsam::Point3 v1est=estimate.at<gtsam::Point3>(v1key);
        gtsam::Point3 v2est=estimate.at<gtsam::Point3>(v2key);
        std::cout << "initial v1: [" << v1init.transpose() << "] (norm=" << v1init.norm() << ")  -->  ";
        std::cout << "optimized v1: [" << v1est.transpose() <<"] (norm=" << v1est.norm() << "). iterations=" << optimizer.iterations() << std::endl;
        std::cout << "initial v2: [" << v2init.transpose() << "] (norm=" << v2init.norm() << ")  -->  ";
        std::cout << "optimized v2: [" << v2est.transpose() <<"] (norm=" << v2est.norm() << "). iterations=" << optimizer.iterations() << std::endl;
        if(abs(v1est.norm() - v2est.norm())>tol){ // if greater than tol, throw error
            std::cerr<<"error: estimated mag diff is greater than tol!"<<std::endl;
            return 1;
        }
    }
    return 0;
}

int test_derivative_numerically(const bioslam::Point3MagnitudeDifferenceFactor& fac, const gtsam::Point3 &v1, const gtsam::Point3 &v2){
    // unit test the Jacobian against GTSAM's numerical derivative
    // this may seem silly for such a simple factor, but it's important to do.
    // () get derived error and jacobians
    gtsam::Matrix derivedH1, derivedH2;
    gtsam::Vector derivedErr=fac.evaluateError(v1, v2, derivedH1, derivedH2);
    // () get numerical jacobians
    //    I think to call it it's numericalDerivativeXY where X=number of input variables and Y=which Jacobian you want to test
    //    templates are: <output type (typically gtsam::Vector), then the input argument types in order)
    gtsam::Matrix numericalH1=gtsam::numericalDerivative21<gtsam::Vector,gtsam::Point3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Point3&,const gtsam::Point3&)>
                    (std::bind(&bioslam::Point3MagnitudeDifferenceFactor::evaluateError,fac,std::placeholders::_1,std::placeholders::_2,boost::none,boost::none)), v1, v2, 1e-5);
    gtsam::Matrix numericalH2=gtsam::numericalDerivative22<gtsam::Vector,gtsam::Point3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Point3&,const gtsam::Point3&)>
                    (std::bind(&bioslam::Point3MagnitudeDifferenceFactor::evaluateError,fac,std::placeholders::_1,std::placeholders::_2,boost::none,boost::none)), v1, v2, 1e-5);
    // now test using gtsam::assert_equal()
    bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-7);
    bool testH2=gtsam::assert_equal(derivedH2,numericalH2,1e-7);
    if (!testH1){
        std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
        return 1;
    }
    if (!testH2){
        std::cerr<<"H2 did not check out numerically."<<std::endl<<"derivedH2="<<derivedH1<<std::endl<<"numericalH2"<<numericalH1<<std::endl;
        return 1;
    }
    return 0;
}