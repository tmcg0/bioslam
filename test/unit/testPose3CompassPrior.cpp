// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// unit test of Pose3CompassPrior
// unit test framework:
// 1) do the jacobians check out numerically?
// 2) optimizer tests

#include "testutils.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/numericalDerivative.h>
#include <factors/Pose3Priors.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

int test_derivative_numerically(const bioslam::Pose3CompassPrior& fac, const gtsam::Pose3& x);
int random_factor_tests(uint nTests);
int opt_tests(uint nTests=1000, double errorTol=1.0e-5);

int main(){
    random_factor_tests(1000);
    opt_tests(1000,1.0e-5);
    return 0;
}

int opt_tests(uint nTests, double errorTol){
    gtsam::SharedNoiseModel myNoiseModel=gtsam::noiseModel::Isotropic::Sigma(1,0.001);
    gtsam::Key xKey=gtsam::Symbol('a',0);
    // (note: when sigma=1, then optimizer error = 0.5*(factor error^2)
    for(uint i=0;i<nTests;i++) {
        gtsam::Values myVals;
        gtsam::Pose3 x=testutils::randomPose3();
        double prior=testutils::dRand(-M_PI,M_PI);
        gtsam::Vector3 refVecLocal(0.0,0.0,1.0), refVecNav(1.0,0.0,0.0), upVecNav(0.0,0.0,1.0);
        myVals.insert(xKey, x);
        gtsam::NonlinearFactorGraph mygraph;
        bioslam::Pose3CompassPrior testFac=bioslam::Pose3CompassPrior(xKey,refVecLocal,refVecNav,upVecNav,prior,myNoiseModel);
        mygraph.push_back(testFac);
        // test derivative numerically
        test_derivative_numerically(testFac, x);
        // optimize
        gtsam::LevenbergMarquardtOptimizer optimizer(mygraph,myVals);
        gtsam::Values estimate=optimizer.optimize();
        // check
        gtsam::Rot3 estRot=estimate.at<gtsam::Pose3>(xKey).rotation();
        double estCompassAng=bioslam::Pose3CompassPrior::compassAngle(estRot,refVecLocal,refVecNav,upVecNav);
        double err=estCompassAng-prior;
        std::cout<<"test #"<<i<<": prior ang = "<<prior<<", converged ang = "<<estCompassAng<<", err = "<<err<<" in "<<optimizer.iterations()<<" iterations"<<std::endl;
        if( err > errorTol ){
            throw std::runtime_error("did not converge to expected prior. err = "+std::to_string(err)+", tol = "+std::to_string(errorTol));
        }
    }
    return 0;
}

int random_factor_tests(uint nTests){
    // in a loop, randomly generate inputs to the max factor and test derivatives numerically.
    gtsam::SharedNoiseModel myNoiseModel=gtsam::noiseModel::Isotropic::Sigma(1,0.001);
    gtsam::Values myVals;
    gtsam::NonlinearFactorGraph mygraph;
    gtsam::Key xKey=gtsam::Symbol('a',0);
    // (note: when sigma=1, then optimizer error = 0.5*(factor error^2)
    for(uint i=0;i<nTests;i++) {
        gtsam::Pose3 x=testutils::randomPose3();
        double prior=testutils::dRand(-M_PI*0.75,M_PI*0.75);
        gtsam::Vector3 refVecLocal=gtsam::Vector3(1.0,0.0,0.0);
        gtsam::Vector3 refVecNav=gtsam::Vector3(0.0,0.0,1.0);
        gtsam::Vector3 upVecNav=gtsam::Vector3(0.0,0.0,1.0);
        if (!myVals.exists(xKey)) {
            myVals.insert(xKey, x);
        } else {
            myVals.update(xKey, x);
        }
        bioslam::Pose3CompassPrior testFac=bioslam::Pose3CompassPrior(xKey,refVecLocal,refVecNav,upVecNav,prior,myNoiseModel);
        // test derivative numerically
        if (!bioslam::Pose3CompassPrior::isCompassVertical(x.rotation(),refVecLocal,upVecNav)){
            // only run test if this random system isn't at the gimbal lock condition
            test_derivative_numerically(testFac, x);
        }else{
            std::cout<<"randomly generated compass system was at gimbal lock condition; skipping numerical testing of jacobian."<<std::endl;
        }
    }
    return 0;
}

int test_derivative_numerically(const bioslam::Pose3CompassPrior& fac, const gtsam::Pose3& x){
    // unit test the Jacobian against GTSAM's numerical derivative
    // this may seem silly for such a simple factor, but it's important to do.
    // () get derived error and jacobians
    gtsam::Matrix derivedH1;
    gtsam::Vector derivedErr=fac.evaluateError(x, derivedH1);
    // () get numerical jacobians
    //    I think to call it it's numericalDerivativeXY where X=number of input variables and Y=which Jacobian you want to test
    //    templates are: <output type (typically gtsam::Vector), then the input argument types in order)
    gtsam::Matrix numericalH1=gtsam::numericalDerivative11<gtsam::Vector,gtsam::Pose3>(
            std::function<gtsam::Vector(const gtsam::Pose3&)>
                    (std::bind(&bioslam::Pose3CompassPrior::evaluateError,fac,std::placeholders::_1,boost::none)),x,1e-8);

    // now test using gtsam::assert_equal()
    bool testH1=gtsam::assert_equal(derivedH1,numericalH1,5.0e-3);
    if (!testH1){
        std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
        throw std::runtime_error("Jacobian did not check out numerically");
    }
    return 0;
}
