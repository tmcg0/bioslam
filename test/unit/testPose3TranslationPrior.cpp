// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// unit test of Pose3TranslationPrior
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

int test_derivative_numerically(const bioslam::Pose3TranslationPrior& fac, const gtsam::Pose3& x);
int random_factor_tests(uint nTests);
int opt_tests(uint nTests=1000, double errorTol=1.0e-5);

int main(){
    random_factor_tests(1000);
    opt_tests(1000,1.0e-50);
    return 0;
}

int opt_tests(uint nTests, double errorTol){
    gtsam::SharedNoiseModel myNoiseModel=gtsam::noiseModel::Isotropic::Sigma(3,0.001);
    gtsam::Key xKey=gtsam::Symbol('a',0);
    // (note: when sigma=1, then optimizer error = 0.5*(factor error^2)
    for(uint i=0;i<nTests;i++) {
        gtsam::Values myVals;
        gtsam::Pose3 x=testutils::randomPose3();
        gtsam::Vector3 prior=testutils::randomVector3();
        myVals.insert(xKey, x);
        gtsam::NonlinearFactorGraph mygraph;
        bioslam::Pose3TranslationPrior testFac=bioslam::Pose3TranslationPrior(xKey, prior, myNoiseModel);
        mygraph.push_back(testFac);
        // test derivative numerically
        test_derivative_numerically(testFac, x);
        // optimize
        gtsam::LevenbergMarquardtOptimizer optimizer(mygraph,myVals);
        gtsam::Values estimate=optimizer.optimize();
        // check
        gtsam::Point3 estTranslation=estimate.at<gtsam::Pose3>(xKey).translation();
        double err=(estTranslation-prior).norm();
        std::cout<<"test #"<<i<<": prior = "<<prior.transpose()<<", converged = "<<estTranslation.transpose()<<", err = "<<err<<" in "<<optimizer.iterations()<<" iterations"<<std::endl;
        if( err > errorTol ){
            throw std::runtime_error("did not converge to expected prior. err norm = "+std::to_string(err)+", tol = "+std::to_string(errorTol));
        }
    }
    return 0;
}

int random_factor_tests(uint nTests){
    // in a loop, randomly generate inputs to the max factor and test derivatives numerically.
    gtsam::SharedNoiseModel myNoiseModel=gtsam::noiseModel::Isotropic::Sigma(3,0.001);
    gtsam::Values myVals;
    gtsam::NonlinearFactorGraph mygraph;
    gtsam::Key xKey=gtsam::Symbol('a',0);
    // (note: when sigma=1, then optimizer error = 0.5*(factor error^2)
    for(uint i=0;i<nTests;i++) {
        gtsam::Pose3 x=testutils::randomPose3();
        gtsam::Vector3 prior=testutils::randomVector3();
        if (!myVals.exists(xKey)) {
            myVals.insert(xKey, x);
        } else {
            myVals.update(xKey, x);
        }
        bioslam::Pose3TranslationPrior testFac=bioslam::Pose3TranslationPrior(xKey, prior, myNoiseModel);
        // test derivative numerically
        test_derivative_numerically(testFac, x);
    }
    return 0;
}

int test_derivative_numerically(const bioslam::Pose3TranslationPrior& fac, const gtsam::Pose3& x){
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
                    (std::bind(&bioslam::Pose3TranslationPrior::evaluateError,fac,std::placeholders::_1,boost::none)),x,1e-5);

    // now test using gtsam::assert_equal()
    bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-10);
    if (!testH1){
        std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
        throw std::runtime_error("Jacobian did not check out numerically");
        return 1;
    }
    return 0;
}
