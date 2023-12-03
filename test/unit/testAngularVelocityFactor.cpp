// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// unit test of AngularVelocityFactor

#include "testutils.h"
#include <factors/AngularVelocityFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/base/numericalDerivative.h>

int test_derivative_numerically(const bioslam::AngularVelocityFactor& fac, const gtsam::Vector3& estAngVel, const gtsam::imuBias::ConstantBias& imuBias);
int random_factor_tests(uint nTests);

int main(){
    random_factor_tests(5000);
}


int random_factor_tests(uint nTests){
    // in a loop, randomly generate inputs to factor and test derivatives numerically, as well as test convergence and solution.
    gtsam::SharedNoiseModel facNoiseModel=gtsam::noiseModel::Isotropic::Sigmas(gtsam::Vector3(1.0e-3, 1.0e-3, 1.0e-3));
    gtsam::SharedNoiseModel estAngVelPriorModel=gtsam::noiseModel::Isotropic::Sigmas(gtsam::Vector3(1.0e-3,1.0e-3,1.0e-3));
    gtsam::Values myVals;
    gtsam::Key estAngVelKey=gtsam::Symbol('a',0), imuBiasKey=gtsam::Symbol('b',0);
    // (note: when sigma=1, then optimizer error = 0.5*(factor error^2)
    for(uint i=0;i<nTests;i++) {
        gtsam::NonlinearFactorGraph mygraph;
        gtsam::Vector3 estAngVel = testutils::randomVector3();
        gtsam::imuBias::ConstantBias imuBias(testutils::randomVector3(),testutils::randomVector3());
        gtsam::Vector3 angVelMeas=testutils::randomVector3();
        if (!myVals.exists(estAngVelKey)) {
            myVals.insert(estAngVelKey, estAngVel);
            myVals.insert(imuBiasKey, imuBias);
        } else {
            myVals.update(estAngVelKey, estAngVel);
            myVals.update(imuBiasKey, imuBias);
        }
        bioslam::AngularVelocityFactor testFac=bioslam::AngularVelocityFactor(estAngVelKey, imuBiasKey, angVelMeas, facNoiseModel);
        // test derivative numerically
        test_derivative_numerically(testFac, estAngVel, imuBias);
        // print
        std::cout<<"--- test "<<i<<"/"<<nTests<<" ---"<<std::endl;
        std::cout<<"    initial conditions: angVel=["<<estAngVel.transpose()<<"], gyroBias=["<<imuBias.gyroscope().transpose()<<"], angular velocity measurement = ["<<angVelMeas.transpose()<<"]"<<std::endl;
        // solve via optimization
        mygraph.push_back(testFac); // add the factor to test
        mygraph.push_back(gtsam::PriorFactor<gtsam::Vector3>(estAngVelKey,estAngVel,estAngVelPriorModel)); // add a prior on the estimated angular velocity
        gtsam::LevenbergMarquardtParams params; params.setVerbosityLM("SUMMARY");
        gtsam::LevenbergMarquardtOptimizer optimizer(mygraph,myVals,params);
        gtsam::Values estimate=optimizer.optimize();
        std::cout<<"    optimized values: angVel=["<<estimate.at<gtsam::Vector3>(estAngVelKey).transpose()<<"], gyroBias=["<<estimate.at<gtsam::imuBias::ConstantBias>(imuBiasKey).gyroscope().transpose()<<"]";
        std::cout<<" (true + bias = ["<<(estimate.at<gtsam::Vector3>(estAngVelKey)+estimate.at<gtsam::imuBias::ConstantBias>(imuBiasKey).gyroscope()).transpose()<<"])"<<std::endl;
        // make comaprison for test
        // it should be meas = true + bias + noise
        bool testResult=gtsam::assert_equal((gtsam::Matrix)angVelMeas,(gtsam::Matrix)(estimate.at<gtsam::Vector3>(estAngVelKey)+estimate.at<gtsam::imuBias::ConstantBias>(imuBiasKey).gyroscope()),1.0e-5);
        if(!testResult){
            throw std::runtime_error("test failed---angular velocity factor did not optimize to expected result.");
        }
    }
    return 0;
}

int test_derivative_numerically(const bioslam::AngularVelocityFactor& fac, const gtsam::Vector3& estAngVel, const gtsam::imuBias::ConstantBias& imuBias){
    // unit test the Jacobian against GTSAM's numerical derivative
    // () get derived error and jacobians
    gtsam::Matrix derivedH1, derivedH2;
    gtsam::Vector derivedErr=fac.evaluateError(estAngVel,imuBias,derivedH1, derivedH2);
    // () get numerical jacobians
    gtsam::Matrix numericalH1=gtsam::numericalDerivative21<gtsam::Vector,gtsam::Vector3,gtsam::imuBias::ConstantBias>(
            std::function<gtsam::Vector(const gtsam::Vector3&, const gtsam::imuBias::ConstantBias&)>
                    (std::bind(&bioslam::AngularVelocityFactor::evaluateError,fac,std::placeholders::_1,std::placeholders::_2,boost::none,boost::none)),estAngVel,imuBias,1e-5);
    gtsam::Matrix numericalH2=gtsam::numericalDerivative22<gtsam::Vector,gtsam::Vector3,gtsam::imuBias::ConstantBias>(
            std::function<gtsam::Vector(const gtsam::Vector3&, const gtsam::imuBias::ConstantBias&)>
                    (std::bind(&bioslam::AngularVelocityFactor::evaluateError,fac,std::placeholders::_1,std::placeholders::_2,boost::none,boost::none)),estAngVel,imuBias,1e-5);
    // now test using gtsam::assert_equal()
    bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-7);
    bool testH2=gtsam::assert_equal(derivedH2,numericalH2,1e-7);
    if (!testH1){
        std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
        throw std::runtime_error("jacobian did not check out numerically.");
    }
    if (!testH2){
        std::cerr<<"H2 did not check out numerically."<<std::endl<<"derivedH2="<<derivedH2<<std::endl<<"numericalH2"<<numericalH2<<std::endl;
        throw std::runtime_error("jacobian did not check out numerically.");
    }
    return 0;
}