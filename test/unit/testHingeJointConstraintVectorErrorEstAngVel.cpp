// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// unit test of the factor of the same name. unit test framework:
// 1) do the jacobians check out numerically?
// 2) when optimized, does the optimization error get driven to zero?

#include "testutils.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <factors/HingeJointFactors.h>

int test_derivative_numerically(const bioslam::HingeJointConstraintVecErrEstAngVel& fac, const gtsam::Pose3& poseA, const gtsam::Vector3& angVelA, const gtsam::Pose3& poseB, const gtsam::Vector3& angVelB, const gtsam::Unit3& axisA);
int random_factor_tests(uint nTests=500);
int test_givenPriorsOn5Variables(uint numTests, double errorTol, const gtsam::SharedNoiseModel& factorNoiseModel, const gtsam::SharedNoiseModel& poseAPriorModel, const gtsam::SharedNoiseModel& angVelAPriorModel,
                                 const gtsam::SharedNoiseModel& poseBPriorModel, const gtsam::SharedNoiseModel& angVelBPriorModel, const gtsam::SharedNoiseModel& axisAPriorModel);

int main(){
    random_factor_tests(); // checks numerical derivatives
    // now test the permutations of tight/loose priors on the argument variables
    double errorTol=1.0e-4, loose=1.0e1, tight=1.0e-6;
    gtsam::SharedNoiseModel facNoiseModel=gtsam::noiseModel::Isotropic::Sigmas(gtsam::Vector3(1.0e-3,1.0e-3,1.0e-3));
    gtsam::noiseModel::Diagonal::shared_ptr tightVec = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << tight,tight,tight).finished());
    gtsam::noiseModel::Diagonal::shared_ptr looseVec = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << loose,loose,loose).finished());
    gtsam::noiseModel::Diagonal::shared_ptr tightAxis = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(2) << tight,tight).finished());
    gtsam::noiseModel::Diagonal::shared_ptr looseAxis = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(2) << loose,loose).finished());
    gtsam::noiseModel::Diagonal::shared_ptr tightPose = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << tight,tight,tight,tight,tight,tight).finished());
    gtsam::noiseModel::Diagonal::shared_ptr loosePose = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << loose,loose,loose,loose,loose,loose).finished());
    // run the permutations. call loose 0 and tight 1.
    test_givenPriorsOn5Variables(50, errorTol, facNoiseModel, loosePose,looseVec,loosePose,looseVec,looseAxis); // 0 0 0 0 0
    return 0;
}

int test_givenPriorsOn5Variables(uint numTests, double errorTol, const gtsam::SharedNoiseModel& factorNoiseModel, const gtsam::SharedNoiseModel& poseAPriorModel, const gtsam::SharedNoiseModel& angVelAPriorModel,
                                  const gtsam::SharedNoiseModel& poseBPriorModel, const gtsam::SharedNoiseModel& angVelBPriorModel, const gtsam::SharedNoiseModel& axisAPriorModel){
    std::cout<<"--- testing with 5 variables given priors ---"<<std::endl;
    // (note: when sigma=1, then optimizer error = 0.5*(factor error^2)
    for(uint i=0;i<numTests;i++){
        gtsam::Values myVals;
        gtsam::NonlinearFactorGraph mygraph;
        gtsam::Key poseAKey=gtsam::Symbol('a',0);
        gtsam::Key angVelAKey=gtsam::Symbol('b',0);
        gtsam::Key poseBKey=gtsam::Symbol('c',0);
        gtsam::Key angVelBKey=gtsam::Symbol('d',0);
        gtsam::Key axisAKey=gtsam::Symbol('e',0);
        bioslam::HingeJointConstraintVecErrEstAngVel testFacA(poseAKey, angVelAKey, poseBKey, angVelBKey, axisAKey, factorNoiseModel);
        mygraph.add(testFacA);
        // generate random data for these six variables
        gtsam::Vector3 angVelA = testutils::randomVector3(), angVelB = testutils::randomVector3();
        gtsam::Pose3 poseA=testutils::randomPose3(), poseB=testutils::randomPose3();
        gtsam::Unit3 axisA=testutils::randomUnit3(), axisB=testutils::randomUnit3();
        if (!myVals.exists(poseAKey)) {
            myVals.insert(angVelAKey, angVelA);
            myVals.insert(angVelBKey, angVelB);
            myVals.insert(poseAKey,poseA);
            myVals.insert(poseBKey,poseB);
            myVals.insert(axisAKey,axisA);
        } else {
            myVals.update(angVelAKey, angVelA);
            myVals.update(angVelBKey, angVelB);
            myVals.update(poseAKey,poseA);
            myVals.update(poseBKey,poseB);
            myVals.update(axisAKey,axisA);
        }
        // now add priors you input as arguments
        mygraph += gtsam::PriorFactor<gtsam::Vector3>(angVelAKey, angVelA, angVelAPriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Vector3>(angVelBKey, angVelB, angVelBPriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Pose3>(poseAKey,poseA,poseAPriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Pose3>(poseBKey,poseB,poseBPriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Unit3>(axisAKey,axisA,axisAPriorModel);
        // optimizer setup. putting convergence criteria at extremes to force LM to give up.
        double relErrDecreaseLimit=1.0e-8, absErrDecreaseLimit=1.0e-8; // convergence criteria for absolute and relative decrease in error
        int maxIterations=5000; // maximum number of iterations
        gtsam::LevenbergMarquardtOptimizer optimizer(mygraph,myVals);
        //gtsam::GaussNewtonOptimizer optimizer(mygraph,myVals); // should also work with GN. However, needs tighter priors to keep rank.
        double initialError=optimizer.error(), currentError=optimizer.error(), previousError, absErrorDecrease=9.0e9, relErrorDecrease=9.0e9;
        uint nIterations=0;
        while(relErrorDecrease>relErrDecreaseLimit && absErrorDecrease>absErrDecreaseLimit && optimizer.iterations()<maxIterations){
            // set previous error values
            previousError=currentError;
            // check jacobian?
            boost::shared_ptr<gtsam::GaussianFactorGraph> gfg=mygraph.linearize(myVals);
            gtsam::Matrix m=gfg->sparseJacobian_();
            gtsamutils::writeEigenMatrixToCsvFile("jac.csv",m);
            // perform iteration and checks
            boost::shared_ptr<gtsam::GaussianFactorGraph> g=optimizer.iterate(); // should have updated things in here now
            //g->print();
            if(optimizer.iterations()!=nIterations+1){ // why didn't this tick up?
                std::cout<<"WARNING: iteration number did not increase! exiting..."<<std::endl;
                break;
            }else{nIterations=optimizer.iterations();}
            // extract current state
            currentError=optimizer.error();
            // compute change in errors
            absErrorDecrease=previousError-currentError; relErrorDecrease=absErrorDecrease/previousError;
            std::cout<<"--- end iteration #"<<optimizer.iterations()-1<<": total error "<<currentError<<" (decrease: "<<absErrorDecrease<<" || "<<relErrorDecrease*100<<"%), factor norm error = "<<testFacA.error(optimizer.values())<<std::endl;
        }
        gtsam::Values estimate=optimizer.optimize();
        if(nIterations==0){
            std::cout<<"WARNING: zero iterations occured!"<<std::endl;
        }
        double finalError=optimizer.error(); // global optimizer error
        // pull out estimated values
        gtsam::Vector3 est_angVelA=estimate.at<gtsam::Vector3>(angVelAKey), est_angVelB=estimate.at<gtsam::Vector3>(angVelBKey);
        gtsam::Pose3 est_poseA=estimate.at<gtsam::Pose3>(poseAKey), est_poseB=estimate.at<gtsam::Pose3>(poseBKey);
        gtsam::Unit3 est_axisA=estimate.at<gtsam::Unit3>(axisAKey);
        // compute and test factor error
        if(testFacA.error(estimate)>errorTol){ // if greater than tolerance
            std::cerr<<"error: factor error should have been driven to zero, but final error: "<<testFacA.error(estimate)<<std::endl<<std::endl;
            return 2;
        }
        // now clear the graph
        myVals.clear();
    }
    return 0;
}

int random_factor_tests(uint nTests){
    // in a loop, randomly generate inputs to the max factor and test derivatives numerically.
    gtsam::Vector3 myNoiseVec(1.0e-3,1.0e-3,1.0e-3);
    gtsam::SharedNoiseModel myNoiseModel=gtsam::noiseModel::Isotropic::Sigmas(myNoiseVec);
    gtsam::Values myVals;
    gtsam::NonlinearFactorGraph mygraph;
    gtsam::Key poseAKey=gtsam::Symbol('a',0);
    gtsam::Key angVelAKey=gtsam::Symbol('b',0);
    gtsam::Key poseBKey=gtsam::Symbol('c',0);
    gtsam::Key angVelBKey=gtsam::Symbol('d',0);
    gtsam::Key axisAKey=gtsam::Symbol('e',0);
    gtsam::Key axisBKey=gtsam::Symbol('f',0);
    // (note: when sigma=1, then optimizer error = 0.5*(factor error^2)
    for(uint i=0;i<nTests;i++) {
        gtsam::Vector3 angVelA = testutils::randomVector3(), angVelB = testutils::randomVector3();
        gtsam::Pose3 poseA=testutils::randomPose3(), poseB=testutils::randomPose3();
        gtsam::Unit3 axisA=testutils::randomUnit3(), axisB=testutils::randomUnit3();
        if (!myVals.exists(poseAKey)) {
            myVals.insert(angVelAKey, angVelA);
            myVals.insert(angVelBKey, angVelB);
            myVals.insert(poseAKey,poseA);
            myVals.insert(poseBKey,poseB);
            myVals.insert(axisAKey,axisA);
            myVals.insert(axisBKey,axisB);
        } else {
            myVals.update(angVelAKey, angVelA);
            myVals.update(angVelBKey, angVelB);
            myVals.update(poseAKey,poseA);
            myVals.update(poseBKey,poseB);
            myVals.update(axisAKey,axisA);
            myVals.update(axisBKey,axisB);
        }
        bioslam::HingeJointConstraintVecErrEstAngVel testFacA(poseAKey, angVelAKey, poseBKey, angVelBKey, axisAKey, myNoiseModel);
        bioslam::HingeJointConstraintVecErrEstAngVel testFacB(poseBKey, angVelBKey, poseAKey, angVelAKey, axisBKey, myNoiseModel);
        // test derivative numerically
        if(test_derivative_numerically(testFacA, poseA, angVelA, poseB, angVelB, axisA)!=0){
            throw std::runtime_error("at least one derivative for the proximal factor did not check out numerically.");
        }
        if(test_derivative_numerically(testFacB, poseA, angVelA, poseB, angVelB, axisB)!=0){
            throw std::runtime_error("at least one derivative for the distal factor did not check out numerically.");
        }
    }
    return 0;
}

int test_derivative_numerically(const bioslam::HingeJointConstraintVecErrEstAngVel& fac, const gtsam::Pose3& poseA, const gtsam::Vector3& angVelA, const gtsam::Pose3& poseB, const gtsam::Vector3& angVelB, const gtsam::Unit3& axisA){
    // unit test the Jacobian against numerical derivative
    // () get derived error and jacobians
    gtsam::Matrix derivedH1, derivedH2, derivedH3, derivedH4, derivedH5;
    gtsam::Vector derivedErr=fac.evaluateError(poseA,angVelA,poseB,angVelB,axisA,derivedH1, derivedH2, derivedH3, derivedH4, derivedH5);
    // () get numerical jacobians
    //    I think to call it it's numericalDerivativeXY where X=number of input variables and Y=which Jacobian you want to test
    //    templates are: <output type (typically gtsam::Vector), then the input argument types in order)
    gtsam::Matrix numericalH1=gtsam::numericalDerivative51<gtsam::Vector,gtsam::Pose3,gtsam::Vector3,gtsam::Pose3,gtsam::Vector3,gtsam::Unit3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Unit3&)>
                    (std::bind(&bioslam::HingeJointConstraintVecErrEstAngVel::evaluateErrorNoJacCall, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5)), poseA, angVelA, poseB, angVelB, axisA, 1e-5);
    gtsam::Matrix numericalH2=gtsam::numericalDerivative52<gtsam::Vector,gtsam::Pose3,gtsam::Vector3,gtsam::Pose3,gtsam::Vector3,gtsam::Unit3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Unit3&)>
                    (std::bind(&bioslam::HingeJointConstraintVecErrEstAngVel::evaluateErrorNoJacCall, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5)), poseA, angVelA, poseB, angVelB, axisA, 1e-5);
    gtsam::Matrix numericalH3=gtsam::numericalDerivative53<gtsam::Vector,gtsam::Pose3,gtsam::Vector3,gtsam::Pose3,gtsam::Vector3,gtsam::Unit3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Unit3&)>
                    (std::bind(&bioslam::HingeJointConstraintVecErrEstAngVel::evaluateErrorNoJacCall, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5)), poseA, angVelA, poseB, angVelB, axisA, 1e-5);
    gtsam::Matrix numericalH4=gtsam::numericalDerivative54<gtsam::Vector,gtsam::Pose3,gtsam::Vector3,gtsam::Pose3,gtsam::Vector3,gtsam::Unit3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Unit3&)>
                    (std::bind(&bioslam::HingeJointConstraintVecErrEstAngVel::evaluateErrorNoJacCall, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5)), poseA, angVelA, poseB, angVelB, axisA, 1e-5);
    gtsam::Matrix numericalH5=gtsam::numericalDerivative55<gtsam::Vector,gtsam::Pose3,gtsam::Vector3,gtsam::Pose3,gtsam::Vector3,gtsam::Unit3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Unit3&)>
                    (std::bind(&bioslam::HingeJointConstraintVecErrEstAngVel::evaluateErrorNoJacCall, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5)), poseA, angVelA, poseB, angVelB, axisA, 1e-5);
    // now test using gtsam::assert_equal()
    bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-9);
    bool testH2=gtsam::assert_equal(derivedH2,numericalH2,1e-9);
    bool testH3=gtsam::assert_equal(derivedH3,numericalH3,1e-9);
    bool testH4=gtsam::assert_equal(derivedH4,numericalH4,1e-9);
    bool testH5=gtsam::assert_equal(derivedH5,numericalH5,1e-9);
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
    if (!testH5){
        std::cerr<<"H5 did not check out numerically."<<std::endl<<"derivedH5="<<derivedH5<<std::endl<<"numericalH5"<<numericalH5<<std::endl;
        return 1;
    }
    return 0;
}