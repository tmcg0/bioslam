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

int test_derivative_numerically(const bioslam::HingeJointConstraintNormErrEstAngVel& fac, const gtsam::Pose3& poseA, const gtsam::Vector3& angVelA, const gtsam::Pose3& poseB, const gtsam::Vector3& angVelB, const gtsam::Unit3& axisA);
int random_factor_tests(uint nTests);
int test_givenPriorsOn5Variables(uint numTests, double errorTol, const gtsam::SharedNoiseModel& factorNoiseModel, const gtsam::SharedNoiseModel& poseAPriorModel, const gtsam::SharedNoiseModel& angVelAPriorModel,
                                 const gtsam::SharedNoiseModel& poseBPriorModel, const gtsam::SharedNoiseModel& angVelBPriorModel, const gtsam::SharedNoiseModel& axisAPriorModel);

int main(){
    random_factor_tests(500);

    // now test the permutations of tight/loose priors on the argument variables
    double errorTol=1.0e-5;
    gtsam::Vector3 noiseVec(1.0e-3,1.0e-3,1.0e-3);
    gtsam::SharedNoiseModel myNoiseModel=gtsam::noiseModel::Isotropic::Sigmas(gtsam::Vector1(noiseVec.norm()),true);
    gtsam::noiseModel::Diagonal::shared_ptr tightVec = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 1.0e-6,1.0e-6,1.0e-6).finished());
    gtsam::noiseModel::Diagonal::shared_ptr looseVec = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 1.0e6,1.0e6,1.0e6).finished());
    gtsam::noiseModel::Diagonal::shared_ptr tightAxis = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(2) << 1.0e-6,1.0e-6).finished());
    gtsam::noiseModel::Diagonal::shared_ptr looseAxis = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(2) << 1.0e6,1.0e6).finished());
    gtsam::noiseModel::Diagonal::shared_ptr tightPose = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1.0e-6,1.0e-6,1.0e-6,1.0e-6,1.0e-6,1.0e-6).finished());
    gtsam::noiseModel::Diagonal::shared_ptr loosePose = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1.0e6,1.0e6,1.0e6,1.0e6,1.0e6,1.0e6).finished());
    // run the permutations. call loose 0 and tight 1.
    test_givenPriorsOn5Variables(50, errorTol, myNoiseModel, loosePose,looseVec,loosePose,looseVec,looseAxis); // 0 0 0 0 0
    return 0;
}

int test_givenPriorsOn5Variables(uint numTests, double errorTol, const gtsam::SharedNoiseModel& factorNoiseModel, const gtsam::SharedNoiseModel& poseAPriorModel, const gtsam::SharedNoiseModel& angVelAPriorModel,
                                  const gtsam::SharedNoiseModel& poseBPriorModel, const gtsam::SharedNoiseModel& angVelBPriorModel, const gtsam::SharedNoiseModel& axisAPriorModel){
    // this factor is a function of six variables. use this as a solution test where you vary all the permutations of tight/loose priors on the 6 variables to make sure it converges to the condition you want: velocity minimized between the two frames.
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
        gtsam::Key axisBKey=gtsam::Symbol('f',0);
        bioslam::HingeJointConstraintNormErrEstAngVel testFacA(poseAKey, angVelAKey, poseBKey, angVelBKey, axisAKey, factorNoiseModel);
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
        //myVals.insert(axisBKey,axisB);
        } else {
        myVals.update(angVelAKey, angVelA);
        myVals.update(angVelBKey, angVelB);
        myVals.update(poseAKey,poseA);
        myVals.update(poseBKey,poseB);
        myVals.update(axisAKey,axisA);
        //myVals.update(axisBKey,axisB);
        }
        // while we're at it, test derivative numerically
        //test_derivative_numerically(testFac, linVelA, angVelA, vecA, linVelB, angVelB, vecB);
        // now add priors you input as arguments
        mygraph += gtsam::PriorFactor<gtsam::Vector3>(angVelAKey, angVelA, angVelAPriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Vector3>(angVelBKey, angVelB, angVelBPriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Pose3>(poseAKey,poseA,poseAPriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Pose3>(poseBKey,poseB,poseBPriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Unit3>(axisAKey,axisA,axisAPriorModel);
        // the inf problem is definitely with the velocity factor. for example, if you remove the velocity factor and add a simple between factor (see below),
        // the graph has finite cost
        //mygraph.add(gtsam::BetweenFactor<gtsam::Vector3>(linVelAKey,linVelBKey,gtsam::Vector3(0.,0.,0.),linVelAPriorModel));
        //mygraph.print();
        // optimizer setup. putting convergence criteria at extremes to force LM to give up.
        double relErrDecreaseLimit=1.0e-8; // convergence criteria for relative decrease in error
        double absErrDecreaseLimit=1.0e-8; // convergence criteria for absolute decrease in error
        int maxIterations=5000; // maximum number of iterations
        gtsam::LevenbergMarquardtParams params; params.setVerbosityLM("SUMMARY");
        //params.setRelativeErrorTol(relErrDecreaseLimit); params.setAbsoluteErrorTol(absErrDecreaseLimit);
        gtsam::LevenbergMarquardtOptimizer optimizer(mygraph,myVals,params);
        double initialError=optimizer.error(), currentError=optimizer.error(), previousError, absErrorDecrease=9.0e9, relErrorDecrease=9.0e9;
        uint nIterations=0;
        while(relErrorDecrease>relErrDecreaseLimit && absErrorDecrease>absErrDecreaseLimit && optimizer.iterations()<maxIterations){
            // set previous error values
            previousError=currentError;
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
        }
        gtsam::Values estimate=optimizer.optimize();
        if(nIterations==0){
            std::cout<<"WARNING: zero iterations occured!"<<std::endl;
        }
        double finalError=optimizer.error();
        // pull out estimated values
        gtsam::Vector3 est_angVelA=estimate.at<gtsam::Vector3>(angVelAKey);
        gtsam::Vector3 est_angVelB=estimate.at<gtsam::Vector3>(angVelBKey);
        gtsam::Pose3 est_poseA=estimate.at<gtsam::Pose3>(poseAKey);
        gtsam::Pose3 est_poseB=estimate.at<gtsam::Pose3>(poseBKey);
        gtsam::Unit3 est_axisA=estimate.at<gtsam::Unit3>(axisAKey);
        /*
        double finalJointVelErr=jointConnectionVelocityError(est_poseA,est_linVelA,est_angVelA,est_vecA,est_poseB,est_linVelB,est_angVelB,est_vecB);
        std::cout<<"    test #"<<i<<": initial velocity error: "<<initialJointVelErr<<", final error: "<<finalJointVelErr<<"  |  optimizer error "<<initialError<<" --> "<<finalError<<" ("<<nIterations<<" iterations, with "<<mygraph.size()<<" factors and "<<myVals.size()<<" values)"<<std::endl;
        std::cout<<"        initial: linVelA=["<<linVelA.transpose()<<"], angVelA=["<<angVelA.transpose()<<"], vecA=["<<vecA.transpose()<<"] ==> jointVelA=["<<(jointVelocity(poseA,linVelA,angVelA,vecA)).transpose()<<"]"<<std::endl;
        std::cout<<"                 linVelB=["<<linVelB.transpose()<<"], angVelB=["<<angVelB.transpose()<<"], vecB=["<<vecB.transpose()<<"] ==> jointVelB=["<<(jointVelocity(poseB,linVelB,angVelB,vecB)).transpose()<<"]"<<std::endl;
        std::cout<<"                     ==> vel diff=["<<(jointVelocity(poseA,linVelA,angVelA,vecA)-jointVelocity(poseB,linVelB,angVelB,vecB)).transpose()<<"], norm="<<(jointVelocity(poseA,linVelA,angVelA,vecA)-jointVelocity(poseB,linVelB,angVelB,vecB)).norm()<<std::endl;
        std::cout<<"        optimized: linVelA=["<<est_linVelA.transpose()<<"], angVelA=["<<est_angVelA.transpose()<<"], vecA=["<<est_vecA.transpose()<<"] ==> jointVelA=["<<(jointVelocity(est_poseA,est_linVelA,est_angVelA,est_vecA)).transpose()<<"]"<<std::endl;
        std::cout<<"                   linVelB=["<<est_linVelB.transpose()<<"], angVelB=["<<est_angVelB.transpose()<<"], vecB=["<<est_vecB.transpose()<<"] ==> jointVelB=["<<(jointVelocity(est_poseB,est_linVelB,est_angVelB,est_vecB)).transpose()<<"]"<<std::endl;
        std::cout<<"                     ==> vel diff=["<<(jointVelocity(est_poseA,est_linVelA,est_angVelA,est_vecA)-jointVelocity(est_poseB,est_linVelB,est_angVelB,est_vecB)).transpose()<<"], norm="<<(jointVelocity(est_poseA,est_linVelA,est_angVelA,est_vecA)-jointVelocity(est_poseB,est_linVelB,est_angVelB,est_vecB)).norm()<<std::endl;
        */
        if(finalError>errorTol){ // if greater than tolerance
            std::cerr<<"error: optimizer did not converge! final error: "<<finalError<<std::endl<<std::endl;
            return 2;
        }
        /*
        if(finalJointVelErr>errorTol){ // if greater than tolerance
            std::cerr<<"error: connection error is greater than "<<errorTol<<"! ("<<finalJointVelErr<<")"<<std::endl<<std::endl;
            return 3;
        }
         */
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
        bioslam::HingeJointConstraintNormErrEstAngVel testFacA(poseAKey, angVelAKey, poseBKey, angVelBKey, axisAKey, myNoiseModel);
        bioslam::HingeJointConstraintNormErrEstAngVel testFacB(poseBKey, angVelBKey, poseAKey, angVelAKey, axisBKey, myNoiseModel);
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

int test_derivative_numerically(const bioslam::HingeJointConstraintNormErrEstAngVel& fac, const gtsam::Pose3& poseA, const gtsam::Vector3& angVelA, const gtsam::Pose3& poseB, const gtsam::Vector3& angVelB, const gtsam::Unit3& axisA){
    // unit test the Jacobian against GTSAM's numerical derivative
    // this may seem silly for such a simple factor, but it's important to do.
    // () get derived error and jacobians
    gtsam::Matrix derivedH1, derivedH2, derivedH3, derivedH4, derivedH5;
    gtsam::Vector derivedErr=fac.evaluateError(poseA,angVelA,poseB,angVelB,axisA,derivedH1, derivedH2, derivedH3, derivedH4, derivedH5);
    // () get numerical jacobians
    //    I think to call it it's numericalDerivativeXY where X=number of input variables and Y=which Jacobian you want to test
    //    templates are: <output type (typically gtsam::Vector), then the input argument types in order)
    gtsam::Matrix numericalH1=gtsam::numericalDerivative51<gtsam::Vector,gtsam::Pose3,gtsam::Vector3,gtsam::Pose3,gtsam::Vector3,gtsam::Unit3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Unit3&)>
                    (std::bind(&bioslam::HingeJointConstraintNormErrEstAngVel::evaluateErrorNoJacCall, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5)), poseA, angVelA, poseB, angVelB, axisA, 1e-5);
    gtsam::Matrix numericalH2=gtsam::numericalDerivative52<gtsam::Vector,gtsam::Pose3,gtsam::Vector3,gtsam::Pose3,gtsam::Vector3,gtsam::Unit3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Unit3&)>
                    (std::bind(&bioslam::HingeJointConstraintNormErrEstAngVel::evaluateErrorNoJacCall, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5)), poseA, angVelA, poseB, angVelB, axisA, 1e-5);
    gtsam::Matrix numericalH3=gtsam::numericalDerivative53<gtsam::Vector,gtsam::Pose3,gtsam::Vector3,gtsam::Pose3,gtsam::Vector3,gtsam::Unit3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Unit3&)>
                    (std::bind(&bioslam::HingeJointConstraintNormErrEstAngVel::evaluateErrorNoJacCall, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5)), poseA, angVelA, poseB, angVelB, axisA, 1e-5);
    gtsam::Matrix numericalH4=gtsam::numericalDerivative54<gtsam::Vector,gtsam::Pose3,gtsam::Vector3,gtsam::Pose3,gtsam::Vector3,gtsam::Unit3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Unit3&)>
                    (std::bind(&bioslam::HingeJointConstraintNormErrEstAngVel::evaluateErrorNoJacCall, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5)), poseA, angVelA, poseB, angVelB, axisA, 1e-5);
    gtsam::Matrix numericalH5=gtsam::numericalDerivative55<gtsam::Vector,gtsam::Pose3,gtsam::Vector3,gtsam::Pose3,gtsam::Vector3,gtsam::Unit3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Unit3&)>
                    (std::bind(&bioslam::HingeJointConstraintNormErrEstAngVel::evaluateErrorNoJacCall, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5)), poseA, angVelA, poseB, angVelB, axisA, 1e-5);
    // now test using gtsam::assert_equal()
    bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-6);
    bool testH2=gtsam::assert_equal(derivedH2,numericalH2,1e-6);
    bool testH3=gtsam::assert_equal(derivedH3,numericalH3,1e-6);
    bool testH4=gtsam::assert_equal(derivedH4,numericalH4,1e-6);
    bool testH5=gtsam::assert_equal(derivedH5,numericalH5,1e-5);
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
