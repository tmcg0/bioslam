// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// unit test of ConstrainedJointCenterNormVelocityFactor
// unit test framework:
// 1) do the jacobians check out numerically?
// 2) when optimized, does the optimization error get driven to zero?
// 3) does an optimizer global error of zero correspond physically to a zero velocity difference?
// 4) can (2) and (3) be repeatedly shown when permutations of variables are held constant?
// todo 5) validation testing: in a lowerBodyPoseEstimator framework, when you set all noises high except for joint velocity factor, does it create a solution which indeed minimizes joint velocity difference?
// todo 6) validation testing: under same conditions as (5), is the joint velocity according to each IMU approximately the same as the discrete time derivative of the position of the joint center? i.e., diff(jointPos)./dt

#include "testutils.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <factors/ConstrainedJointCenterVelocityFactor.h>
#include <lowerBodyPoseEstimator.h>

int test_derivative_numerically(const bioslam::ConstrainedJointCenterNormVelocityFactor& fac, const gtsam::Pose3& poseA, const gtsam::Vector3& linVelA, const gtsam::Vector3& angVelA, const gtsam::Point3& sA, const gtsam::Pose3& poseB, const gtsam::Vector3& linVelB, const gtsam::Vector3& angVelB, const gtsam::Point3& sB);
int random_factor_tests(uint nTests);
int verifyErrorModel(const gtsam::Pose3& poseA, const gtsam::Vector3& linVelA, const gtsam::Vector3& angVelA, const gtsam::Point3& vecA, const gtsam::Pose3& poseB, const gtsam::Vector3& linVelB, const gtsam::Vector3& angVelB, const gtsam::Point3& vecB);
int test_givenPriorsOn8Variables(uint numTests, double errorTol, const gtsam::SharedNoiseModel& factorNoiseModel, const gtsam::SharedNoiseModel& poseAPriorModel, const gtsam::SharedNoiseModel& linVelAPriorModel, const gtsam::SharedNoiseModel& angVelAPriorModel, const gtsam::SharedNoiseModel& vecAPriorModel,
                                 const gtsam::SharedNoiseModel& poseBPriorModel, const gtsam::SharedNoiseModel& linVelBPriorModel, const gtsam::SharedNoiseModel& angVelBPriorModel, const gtsam::SharedNoiseModel& vecBPriorModel);
gtsam::Vector3 jointVelocity(const gtsam::Pose3& poseA, const gtsam::Vector3& linVelA, const gtsam::Vector3& angVelA, const gtsam::Point3& vecA);
double jointConnectionVelocityError(const gtsam::Pose3& poseA, const gtsam::Vector3& linVelA, const gtsam::Vector3& angVelA, const gtsam::Point3& vecA, const gtsam::Pose3& poseB, const gtsam::Vector3& linVelB, const gtsam::Vector3& angVelB, const gtsam::Point3& vecB);
int validation_testing(const double& errorTol);

int main(){
    random_factor_tests(50);
    // now test the permutations of tight/loose priors on the argument variables
    double errorTol=1.0e-5;
    gtsam::Vector3 noiseVec(1.0e-3,1.0e-3,1.0e-3);
    gtsam::SharedNoiseModel myNoiseModel=gtsam::noiseModel::Isotropic::Sigmas(gtsam::Vector1(noiseVec.norm()),true);
    gtsam::noiseModel::Diagonal::shared_ptr tightVec = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 1.0e-6,1.0e-6,1.0e-6).finished());
    gtsam::noiseModel::Diagonal::shared_ptr looseVec = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 1.0e6,1.0e6,1.0e6).finished());
    gtsam::noiseModel::Diagonal::shared_ptr tightPose = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1.0e-6,1.0e-6,1.0e-6,1.0e-6,1.0e-6,1.0e-6).finished());
    gtsam::noiseModel::Diagonal::shared_ptr loosePose = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1.0e6,1.0e6,1.0e6,1.0e6,1.0e6,1.0e6).finished());
    // run the permutations. call loose 0 and tight 1.
    test_givenPriorsOn8Variables(50, errorTol, myNoiseModel, loosePose, looseVec,looseVec,looseVec,loosePose,looseVec,looseVec,looseVec); // 0 0 0 0 0 0 0 0
    // do the validation testing (5 and 6 above)
    /* for now, this validation testing has been tabled because it takes a long time to run the optimization and you aren't prioritizing this factor right now anyway.
    uint passValidationTest=validation_testing(errorTol);
    if(passValidationTest!=0){
        throw std::runtime_error("validation test failed");
    }
     */
    return 0;
}

int validation_testing(const double& errorTol){
    // test the factor in a physics scenario.
    // setup: setup a lowerBodyPoseEstimator with this factor and a minimal number of other factorNoiseModel
    // expectation (1): when you set all of the other factors' noise high, this should converge to a solution which indeed minimizes velocity difference
    // expectation (2): make sure that the equation for velocity of a joint gives results which are similar to the discrete time-series derivative of joint position
    // expectation (3): the error of the optimizer due to this factor is high initially, but near zero at the end
    // expectation (4): the global optimizer error should be near zero at the converged solution
    // () begin
    std::map<std::string,imu> ImuMap=imu::getImuMapFromDataFile(testutils::getTestDataFile("20170411-154746-Y1_TUG_6.h5"));
    //std::map<std::string,imu> ImuMap=imu::getImuMapFromDataFile(testutils::getTestDataFile("20170719-113432-Y13_TUG_13.h5"));
    // pull out imus
    imu sacrumImu=ImuMap["Sacrum"];
    imu rightThighImu=ImuMap["Right Thigh"];
    imu rightShankImu=ImuMap["Right Tibia"];
    imu rightFootImu=ImuMap["Right Foot"];
    imu leftThighImu=ImuMap["Left Thigh"];
    imu leftShankImu=ImuMap["Left Tibia"];
    imu leftFootImu=ImuMap["Left Foot"];
    // setup and run imu pose problems
    imuPoseEstimator sacrumImuPoseProblem(sacrumImu,"sacrum");
    sacrumImuPoseProblem.setImuBiasModelMode(1);
    sacrumImuPoseProblem.setup(); sacrumImuPoseProblem.setOptimizedValuesToInitialValues();
    imuPoseEstimator rthighImuPoseProblem(rightThighImu, "rthigh");
    rthighImuPoseProblem.setImuBiasModelMode(1);
    rthighImuPoseProblem.setup(); rthighImuPoseProblem.setOptimizedValuesToInitialValues();
    imuPoseEstimator rshankImuPoseProblem(rightShankImu, "rshank");
    rshankImuPoseProblem.setImuBiasModelMode(1);
    rshankImuPoseProblem.setup(); rshankImuPoseProblem.setOptimizedValuesToInitialValues();
    imuPoseEstimator rfootImuPoseProblem(rightFootImu, "rfoot");
    rfootImuPoseProblem.setImuBiasModelMode(1);
    rfootImuPoseProblem.setup(); rfootImuPoseProblem.setOptimizedValuesToInitialValues();
    imuPoseEstimator lthighImuPoseProblem(leftThighImu, "lthigh");
    lthighImuPoseProblem.setImuBiasModelMode(1);
    lthighImuPoseProblem.setup(); lthighImuPoseProblem.setOptimizedValuesToInitialValues();
    imuPoseEstimator lshankImuPoseProblem(leftShankImu, "lshank");
    lshankImuPoseProblem.setImuBiasModelMode(1);
    lshankImuPoseProblem.setup(); lshankImuPoseProblem.setOptimizedValuesToInitialValues();
    imuPoseEstimator lfootImuPoseProblem(leftFootImu, "lfoot");
    lfootImuPoseProblem.setImuBiasModelMode(1);
    lfootImuPoseProblem.setup(); lfootImuPoseProblem.setOptimizedValuesToInitialValues();
    // setup pose estimator
    lowerBodyPoseEstimator lbpe(sacrumImuPoseProblem,rthighImuPoseProblem,rshankImuPoseProblem,rfootImuPoseProblem,lthighImuPoseProblem,lshankImuPoseProblem,lfootImuPoseProblem,"test");
    lbpe.m_useJointVelFactor=true;
    lbpe.m_lambdaInitial=1.0e1;
    lbpe.m_convergeAtConsecSmallIter=1; // so it doesnt take that long
    lbpe.m_maxIterations=500;
    // () now before running setup, make all of the noises high!
    lbpe.m_kneeJointCtrConnectionNoise=gtsam::Vector3(6.7348, 12.3006, 10.1090)*1.0e-2*1.0e3; // std, meters
    lbpe.m_hipJointCtrConnectionNoise=gtsam::Vector3(9.135, 16.517 ,29.378)*1.0e-2*1.0e3; // std, meters
    lbpe.m_ankleJointCtrConnectionNoise=gtsam::Vector3(7.3741, 6.2212 ,4.6993)*1.0e-2*1.0e3; // std, meters
    lbpe.m_kneeHingeAxisNoise= gtsam::Vector3(1., 1., 1) * 1.0e3; // std, rad/s
    lbpe.m_rfemurLengthSigma=100.01, lbpe.m_rtibiaLengthSigma=100.01, lbpe.m_lfemurLengthSigma=100.01, lbpe.m_ltibiaLengthSigma=100.01, lbpe.m_pelvicWidthSigma=100.01; // std, meters
    // and set velocity factor noise really low
    lbpe.m_jointCtrVelConnectionNoise=gtsam::Vector3(1, 1, 1)*1.0e-4; // m/s
    // () now, setup and optimize
    lbpe.setup();
    lbpe.fastOptimize();
    // now testing expectation (1) above: pull out the RMSE of velocity difference for each joint. it should be nearly zero.
    std::vector<double> rhipJointVelErr(lbpe.m_sacrumImuAngVelKeys.size()), rhipJointVelSqErr(lbpe.m_sacrumImuAngVelKeys.size());
    std::vector<double> rkneeJointVelErr(lbpe.m_sacrumImuAngVelKeys.size()), rkneeJointVelSqErr(lbpe.m_sacrumImuAngVelKeys.size());
    std::vector<double> rankleJointVelErr(lbpe.m_sacrumImuAngVelKeys.size()), rankleJointVelSqErr(lbpe.m_sacrumImuAngVelKeys.size());
    std::vector<double> lhipJointVelErr(lbpe.m_sacrumImuAngVelKeys.size()), lhipJointVelSqErr(lbpe.m_sacrumImuAngVelKeys.size());
    std::vector<double> lkneeJointVelErr(lbpe.m_sacrumImuAngVelKeys.size()), lkneeJointVelSqErr(lbpe.m_sacrumImuAngVelKeys.size());
    std::vector<double> lankleJointVelErr(lbpe.m_sacrumImuAngVelKeys.size()), lankleJointVelSqErr(lbpe.m_sacrumImuAngVelKeys.size());
    for(uint k=0; k<lbpe.m_rthighImuPoseProblem.m_nKeyframes; k++) {
        rhipJointVelErr[k]=jointConnectionVelocityError(lbpe.m_sacrumImuPose[k], lbpe.m_sacrumImuVelocity[k],lbpe.m_sacrumImuAngVel[k], lbpe.m_sacrumImuToRHipCtr, lbpe.m_rthighImuPose[k], lbpe.m_rthighImuVelocity[k], lbpe.m_rthighImuAngVel[k], lbpe.m_rthighImuToHipCtr);
        rhipJointVelSqErr[k]=pow(rhipJointVelErr[k],2.0);
        rkneeJointVelErr[k]=jointConnectionVelocityError(lbpe.m_rthighImuPose[k], lbpe.m_rthighImuVelocity[k],lbpe.m_rthighImuAngVel[k], lbpe.m_rthighImuToKneeCtr, lbpe.m_rshankImuPose[k], lbpe.m_rshankImuVelocity[k], lbpe.m_rshankImuAngVel[k], lbpe.m_rshankImuToKneeCtr);
        rkneeJointVelSqErr[k]=pow(rkneeJointVelErr[k],2.0);
        rankleJointVelErr[k]=jointConnectionVelocityError(lbpe.m_rshankImuPose[k], lbpe.m_rshankImuVelocity[k],lbpe.m_rshankImuAngVel[k], lbpe.m_rshankImuToAnkleCtr, lbpe.m_rfootImuPose[k], lbpe.m_rfootImuVelocity[k], lbpe.m_rfootImuAngVel[k], lbpe.m_rfootImuToAnkleCtr);
        rankleJointVelSqErr[k]=pow(rankleJointVelErr[k],2.0);
        lhipJointVelErr[k]=jointConnectionVelocityError(lbpe.m_sacrumImuPose[k], lbpe.m_sacrumImuVelocity[k],lbpe.m_sacrumImuAngVel[k], lbpe.m_sacrumImuToLHipCtr, lbpe.m_lthighImuPose[k], lbpe.m_lthighImuVelocity[k], lbpe.m_lthighImuAngVel[k], lbpe.m_lthighImuToHipCtr);
        lhipJointVelSqErr[k]=pow(lhipJointVelErr[k],2.0);
        lkneeJointVelErr[k]=jointConnectionVelocityError(lbpe.m_lthighImuPose[k], lbpe.m_lthighImuVelocity[k],lbpe.m_lthighImuAngVel[k], lbpe.m_lthighImuToKneeCtr, lbpe.m_lshankImuPose[k], lbpe.m_lshankImuVelocity[k], lbpe.m_lshankImuAngVel[k], lbpe.m_lshankImuToKneeCtr);
        lkneeJointVelSqErr[k]=pow(lkneeJointVelErr[k],2.0);
        lankleJointVelErr[k]=jointConnectionVelocityError(lbpe.m_lshankImuPose[k], lbpe.m_lshankImuVelocity[k],lbpe.m_lshankImuAngVel[k], lbpe.m_lshankImuToAnkleCtr, lbpe.m_lfootImuPose[k], lbpe.m_lfootImuVelocity[k], lbpe.m_lfootImuAngVel[k], lbpe.m_lfootImuToAnkleCtr);
        lankleJointVelSqErr[k]=pow(lankleJointVelErr[k],2.0);
    }
    // now calculate RMSE of each joint's velocity error
    double rhipJointVelRMSE=sqrt(std::accumulate(rhipJointVelSqErr.begin(), rhipJointVelSqErr.end(), 0.0)/rhipJointVelSqErr.size());
    double rkneeJointVelRMSE=sqrt(std::accumulate(rkneeJointVelSqErr.begin(), rkneeJointVelSqErr.end(), 0.0)/rkneeJointVelSqErr.size());
    double rankleJointVelRMSE=sqrt(std::accumulate(rankleJointVelSqErr.begin(), rankleJointVelSqErr.end(), 0.0)/rankleJointVelSqErr.size());
    double lhipJointVelRMSE=sqrt(std::accumulate(lhipJointVelSqErr.begin(), lhipJointVelSqErr.end(), 0.0)/lhipJointVelSqErr.size());
    double lkneeJointVelRMSE=sqrt(std::accumulate(lkneeJointVelSqErr.begin(), lkneeJointVelSqErr.end(), 0.0)/lkneeJointVelSqErr.size());
    double lankleJointVelRMSE=sqrt(std::accumulate(lankleJointVelSqErr.begin(), lankleJointVelSqErr.end(), 0.0)/lankleJointVelSqErr.size());
    std::cout<<"joint velocity RMSEs: rhip="<<rhipJointVelRMSE<<", rknee="<<rkneeJointVelRMSE<<", rankle="<<rankleJointVelRMSE<<", lhip="<<lhipJointVelRMSE<<", lknee="<<lkneeJointVelRMSE<<", lankle="<<lankleJointVelRMSE<<" m/s"<<std::endl;
    if(rhipJointVelRMSE>errorTol || rkneeJointVelRMSE>errorTol || rankleJointVelRMSE>errorTol || lhipJointVelRMSE>errorTol || lkneeJointVelRMSE>errorTol || lankleJointVelRMSE>errorTol){
        std::cerr<<"at least one joint velocity RMSE is greater than the error tolerance ("<<errorTol<<")"<<std::endl;
        return 1;
    } // expectation 1 passes!
    // note: expectation 1 seems to pass here, but not in the MATLAB implementation. why?
    // () expectation 2: make sure the derivative of time series position is similar to velocity for each calculated joint velocity
    // PROBLEM with expectation 2: even for a single imuPoseEstimator the position and velocity don't align extremely well. So you would need that first before running expectation 2.
    int testExpectation2OK=lbpe.testJointCenterVelocityAsDiscretePositionDerivative(1.0e3);
    // () expectation 3:
    return 0;
}

int test_givenPriorsOn8Variables(uint numTests, double errorTol, const gtsam::SharedNoiseModel& factorNoiseModel, const gtsam::SharedNoiseModel& poseAPriorModel, const gtsam::SharedNoiseModel& linVelAPriorModel, const gtsam::SharedNoiseModel& angVelAPriorModel, const gtsam::SharedNoiseModel& vecAPriorModel,
                                 const gtsam::SharedNoiseModel& poseBPriorModel, const gtsam::SharedNoiseModel& linVelBPriorModel, const gtsam::SharedNoiseModel& angVelBPriorModel, const gtsam::SharedNoiseModel& vecBPriorModel){
    // this factor is a function of six variables. use this as a solution test where you vary all the permutations of tight/loose priors on the 6 variables to make sure it converges to the condition you want: velocity minimized between the two frames.
    std::cout<<"--- testing with 8 variables given priors ---"<<std::endl;
    // (note: when sigma=1, then optimizer error = 0.5*(factor error^2)
    for(uint i=0;i<numTests;i++){
        // generate two two zero poses, x1 and angVelB
        gtsam::Values myVals;
        gtsam::NonlinearFactorGraph mygraph;
        gtsam::Key linVelAKey=gtsam::Symbol('a', 0); gtsam::Key angVelAKey=gtsam::Symbol('b', 0); gtsam::Key vecAKey=gtsam::Symbol('c', 0);
        gtsam::Key linVelBKey=gtsam::Symbol('d', 0); gtsam::Key angVelBKey=gtsam::Symbol('e', 0); gtsam::Key vecBKey=gtsam::Symbol('f', 0);
        gtsam::Key poseAKey=gtsam::Symbol('g',0), poseBKey=gtsam::Symbol('h',0);
        bioslam::ConstrainedJointCenterNormVelocityFactor testFac=bioslam::ConstrainedJointCenterNormVelocityFactor(poseAKey, linVelAKey, angVelAKey, vecAKey, poseBKey, linVelBKey, angVelBKey, vecBKey, factorNoiseModel);
        mygraph.add(testFac);
        // generate random data for these six variables
        gtsam::Vector3 linVelA=testutils::randomVector3(), angVelA=testutils::randomVector3(), linVelB=testutils::randomVector3(), angVelB=testutils::randomVector3();
        gtsam::Point3 vecA=testutils::randomPoint3(), vecB=testutils::randomPoint3();
        gtsam::Pose3 poseA=testutils::randomPose3(), poseB=testutils::randomPose3();
        // put them in Values
        if(!myVals.exists(vecAKey)){
            myVals.insert(linVelAKey, linVelA); myVals.insert(angVelAKey, angVelA); myVals.insert(linVelBKey, linVelB); myVals.insert(angVelBKey, angVelB); myVals.insert(vecAKey,vecA); myVals.insert(vecBKey,vecB); myVals.insert(poseAKey,poseA); myVals.insert(poseBKey,poseB);
        }else{
            myVals.update(linVelAKey, linVelA); myVals.update(angVelAKey, angVelA); myVals.update(linVelBKey, linVelB); myVals.update(angVelBKey, angVelB); myVals.update(vecAKey,vecA); myVals.update(vecBKey,vecB); myVals.update(poseAKey,poseA); myVals.update(poseBKey,poseB);
        }
        // while we're at it, test derivative numerically
        //test_derivative_numerically(testFac, linVelA, angVelA, vecA, linVelB, angVelB, vecB);
        // verify error model
        //verifyErrorModel(poseA,linVelA,angVelA, vecA,poseB, linVelB, angVelB, vecB);
        // now add priors you input as arguments
        mygraph += gtsam::PriorFactor<gtsam::Vector3>(linVelAKey, linVelA, linVelAPriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Vector3>(angVelAKey, angVelA, angVelAPriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Point3>(vecAKey,vecA,vecAPriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Vector3>(linVelBKey, linVelB, linVelBPriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Vector3>(angVelBKey, angVelB, angVelBPriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Point3>(vecBKey,vecB,vecBPriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Pose3>(poseAKey,poseA,poseAPriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Pose3>(poseBKey,poseB,poseBPriorModel);
        // the inf problem is definitely with the velocity factor. for example, if you remove the velocity factor and add a simple between factor (see below),
        // the graph has finite cost
        //mygraph.add(gtsam::BetweenFactor<gtsam::Vector3>(linVelAKey,linVelBKey,gtsam::Vector3(0.,0.,0.),linVelAPriorModel));
        //mygraph.print();
        // optimizer setup. putting convergence criteria at extremes to force LM to give up.
        double relErrDecreaseLimit=1.0e-8; // convergence criteria for relative decrease in error
        double absErrDecreaseLimit=1.0e-8; // convergence criteria for absolute decrease in error
        int maxIterations=50; // maximum number of iterations
        gtsam::LevenbergMarquardtParams params; params.setVerbosityLM("SUMMARY");
        //params.setRelativeErrorTol(relErrDecreaseLimit); params.setAbsoluteErrorTol(absErrDecreaseLimit);
        gtsam::LevenbergMarquardtOptimizer optimizer(mygraph,myVals,params);
        double initialError=optimizer.error(), currentError=optimizer.error(), previousError, absErrorDecrease=9.0e9, relErrorDecrease=9.0e9;
        uint nIterations=0;
        double initialJointVelErr=jointConnectionVelocityError(poseA,linVelA,angVelA,vecA,poseB,linVelB,angVelB,vecB);
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
        gtsam::Vector3 est_linVelA=estimate.at<gtsam::Vector3>(linVelAKey);
        gtsam::Vector3 est_angVelA=estimate.at<gtsam::Vector3>(angVelAKey);
        gtsam::Vector3 est_linVelB=estimate.at<gtsam::Vector3>(linVelBKey);
        gtsam::Vector3 est_angVelB=estimate.at<gtsam::Vector3>(angVelBKey);
        gtsam::Point3 est_vecA=estimate.at<gtsam::Point3>(vecAKey);
        gtsam::Point3 est_vecB=estimate.at<gtsam::Point3>(vecBKey);
        gtsam::Pose3 est_poseA=estimate.at<gtsam::Pose3>(poseAKey);
        gtsam::Pose3 est_poseB=estimate.at<gtsam::Pose3>(poseBKey);
        double finalJointVelErr=jointConnectionVelocityError(est_poseA,est_linVelA,est_angVelA,est_vecA,est_poseB,est_linVelB,est_angVelB,est_vecB);
        std::cout<<"    test #"<<i<<": initial velocity error: "<<initialJointVelErr<<", final error: "<<finalJointVelErr<<"  |  optimizer error "<<initialError<<" --> "<<finalError<<" ("<<nIterations<<" iterations, with "<<mygraph.size()<<" factors and "<<myVals.size()<<" values)"<<std::endl;
        std::cout<<"        initial: linVelA=["<<linVelA.transpose()<<"], angVelA=["<<angVelA.transpose()<<"], vecA=["<<vecA.transpose()<<"] ==> jointVelA=["<<(jointVelocity(poseA,linVelA,angVelA,vecA)).transpose()<<"]"<<std::endl;
        std::cout<<"                 linVelB=["<<linVelB.transpose()<<"], angVelB=["<<angVelB.transpose()<<"], vecB=["<<vecB.transpose()<<"] ==> jointVelB=["<<(jointVelocity(poseB,linVelB,angVelB,vecB)).transpose()<<"]"<<std::endl;
        std::cout<<"                     ==> vel diff=["<<(jointVelocity(poseA,linVelA,angVelA,vecA)-jointVelocity(poseB,linVelB,angVelB,vecB)).transpose()<<"], norm="<<(jointVelocity(poseA,linVelA,angVelA,vecA)-jointVelocity(poseB,linVelB,angVelB,vecB)).norm()<<std::endl;
        std::cout<<"        optimized: linVelA=["<<est_linVelA.transpose()<<"], angVelA=["<<est_angVelA.transpose()<<"], vecA=["<<est_vecA.transpose()<<"] ==> jointVelA=["<<(jointVelocity(est_poseA,est_linVelA,est_angVelA,est_vecA)).transpose()<<"]"<<std::endl;
        std::cout<<"                   linVelB=["<<est_linVelB.transpose()<<"], angVelB=["<<est_angVelB.transpose()<<"], vecB=["<<est_vecB.transpose()<<"] ==> jointVelB=["<<(jointVelocity(est_poseB,est_linVelB,est_angVelB,est_vecB)).transpose()<<"]"<<std::endl;
        std::cout<<"                     ==> vel diff=["<<(jointVelocity(est_poseA,est_linVelA,est_angVelA,est_vecA)-jointVelocity(est_poseB,est_linVelB,est_angVelB,est_vecB)).transpose()<<"], norm="<<(jointVelocity(est_poseA,est_linVelA,est_angVelA,est_vecA)-jointVelocity(est_poseB,est_linVelB,est_angVelB,est_vecB)).norm()<<std::endl;
        if(finalError>errorTol){ // if greater than tolerance
            std::cerr<<"error: optimizer did not converge! final error: "<<finalError<<std::endl<<std::endl;
            return 2;
        }
        if(finalJointVelErr>errorTol){ // if greater than tolerance
            std::cerr<<"error: connection error is greater than "<<errorTol<<"! ("<<finalJointVelErr<<")"<<std::endl<<std::endl;
            return 3;
        }
        // now clear the graph
        myVals.clear();
    }
    return 0;
}

int verifyErrorModel(const gtsam::Pose3& poseA, const gtsam::Vector3& linVelA, const gtsam::Vector3& angVelA, const gtsam::Point3& vecA, const gtsam::Pose3& poseB, const gtsam::Vector3& linVelB, const gtsam::Vector3& angVelB, const gtsam::Point3& vecB){
    // to verify algebra, I'll create a residual e which is on the left hand side of these equations
    // according to Wenk and Frese ("Posture from Motion") a velocity model exists of:
    // linVelA+cross(angVelA,vecA) = R[B->A](linVelB + cross(angVelB,vecB))\
    // linVelA+cross(angVelA,vecA) = inv(R[A->N])*R[B->N](linVelB + cross(angVelB,vecB))   // noting that R[B->A]=inv(R[A->N])*R[B->N]
    gtsam::Vector3 errWenk=-linVelA-angVelA.cross(vecA) + poseA.rotation().inverse().matrix()*poseB.rotation().matrix()*(linVelB+angVelB.cross(vecB));
    // R[A->N](linVelA+cross(angVelA,vecA)) = R[B->N](linVelB + cross(angVelB,vecB))   // moving R[A->N] to left hand side
    gtsam::Vector3 err1=-poseA.rotation().matrix()*(linVelA+angVelA.cross(vecA)) + poseB.rotation().matrix()*(linVelB+angVelB.cross(vecB));
    std::cout<<"errWenk=["<<errWenk.transpose()<<"]"<<std::endl;
    std::cout<<"err1=["<<err1.transpose()<<"]"<<std::endl;
    return 0;
}

double jointConnectionVelocityError(const gtsam::Pose3& poseA, const gtsam::Vector3& linVelA, const gtsam::Vector3& angVelA, const gtsam::Point3& vecA, const gtsam::Pose3& poseB, const gtsam::Vector3& linVelB, const gtsam::Vector3& angVelB, const gtsam::Point3& vecB){
    gtsam::Vector3 jointVelA=jointVelocity(poseA,linVelA,angVelA,vecA);
    gtsam::Vector3 jointVelB=jointVelocity(poseB,linVelB,angVelB,vecB);
    gtsam::Vector3 velDiff=jointVelA-jointVelB;
    return velDiff.norm();
}

gtsam::Vector3 jointVelocity(const gtsam::Pose3& poseA, const gtsam::Vector3& linVelA, const gtsam::Vector3& angVelA, const gtsam::Point3& vecA){
    return linVelA+poseA.rotation().matrix()*(angVelA.cross(vecA));
}

int random_factor_tests(uint nTests){
    // in a loop, randomly generate inputs to the max factor and test derivatives numerically.
    gtsam::Vector3 myNoiseVec(1.0e-3,1.0e-3,1.0e-3);
    gtsam::SharedNoiseModel myNoiseModel=gtsam::noiseModel::Isotropic::Sigmas(myNoiseVec);
    gtsam::Values myVals;
    gtsam::NonlinearFactorGraph mygraph;
    gtsam::Key linVelAKey=gtsam::Symbol('a',0);
    gtsam::Key angVelAKey=gtsam::Symbol('b',0);
    gtsam::Key sAKey=gtsam::Symbol('c',0);
    gtsam::Key linVelBKey=gtsam::Symbol('d',0);
    gtsam::Key angVelBKey=gtsam::Symbol('e',0);
    gtsam::Key sBKey=gtsam::Symbol('f',0);
    gtsam::Key poseAKey=gtsam::Symbol('g',0), poseBKey=gtsam::Symbol('h',0);
    // (note: when sigma=1, then optimizer error = 0.5*(factor error^2)
    for(uint i=0;i<nTests;i++) {
        gtsam::Vector3 linVelA = testutils::randomVector3(), linVelB = testutils::randomVector3(), angVelA = testutils::randomVector3(), angVelB = testutils::randomVector3();
        gtsam::Vector3 sA=testutils::randomPoint3(),sB=testutils::randomPoint3();
        gtsam::Pose3 poseA=testutils::randomPose3(), poseB=testutils::randomPose3();
        if (!myVals.exists(linVelAKey)) {
            myVals.insert(linVelAKey, linVelA);
            myVals.insert(angVelAKey, angVelA);
            myVals.insert(sAKey, sA);
            myVals.insert(linVelBKey, linVelB);
            myVals.insert(angVelBKey, angVelB);
            myVals.insert(sBKey, sB);
            myVals.insert(poseAKey,poseA);
            myVals.insert(poseBKey,poseB);
        } else {
            myVals.update(linVelAKey, linVelA);
            myVals.update(angVelAKey, angVelA);
            myVals.update(sAKey, sA);
            myVals.update(linVelBKey, linVelB);
            myVals.update(angVelBKey, angVelB);
            myVals.update(sBKey, sB);
            myVals.update(poseAKey,poseA);
            myVals.update(poseBKey,poseB);
        }
        bioslam::ConstrainedJointCenterNormVelocityFactor testFac=bioslam::ConstrainedJointCenterNormVelocityFactor(poseAKey, linVelAKey, angVelAKey, sAKey, poseBKey, linVelBKey, angVelBKey, sBKey, myNoiseModel);
        // test derivative numerically
        test_derivative_numerically(testFac, poseA,linVelA, angVelA, sA, poseB,linVelB, angVelB, sB);
    }
    return 0;
}

int test_derivative_numerically(const bioslam::ConstrainedJointCenterNormVelocityFactor& fac, const gtsam::Pose3& poseA, const gtsam::Vector3& linVelA, const gtsam::Vector3& angVelA, const gtsam::Point3& sA, const gtsam::Pose3& poseB, const gtsam::Vector3& linVelB, const gtsam::Vector3& angVelB, const gtsam::Point3& sB){
    // unit test the Jacobian against GTSAM's numerical derivative
    // this may seem silly for such a simple factor, but it's important to do.
    // () get derived error and jacobians
    gtsam::Matrix derivedH1, derivedH2, derivedH3, derivedH4, derivedH5, derivedH6, derivedH7, derivedH8;
    gtsam::Vector derivedErr=fac.evaluateError(poseA,linVelA,angVelA,sA,poseB,linVelB,angVelB,sB,derivedH1, derivedH2, derivedH3, derivedH4, derivedH5, derivedH6,derivedH7,derivedH8);
    // () get numerical jacobians
    //    I think to call it it's numericalDerivativeXY where X=number of input variables and Y=which Jacobian you want to test
    //    templates are: <output type (typically gtsam::Vector), then the input argument types in order)
    gtsam::Matrix numericalH1=gtsamutils::numericalDerivative81<gtsam::Vector,gtsam::Pose3,gtsam::Vector3,gtsam::Vector3,gtsam::Point3,gtsam::Pose3,gtsam::Vector3,gtsam::Vector3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Vector3&, const gtsam::Point3&, const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Vector3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::ConstrainedJointCenterNormVelocityFactor::evaluateErrorNoJacCall, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5,std::placeholders::_6,std::placeholders::_7,std::placeholders::_8)), poseA, linVelA, angVelA, sA, poseB, linVelB, angVelB, sB, 1e-5);
    gtsam::Matrix numericalH2=gtsamutils::numericalDerivative82<gtsam::Vector,gtsam::Pose3,gtsam::Vector3,gtsam::Vector3,gtsam::Point3,gtsam::Pose3,gtsam::Vector3,gtsam::Vector3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Vector3&, const gtsam::Point3&, const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Vector3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::ConstrainedJointCenterNormVelocityFactor::evaluateErrorNoJacCall, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5,std::placeholders::_6,std::placeholders::_7,std::placeholders::_8)), poseA, linVelA, angVelA, sA, poseB, linVelB, angVelB, sB, 1e-5);
    gtsam::Matrix numericalH3=gtsamutils::numericalDerivative83<gtsam::Vector,gtsam::Pose3,gtsam::Vector3,gtsam::Vector3,gtsam::Point3,gtsam::Pose3,gtsam::Vector3,gtsam::Vector3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Vector3&, const gtsam::Point3&, const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Vector3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::ConstrainedJointCenterNormVelocityFactor::evaluateErrorNoJacCall, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5,std::placeholders::_6,std::placeholders::_7,std::placeholders::_8)), poseA, linVelA, angVelA, sA, poseB, linVelB, angVelB, sB, 1e-5);
    gtsam::Matrix numericalH4=gtsamutils::numericalDerivative84<gtsam::Vector,gtsam::Pose3,gtsam::Vector3,gtsam::Vector3,gtsam::Point3,gtsam::Pose3,gtsam::Vector3,gtsam::Vector3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Vector3&, const gtsam::Point3&, const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Vector3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::ConstrainedJointCenterNormVelocityFactor::evaluateErrorNoJacCall, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5,std::placeholders::_6,std::placeholders::_7,std::placeholders::_8)), poseA, linVelA, angVelA, sA, poseB, linVelB, angVelB, sB, 1e-5);
    gtsam::Matrix numericalH5=gtsamutils::numericalDerivative85<gtsam::Vector,gtsam::Pose3,gtsam::Vector3,gtsam::Vector3,gtsam::Point3,gtsam::Pose3,gtsam::Vector3,gtsam::Vector3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Vector3&, const gtsam::Point3&, const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Vector3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::ConstrainedJointCenterNormVelocityFactor::evaluateErrorNoJacCall, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5,std::placeholders::_6,std::placeholders::_7,std::placeholders::_8)), poseA, linVelA, angVelA, sA, poseB, linVelB, angVelB, sB, 1e-5);
    gtsam::Matrix numericalH6=gtsamutils::numericalDerivative86<gtsam::Vector,gtsam::Pose3,gtsam::Vector3,gtsam::Vector3,gtsam::Point3,gtsam::Pose3,gtsam::Vector3,gtsam::Vector3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Vector3&, const gtsam::Point3&, const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Vector3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::ConstrainedJointCenterNormVelocityFactor::evaluateErrorNoJacCall, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5,std::placeholders::_6,std::placeholders::_7,std::placeholders::_8)), poseA, linVelA, angVelA, sA, poseB, linVelB, angVelB, sB, 1e-5);
    gtsam::Matrix numericalH7=gtsamutils::numericalDerivative87<gtsam::Vector,gtsam::Pose3,gtsam::Vector3,gtsam::Vector3,gtsam::Point3,gtsam::Pose3,gtsam::Vector3,gtsam::Vector3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Vector3&, const gtsam::Point3&, const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Vector3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::ConstrainedJointCenterNormVelocityFactor::evaluateErrorNoJacCall, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5,std::placeholders::_6,std::placeholders::_7,std::placeholders::_8)), poseA, linVelA, angVelA, sA, poseB, linVelB, angVelB, sB, 1e-5);
    gtsam::Matrix numericalH8=gtsamutils::numericalDerivative88<gtsam::Vector,gtsam::Pose3,gtsam::Vector3,gtsam::Vector3,gtsam::Point3,gtsam::Pose3,gtsam::Vector3,gtsam::Vector3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Vector3&, const gtsam::Point3&, const gtsam::Pose3&, const gtsam::Vector3&, const gtsam::Vector3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::ConstrainedJointCenterNormVelocityFactor::evaluateErrorNoJacCall, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5,std::placeholders::_6,std::placeholders::_7,std::placeholders::_8)), poseA, linVelA, angVelA, sA, poseB, linVelB, angVelB, sB, 1e-5);

    // now test using gtsam::assert_equal()
    bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-7);
    bool testH2=gtsam::assert_equal(derivedH2,numericalH2,1e-7);
    bool testH3=gtsam::assert_equal(derivedH3,numericalH3,1e-7);
    bool testH4=gtsam::assert_equal(derivedH4,numericalH4,1e-7);
    bool testH5=gtsam::assert_equal(derivedH5,numericalH5,1e-7);
    bool testH6=gtsam::assert_equal(derivedH6,numericalH6,1e-7);
    bool testH7=gtsam::assert_equal(derivedH7,numericalH7,1e-7);
    bool testH8=gtsam::assert_equal(derivedH8,numericalH8,1e-7);
    if (!testH1){
        std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
        throw std::runtime_error("Jacobian check numerically failed.");
        return 1;
    }
    if (!testH2){
        std::cerr<<"H2 did not check out numerically."<<std::endl<<"derivedH2="<<derivedH2<<std::endl<<"numericalH2"<<numericalH2<<std::endl;
        throw std::runtime_error("Jacobian check numerically failed.");
        return 1;
    }
    if (!testH3){
        std::cerr<<"H3 did not check out numerically."<<std::endl<<"derivedH3="<<derivedH3<<std::endl<<"numericalH3"<<numericalH3<<std::endl;
        throw std::runtime_error("Jacobian check numerically failed.");
        return 1;
    }
    if (!testH4){
        std::cerr<<"H4 did not check out numerically."<<std::endl<<"derivedH4="<<derivedH4<<std::endl<<"numericalH4"<<numericalH4<<std::endl;
        throw std::runtime_error("Jacobian check numerically failed.");
        return 1;
    }
    if (!testH5){
        std::cerr<<"H5 did not check out numerically."<<std::endl<<"derivedH5="<<derivedH5<<std::endl<<"numericalH5"<<numericalH5<<std::endl;
        throw std::runtime_error("Jacobian check numerically failed.");
        return 1;
    }
    if (!testH6){
        std::cerr<<"H6 did not check out numerically."<<std::endl<<"derivedH6="<<derivedH6<<std::endl<<"numericalH6"<<numericalH6<<std::endl;
        throw std::runtime_error("Jacobian check numerically failed.");
        return 1;
    }
    if (!testH7){
        std::cerr<<"H7 did not check out numerically."<<std::endl<<"derivedH7="<<derivedH7<<std::endl<<"numericalH7"<<numericalH7<<std::endl;
        return 1;
    }
    if (!testH8){
        std::cerr<<"H8 did not check out numerically."<<std::endl<<"derivedH8="<<derivedH8<<std::endl<<"numericalH8"<<numericalH8<<std::endl;
        throw std::runtime_error("Jacobian check numerically failed.");
        return 1;
    }
    return 0;
}

/*
   Some intuition on frames courtesy of gtsam::NavState
       in NavState::bodyVelocity(), the body velocity is calculated as: Vector3 b_v = nRb.unrotate(n_v, H ? &D_bv_nRb : 0);
       and of course we know that nRb is GTSAM's notation for R[B->N]. This confirms that velocity, like position, is represented in the navigation frame inherently.

  And on the NavState model, looking at NavState::update(accel,gyro,dt)
      (here accel and gyro are body frame measurements)
  The integration model is:
  deltaRot = dt * omega; (Tim: i.e., this should be R[B->B2])
  deltaPos = dt * b_v + 1/2*dt^2 * accel; (where b_v is the body velocity from NavState::bodyVelocity())
  deltaVel = dt * accel;

 */