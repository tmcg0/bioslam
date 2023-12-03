// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// unit test of ConstrainedJointCenterNormPositionFactor
// it should include the following tests:
// 1) the previous tests: consider all 4 variables freely.
// 2-5) now, go through the four variables and strong priors on each one individually, rerun tests
// 6,7) set strong priors on BOTH poses and then BOTH vectors
// 8-11) allow only one variable to be loose

#include <gtsam/geometry/Pose3.h>
#include <factors/ConstrainedJointCenterPositionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Testable.h>
#include "testutils.h"

double jointConnectionError(const gtsam::Pose3& x1, const gtsam::Pose3& x2, const gtsam::Point3& v1, const gtsam::Point3& v2);
int assertJointCtrCalcMethodsSame();
gtsam::Point3 jointCtrFromImuPoseAndVec_SO3_R3(const gtsam::Pose3& x1, const gtsam::Point3& v1);
gtsam::Point3 jointCtrFromImuPoseAndVec_SE3(const gtsam::Pose3& x1, const gtsam::Point3& v1);
void printStateOnError(const gtsam::Pose3& x1, const gtsam::Pose3& x2, const gtsam::Point3& v1, const gtsam::Point3& v2);
int test_givenPriorsOn4Variables(bool randExpectedDist, bool useRandomPoses, uint numTests, double errorTol, const gtsam::SharedNoiseModel& factorNoiseModel,
                                 const gtsam::SharedNoiseModel& poseAPriorModel, const gtsam::SharedNoiseModel& poseBPriorModel,
                                 const gtsam::SharedNoiseModel& vecAPriorModel, const gtsam::SharedNoiseModel& vecBPriorModel);
int test_derivative_numerically(const bioslam::ConstrainedJointCenterNormPositionFactor& fac, const gtsam::Pose3 &xA, const gtsam::Pose3 &xB, const gtsam::Point3 &L_A_to_ctr, const gtsam::Point3 &L_B_to_ctr);

int main(){
    // first confirm that how you calculate the joint ctr is equivalent from the SE(3) formulation vs. the separated SO(3)/R(3) formulation
    assertJointCtrCalcMethodsSame();
    //
    gtsam::Vector3 noiseVec; noiseVec<<1.0e-3,1.0e-3,1.0e-3;
    gtsam::SharedNoiseModel myNoiseModel=gtsam::noiseModel::Isotropic::Sigmas(gtsam::Vector1(noiseVec.norm()),true);
    gtsam::noiseModel::Diagonal::shared_ptr tightPose = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1.0e-6,1.0e-6,1.0e-6,1.0e-6,1.0e-6,1.0e-6).finished());
    gtsam::noiseModel::Diagonal::shared_ptr loosePose = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1.0e6,1.0e6,1.0e6,1.0e6,1.0e6,1.0e6).finished());
    gtsam::noiseModel::Diagonal::shared_ptr tightVec = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 1.0e-6,1.0e-6,1.0e-6).finished());
    gtsam::noiseModel::Diagonal::shared_ptr looseVec = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 1.0e6,1.0e6,1.0e6).finished());

    int test0= test_givenPriorsOn4Variables(true,false,50,0.001,myNoiseModel,loosePose,loosePose,looseVec,looseVec);
    if(test0!=0){ throw std::runtime_error("test failed."); }
    int test1= test_givenPriorsOn4Variables(true,true,50,0.001,myNoiseModel,loosePose,loosePose,looseVec,looseVec);
    if(test1!=0){ throw std::runtime_error("test failed."); }
    int test2= test_givenPriorsOn4Variables(true,true,50,0.001,myNoiseModel,tightPose,loosePose,looseVec,looseVec);
    if(test2!=0){ throw std::runtime_error("test failed."); }
    int test3= test_givenPriorsOn4Variables(true,true,50,0.001,myNoiseModel,loosePose,tightPose,looseVec,looseVec);
    if(test3!=0){ throw std::runtime_error("test failed."); }
    int test4= test_givenPriorsOn4Variables(true,true,50,0.001,myNoiseModel,loosePose,loosePose,tightVec,looseVec);
    if(test4!=0){ throw std::runtime_error("test failed."); }
    int test5= test_givenPriorsOn4Variables(true,true,50,0.001,myNoiseModel,loosePose,loosePose,looseVec,tightVec);
    if(test5!=0){ throw std::runtime_error("test failed."); }
    int test6= test_givenPriorsOn4Variables(true,true,50,0.001,myNoiseModel,tightPose,tightPose,looseVec,looseVec);
    if(test6!=0){ throw std::runtime_error("test failed."); }
    int test7= test_givenPriorsOn4Variables(true,true,50,0.001,myNoiseModel,loosePose,loosePose,tightVec,tightVec);
    if(test7!=0){ throw std::runtime_error("test failed."); }
    int test8= test_givenPriorsOn4Variables(true,true,50,0.001,myNoiseModel,loosePose,tightPose,tightVec,tightVec);
    if(test8!=0){ throw std::runtime_error("test failed."); }
    int test9= test_givenPriorsOn4Variables(true,true,50,0.001,myNoiseModel,tightPose,loosePose,tightVec,tightVec);
    if(test9!=0){ throw std::runtime_error("test failed."); }
    int test10= test_givenPriorsOn4Variables(true,true,50,0.001,myNoiseModel,tightPose,tightPose,looseVec,tightVec);
    if(test10!=0){ throw std::runtime_error("test failed."); }
    int test11= test_givenPriorsOn4Variables(true,true,50,0.001,myNoiseModel,tightPose,tightPose,tightVec,looseVec);
    if(test11!=0){ throw std::runtime_error("test failed."); }
    int testx= test_givenPriorsOn4Variables(true,true,50,0.001,myNoiseModel,tightPose,tightPose,tightVec,looseVec);
    // this following test should fail: you have strong priors on all variables
    // int testFail = test_givenPriorsOn4Variables(true,true,50,0.001,myNoiseModel,tightPose,tightPose,tightVec,tightVec);
    return 0;
}

int test_derivative_numerically(const bioslam::ConstrainedJointCenterNormPositionFactor& fac, const gtsam::Pose3 &xA, const gtsam::Pose3 &xB, const gtsam::Point3 &L_A_to_ctr, const gtsam::Point3 &L_B_to_ctr){
    // unit test the Jacobian against GTSAM's numerical derivative
    // () get derived error and jacobians
    gtsam::Matrix derivedH1, derivedH2, derivedH3, derivedH4;
    gtsam::Vector derivedErr=fac.evaluateError(xA,xB,L_A_to_ctr,L_B_to_ctr,derivedH1, derivedH2, derivedH3, derivedH4);
    // () get numerical jacobians
    //    I think to call it it's numericalDerivativeXY where X=number of input variables and Y=which Jacobian you want to test
    //    templates are: <output type (typically gtsam::Vector), then the input argument types in order)
    gtsam::Matrix numericalH1=gtsam::numericalDerivative41<gtsam::Vector,gtsam::Pose3,gtsam::Pose3,gtsam::Point3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Pose3&, const gtsam::Point3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::ConstrainedJointCenterNormPositionFactor::evaluateError, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4, boost::none, boost::none, boost::none, boost::none)), xA, xB, L_A_to_ctr, L_B_to_ctr, 1e-5);
    gtsam::Matrix numericalH2=gtsam::numericalDerivative42<gtsam::Vector,gtsam::Pose3,gtsam::Pose3,gtsam::Point3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Pose3&, const gtsam::Point3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::ConstrainedJointCenterNormPositionFactor::evaluateError, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4, boost::none, boost::none, boost::none, boost::none)), xA, xB, L_A_to_ctr, L_B_to_ctr, 1e-5);
    gtsam::Matrix numericalH3=gtsam::numericalDerivative43<gtsam::Vector,gtsam::Pose3,gtsam::Pose3,gtsam::Point3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Pose3&, const gtsam::Point3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::ConstrainedJointCenterNormPositionFactor::evaluateError, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4, boost::none, boost::none, boost::none, boost::none)), xA, xB, L_A_to_ctr, L_B_to_ctr, 1e-5);
    gtsam::Matrix numericalH4=gtsam::numericalDerivative44<gtsam::Vector,gtsam::Pose3,gtsam::Pose3,gtsam::Point3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Pose3&, const gtsam::Point3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::ConstrainedJointCenterNormPositionFactor::evaluateError, fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4, boost::none, boost::none, boost::none, boost::none)), xA, xB, L_A_to_ctr, L_B_to_ctr, 1e-5);
    // now test using gtsam::assert_equal()
    bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-9);
    bool testH2=gtsam::assert_equal(derivedH2,numericalH2,1e-9);
    bool testH3=gtsam::assert_equal(derivedH3,numericalH3,1e-9);
    bool testH4=gtsam::assert_equal(derivedH4,numericalH4,1e-9);
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
    return 0;
}

int test_givenPriorsOn4Variables(bool randExpectedDist, bool useRandomPoses, uint numTests, double errorTol, const gtsam::SharedNoiseModel& factorNoiseModel,
        const gtsam::SharedNoiseModel& poseAPriorModel, const gtsam::SharedNoiseModel& poseBPriorModel,
        const gtsam::SharedNoiseModel& vecAPriorModel, const gtsam::SharedNoiseModel& vecBPriorModel){
    std::cout<<"--- testing with 4 variables given priors ---"<<std::endl;
    // (note: when sigma=1, then optimizer error = 0.5*(factor error^2)
    for(uint i=0;i<numTests;i++){
        // random expected dist (overriding default zero)
        double expectedDist=0.0;
        if(randExpectedDist){
            expectedDist=testutils::dRand(0.0,10.0);
        }
        // generate two two zero poses, x1 and x2
        gtsam::Values myVals;
        gtsam::NonlinearFactorGraph mygraph;
        gtsam::Key x1Key=gtsam::Symbol('a',0); gtsam::Key x2Key=gtsam::Symbol('b',0); gtsam::Key v1Key=gtsam::Symbol('c',0); gtsam::Key v2Key=gtsam::Symbol('d',0);
        bioslam::ConstrainedJointCenterNormPositionFactor testFac=bioslam::ConstrainedJointCenterNormPositionFactor(x1Key, x2Key, v1Key, v2Key, expectedDist, factorNoiseModel);
        mygraph.add(testFac);
        gtsam::Pose3 x1, x2;
        if(useRandomPoses){
            x1 = testutils::randomPose3();
            x2 = testutils::randomPose3();
        }else { // use zero poses
            x1 = gtsam::Pose3(gtsam::Rot3(1., 0., 0., 0.), gtsam::Point3(0., 0., 0.));
            x2 = gtsam::Pose3(gtsam::Rot3(1., 0., 0., 0.), gtsam::Point3(0., 0., 0.));
        }
        // generate two random vectors, v1 and v2
        gtsam::Point3 v1=testutils::randomPoint3();
        gtsam::Point3 v2=testutils::randomPoint3();
        if(!myVals.exists(v1Key)){
            myVals.insert(x1Key,x1); myVals.insert(x2Key,x2);
            myVals.insert(v1Key,v1); myVals.insert(v2Key,v2);
        }else{
            myVals.update(x1Key,x1); myVals.update(x2Key,x2);
            myVals.update(v1Key,v1); myVals.update(v2Key,v2);
        }
        // now test derivative numerically
        test_derivative_numerically(testFac, x1, x2, v1, v2);
        // now add priors
        mygraph += gtsam::PriorFactor<gtsam::Pose3>(x1Key, x1, poseAPriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Pose3>(x2Key, x2, poseBPriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Point3>(v1Key,v1,vecAPriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Point3>(v2Key,v2,vecBPriorModel);
        // optimizer setup
        double relErrDecreaseLimit=1.0e-5; // convergence criteria for relative decrease in error
        double absErrDecreaseLimit=1.0e-7; // convergence criteria for absolute decrease in error
        int maxIterations=500; // maximum number of iterations
        gtsam::LevenbergMarquardtOptimizer optimizer(mygraph,myVals);
        double initialError=optimizer.error();
        double currentError=optimizer.error(), errorInit=currentError, previousError, absErrorDecrease=9.0e9, relErrorDecrease=9.0e9;
        uint nIterations=0;
        double initialConnectionError=jointConnectionError(x1,x2,v1,v2);
        while(relErrorDecrease>relErrDecreaseLimit && absErrorDecrease>absErrDecreaseLimit && optimizer.iterations()<maxIterations){
            // set previous error values
            previousError=currentError;
            // perform iteration and checks
            boost::shared_ptr<gtsam::GaussianFactorGraph> g=optimizer.iterate(); // should have updated things in here now
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
        gtsam::Pose3 est_x1=estimate.at<gtsam::Pose3>(x1Key);
        gtsam::Pose3 est_x2=estimate.at<gtsam::Pose3>(x2Key);
        gtsam::Point3 est_v1=estimate.at<gtsam::Point3>(v1Key);
        gtsam::Point3 est_v2=estimate.at<gtsam::Point3>(v2Key);
        double finalConnectionError=jointConnectionError(est_x1,est_x2,est_v1,est_v2);
        std::cout<<"    test #"<<i<<" (expectedDist="<<expectedDist<<"): initial connection error: "<<initialConnectionError<<", final error: "<<finalConnectionError<<"  |  optimizer error "<<initialError<<" --> "<<finalError<<" ("<<nIterations<<" iterations, with "<<mygraph.size()<<" factors and "<<myVals.size()<<" values)"<<std::endl;
        std::cout<<"        initial: q1=["<<x1.rotation().quaternion().transpose()<<"], p1=["<<x1.translation().transpose()<<"], v1=["<<v1.transpose()<<"] | q2=["<<x2.rotation().quaternion().transpose()<<"], p2=["<<x2.translation().transpose()<<"], v2=["<<v2.transpose()<<"]"<<std::endl;
        std::cout<<"        optimized: q1=["<<est_x1.rotation().quaternion().transpose()<<"], p1=["<<est_x1.translation().transpose()<<"], v1=["<<est_v1.transpose()<<"] | q2=["<<est_x2.rotation().quaternion().transpose()<<"], p2=["<<est_x2.translation().transpose()<<"], v2=["<<est_v2.transpose()<<"]"<<std::endl;
        // can find the residual in MATLAB with: (quatrotate(quatinv(q1),v1)+p1)-(quatrotate(quatinv(q2),v2)+p2)
        //     why do I have to use quatinv() here?
        if(abs(expectedDist-finalConnectionError)>errorTol){ // if greater than 1mm
            std::cerr<<"error: connection error is greater than "<<errorTol<<"! ("<<abs(expectedDist-finalConnectionError)<<")"<<std::endl<<std::endl;
            printStateOnError(est_x1, est_x2, est_v1, est_v2);
            return 1;
        }
        // now clear the graph
        myVals.clear();
    }
    return 0;
}

void printStateOnError(const gtsam::Pose3& x1, const gtsam::Pose3& x2, const gtsam::Point3& v1, const gtsam::Point3& v2){
    gtsam::Point3 jointCenterAccordingTo1=x1.rotation()*v1 + x1.translation(); // R[B->N]*v1[B] + p[N]
    gtsam::Point3 jointCenterAccordingTo2=x2.rotation()*v2 + x2.translation(); // R[B->N]*v2[B] + p[N]
    gtsam::Point3 jointConnectionDiff=jointCenterAccordingTo1-jointCenterAccordingTo2;
    std::cout<<"joint center according to first IMU: "<<std::endl<<"R="<<std::endl<<x1.rotation().matrix()<<std::endl<<"*v="<<std::endl<<v1.transpose()<<std::endl<<"+p="<<std::endl<<x1.translation().transpose()<<std::endl<<"-------------------------"<<std::endl<<jointCenterAccordingTo1.transpose()<<std::endl;
    std::cout<<std::endl<<"joint center according to second IMU: "<<std::endl<<"R="<<std::endl<<x2.rotation().matrix()<<std::endl<<"*v="<<std::endl<<v2.transpose()<<std::endl<<"+p="<<std::endl<<x2.translation().transpose()<<std::endl<<"-------------------------"<<std::endl<<jointCenterAccordingTo2.transpose()<<std::endl;
    std::cout<<std::endl<<"for a resulting difference of: "<<jointConnectionDiff.transpose()<<" (norm="<<jointConnectionDiff.norm()<<")"<<std::endl;
}

double jointConnectionError(const gtsam::Pose3& x1, const gtsam::Pose3& x2, const gtsam::Point3& v1, const gtsam::Point3& v2){
    // for a single set of poses and vectors, figure out the magnitude of the error (m)
    // poses are assumed R[B->N], vectors are in body frame B
    gtsam::Point3 jointCenterAccordingTo1=x1.rotation()*v1 + x1.translation(); // R[B->N]*v1[B] + p[N]
    gtsam::Point3 jointCenterAccordingTo2=x2.rotation()*v2 + x2.translation(); // R[B->N]*v2[B] + p[N]
    gtsam::Point3 jointConnectionDiff=jointCenterAccordingTo1-jointCenterAccordingTo2;
    double errorNorm=jointConnectionDiff.norm();
    return errorNorm;
}

int assertJointCtrCalcMethodsSame(){
    gtsam::Pose3 T=testutils::randomPose3();
    gtsam::Point3 p=testutils::randomPoint3();
    gtsam::Point3 calc1=jointCtrFromImuPoseAndVec_SO3_R3(T,p);
    gtsam::Point3 calc2=jointCtrFromImuPoseAndVec_SE3(T,p);
    if(!gtsam::assert_equal(calc1,calc2)){
        std::cerr<<"ERROR: joint ctr calc methods are not the same!"<<std::endl;
        std::cerr<<"calc1="<<calc1.transpose()<<std::endl;
        std::cerr<<"calc2="<<calc2.transpose()<<std::endl;
        return 1;
    }
    return 0;
}

gtsam::Point3 jointCtrFromImuPoseAndVec_SO3_R3(const gtsam::Pose3& x1, const gtsam::Point3& v1){
    // compute the joint ctr from the individual components of the SE(3) pose
    gtsam::Point3 jointCenterAccordingTo1=x1.rotation()*v1 + x1.translation(); // R[B->N]*v1[B] + p[N]
    return jointCenterAccordingTo1;
}

gtsam::Point3 jointCtrFromImuPoseAndVec_SE3(const gtsam::Pose3 &x1, const gtsam::Point3& v1){
    // compute the joint ctr from the individual components of the SE(3) pose
    gtsam::Point3 jointCenterAccordingTo1=x1.transformFrom(v1);
    // correct is x1.transformFrom(v1)!
    return jointCenterAccordingTo1;
}