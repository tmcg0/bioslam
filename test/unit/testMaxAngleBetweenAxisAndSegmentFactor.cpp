// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// unit test of the factor of the same name. unit test framework:
// 1) do the jacobians check out numerically?
// 2) optimizer tests: make sure that you can run through the permutations of tight/loose priors and still get the expected minimization.

#include "testutils.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/numericalDerivative.h>
#include <factors/AngleBetweenAxisAndSegmentFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

int test_derivative_numerically(const bioslam::MaxAngleBetweenAxisAndSegmentFactor& fac, const gtsam::Unit3& axis, const gtsam::Point3& v1, const gtsam::Point3& v2);
int random_factor_tests(uint nTests);
double angleBwVectors(const gtsam::Unit3& axis, const gtsam::Point3& v1, const gtsam::Point3& v2);
int test_givenPriorsOn3Variables(uint numTests, double errorTol, const gtsam::SharedNoiseModel& factorNoiseModel,
                                 const gtsam::SharedNoiseModel& axisPriorModel, const gtsam::SharedNoiseModel& v1PriorModel,const gtsam::SharedNoiseModel& v2PriorModel);

int main(){
    random_factor_tests(500);
    // now do permutations of tight/loose priors on variables
    gtsam::Vector1 noiseVec; noiseVec<<1.0e-3;
    gtsam::SharedNoiseModel myNoiseModel=gtsam::noiseModel::Isotropic::Sigmas(noiseVec);
    gtsam::noiseModel::Diagonal::shared_ptr tightAxis = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(2) << 1.0e-6,1.0e-6).finished());
    gtsam::noiseModel::Diagonal::shared_ptr looseAxis = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(2) << 1.0e6,1.0e6).finished());
    gtsam::noiseModel::Diagonal::shared_ptr tightVec = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 1.0e-6,1.0e-6,1.0e-6).finished());
    gtsam::noiseModel::Diagonal::shared_ptr looseVec = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 1.0e6,1.0e6,1.0e6).finished());
    // run those permutations here:
    test_givenPriorsOn3Variables(50,1.0e-3,myNoiseModel,looseAxis,looseVec,looseVec);
    test_givenPriorsOn3Variables(50,1.0e-3,myNoiseModel,looseAxis,tightVec,tightVec);
    test_givenPriorsOn3Variables(50,1.0e-3,myNoiseModel,tightAxis,looseVec,tightVec);
    test_givenPriorsOn3Variables(50,1.0e-3,myNoiseModel,tightAxis,tightVec,looseVec);
    return 0;
}


int test_givenPriorsOn3Variables(uint numTests, double errorTol, const gtsam::SharedNoiseModel& factorNoiseModel,
                                 const gtsam::SharedNoiseModel& axisPriorModel, const gtsam::SharedNoiseModel& v1PriorModel,const gtsam::SharedNoiseModel& v2PriorModel){
    std::cout<<"--- testing with 3 variables given priors ---"<<std::endl;
    // (note: when sigma=1, then optimizer error = 0.5*(factor error^2)
    for(uint i=0;i<numTests;i++){
        // generate two two zero poses, x1 and x2
        gtsam::Values myVals;
        gtsam::NonlinearFactorGraph mygraph;
        gtsam::Key axisKey=gtsam::Symbol('a',0); gtsam::Key v1Key=gtsam::Symbol('c',0); gtsam::Key v2Key=gtsam::Symbol('d',0);
        double maxang=0.2*(M_PI/2);
        bioslam::MaxAngleBetweenAxisAndSegmentFactor testFac=bioslam::MaxAngleBetweenAxisAndSegmentFactor(axisKey, v1Key, v2Key, maxang, factorNoiseModel);
        mygraph.add(testFac);
        gtsam::Unit3 axis = testutils::randomUnit3();
        // generate two random vectors, v1 and v2
        gtsam::Point3 v1=testutils::randomPoint3(), v2=testutils::randomPoint3();
        if(!myVals.exists(v1Key)){
            myVals.insert(axisKey,axis);
            myVals.insert(v1Key,v1); myVals.insert(v2Key,v2);
        }else{
            myVals.update(axisKey,axis);
            myVals.update(v1Key,v1); myVals.update(v2Key,v2);
        }
        // now test derivative numerically
        test_derivative_numerically(testFac, axis, v1, v2);
        // now add priors
        mygraph += gtsam::PriorFactor<gtsam::Unit3>(axisKey, axis, axisPriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Point3>(v1Key,v1,v1PriorModel);
        mygraph += gtsam::PriorFactor<gtsam::Point3>(v2Key,v2,v2PriorModel);
        // optimizer setup
        double relErrDecreaseLimit=1.0e-5; // convergence criteria for relative decrease in error
        double absErrDecreaseLimit=1.0e-7; // convergence criteria for absolute decrease in error
        int maxIterations=500; // maximum number of iterations
        gtsam::LevenbergMarquardtOptimizer optimizer(mygraph,myVals);
        double initialError=optimizer.error();
        double currentError=optimizer.error(), errorInit=currentError, previousError, absErrorDecrease=9.0e9, relErrorDecrease=9.0e9;
        uint nIterations=0;
        double initialAngle=angleBwVectors(axis,v1,v2);
        while(relErrorDecrease>relErrDecreaseLimit && absErrorDecrease>absErrDecreaseLimit && optimizer.iterations()<maxIterations){
            // set previous error values
            previousError=currentError;
            // perform iteration and checks
            optimizer.iterate(); // should have updated things in here now
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
        gtsam::Unit3 est_axis=estimate.at<gtsam::Unit3>(axisKey);
        gtsam::Point3 est_v1=estimate.at<gtsam::Point3>(v1Key);
        gtsam::Point3 est_v2=estimate.at<gtsam::Point3>(v2Key);
        double finalAngle=angleBwVectors(est_axis,est_v1,est_v2);
        std::cout<<"    test #"<<i<<": initial angle between: "<<initialAngle<<", final angle between: "<<finalAngle<<"  |  optimizer error "<<initialError<<" --> "<<finalError<<" ("<<nIterations<<" iterations, with "<<mygraph.size()<<" factors and "<<myVals.size()<<" values)"<<std::endl;
        //std::cout<<"        initial: q1=["<<axis.rotation().quaternion().transpose()<<"], p1=["<<axis.translation().transpose()<<"], v1=["<<v1.transpose()<<"] | q2=["<<x2.rotation().quaternion().transpose()<<"], p2=["<<x2.translation().transpose()<<"], v2=["<<v2.transpose()<<"]"<<std::endl;
        //std::cout<<"        optimized: q1=["<<est_axis.rotation().quaternion().transpose()<<"], p1=["<<est_axis.translation().transpose()<<"], v1=["<<est_v1.transpose()<<"] | q2=["<<est_x2.rotation().quaternion().transpose()<<"], p2=["<<est_x2.translation().transpose()<<"], v2=["<<est_v2.transpose()<<"]"<<std::endl;
        // can find the residual in MATLAB with: (quatrotate(quatinv(q1),v1)+p1)-(quatrotate(quatinv(q2),v2)+p2)
        //     why do I have to use quatinv() here?
        if((finalAngle-maxang)>errorTol){ //
            std::cerr<<"error: final angle is greater than maximum set angle ("<<finalAngle<<" > "<<maxang<<" + tol="<<errorTol<<")"<<std::endl<<std::endl;
            return 1;
        }
        // now clear the graph
        myVals.clear();
    }
    return 0;
}

double angleBwVectors(const gtsam::Unit3& axis, const gtsam::Point3& v1, const gtsam::Point3& v2){
    double numerator=(axis.unitVector().transpose()*(v1-v2));
    double denom=((v1-v2).norm());
    return acos( numerator / denom );
}

int random_factor_tests(uint nTests){
    // in a loop, randomly generate inputs to the max factor and test derivatives numerically.
    gtsam::Vector1 myNoiseVec(1.0e-3);
    gtsam::SharedNoiseModel myNoiseModel=gtsam::noiseModel::Isotropic::Sigmas(myNoiseVec);
    gtsam::Values myVals;
    gtsam::NonlinearFactorGraph mygraph;
    gtsam::Key axisKey=gtsam::Symbol('a',0);
    gtsam::Key v1Key=gtsam::Symbol('b',0);
    gtsam::Key v2Key=gtsam::Symbol('c',0);
    double ang=M_PI/2;
    // (note: when sigma=1, then optimizer error = 0.5*(factor error^2)
    for(uint i=0;i<nTests;i++) {
        // generate random vectors and then put in keys
        gtsam::Point3 v1=testutils::randomPoint3(), v2=testutils::randomPoint3();
        gtsam::Unit3 axis=testutils::randomUnit3();
        if (!myVals.exists(axisKey)) {
            myVals.insert(axisKey, axis);
            myVals.insert(v1Key, v1);
            myVals.insert(v2Key, v2);
        } else {
            myVals.update(axisKey, axis);
            myVals.update(v1Key, v1);
            myVals.update(v2Key, v2);
        }
        bioslam::MaxAngleBetweenAxisAndSegmentFactor testFac=bioslam::MaxAngleBetweenAxisAndSegmentFactor(axisKey, v1Key, v2Key, ang, myNoiseModel);
        // test derivative numerically
        test_derivative_numerically(testFac, axis, v1, v2);
    }
    return 0;
}

int test_derivative_numerically(const bioslam::MaxAngleBetweenAxisAndSegmentFactor& fac,const gtsam::Unit3& axis, const gtsam::Point3& v1, const gtsam::Point3& v2){
    // unit test the Jacobian against GTSAM's numerical derivative
    // this may seem silly for such a simple factor, but it's important to do.
    // () get derived error and jacobians
    gtsam::Matrix derivedH1, derivedH2, derivedH3;
    gtsam::Vector derivedErr=fac.evaluateError(axis,v1, v2, derivedH1, derivedH2, derivedH3);
    // () get numerical jacobians
    //    I think to call it it's numericalDerivativeXY where X=number of input variables and Y=which Jacobian you want to test
    //    templates are: <output type (typically gtsam::Vector), then the input argument types in order)
    gtsam::Matrix numericalH1=gtsam::numericalDerivative31<gtsam::Vector,gtsam::Unit3,gtsam::Point3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Unit3&, const gtsam::Point3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::MaxAngleBetweenAxisAndSegmentFactor::evaluateError,fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,boost::none,boost::none,boost::none)),axis,v1,v2,1e-5);
    gtsam::Matrix numericalH2=gtsam::numericalDerivative32<gtsam::Vector,gtsam::Unit3,gtsam::Point3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Unit3&, const gtsam::Point3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::MaxAngleBetweenAxisAndSegmentFactor::evaluateError,fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,boost::none,boost::none,boost::none)),axis,v1,v2,1e-5);
    gtsam::Matrix numericalH3=gtsam::numericalDerivative33<gtsam::Vector,gtsam::Unit3,gtsam::Point3,gtsam::Point3>(
            std::function<gtsam::Vector(const gtsam::Unit3&, const gtsam::Point3&, const gtsam::Point3&)>
                    (std::bind(&bioslam::MaxAngleBetweenAxisAndSegmentFactor::evaluateError,fac,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,boost::none,boost::none,boost::none)),axis,v1,v2,1e-5);

    // now test using gtsam::assert_equal()
    bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-5);
    bool testH2=gtsam::assert_equal(derivedH2,numericalH2,1e-5);
    bool testH3=gtsam::assert_equal(derivedH3,numericalH3,1e-5);
    if (!testH1){
        std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
        throw std::runtime_error("jacobian did not check out numerically.");
        return 1;
    }
    if (!testH2){
        std::cerr<<"H2 did not check out numerically."<<std::endl<<"derivedH2="<<derivedH2<<std::endl<<"numericalH2"<<numericalH2<<std::endl;
        throw std::runtime_error("jacobian did not check out numerically.");
        return 1;
    }
    if (!testH3){
        std::cerr<<"H3 did not check out numerically."<<std::endl<<"derivedH3="<<derivedH3<<std::endl<<"numericalH3"<<numericalH3<<std::endl;
        throw std::runtime_error("jacobian did not check out numerically.");
        return 1;
    }
    return 0;
}
