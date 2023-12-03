// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// test magnetometer factors in both bioslam and gtsam

#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/MagFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include "factors/MagPose3Factor.h"
#include <gtsam/base/numericalDerivative.h>
#include "testutils.h"
#include "mathutils.h"

std::vector<gtsam::Rot3> get_vector_orientation_from_Pose3_Values(gtsam::Values vals);
std::vector<gtsam::Rot3> get_vector_orientation_from_Rot3_Values(gtsam::Values vals);
std::vector<gtsam::Rot3> MagFactor1_only_estimation(gtsam::Point3 B_global, gtsam::SharedNoiseModel magNoiseModel, std::vector<gtsam::Point3> magMeas);
std::vector<gtsam::Rot3> MagPose3Factor_only_estimation(gtsam::Point3 B_global, gtsam::SharedNoiseModel magNoiseModel, std::vector<gtsam::Point3> magMeas);
uint random_factor_tests(uint nTests=1000);
int test_derivative_numerically(const bioslam::MagPose3Factor& fac, const gtsam::Pose3 &x);
void print_test_results(double err, uint iterations, const gtsam::Pose3& initPose, const gtsam::Pose3& finalPose, const gtsam::Vector3& measB, const gtsam::Vector3& bN);

int main(){
    // ----------
    random_factor_tests();
    // ---------
    // (0) settings
    int n=100;
    gtsam::Point3 B_Global(19.6,5.094,-47.741);
    Eigen::VectorXd magNoiseVec(3); magNoiseVec<<10.51,10.51,10.51;
    gtsam::SharedNoiseModel magNoiseModel=gtsam::noiseModel::Isotropic::Sigmas(magNoiseVec);
    double errorTol=1.0e-5; // error tolerance for tests to pass
    // (1) create a chain of Rot3 objects to represent the truth data
    std::vector<gtsam::Rot3> truth(n); // L->G
    truth[0]=gtsam::Rot3();
    for(int i=1;i<n;i++){
        truth[i]=truth[i-1]*testutils::randomRot3();
    }
    // (2) from global mag def and truth rotation data, create measurements
    std::vector<gtsam::Point3> meas(n);
    for(int i=0;i<n;i++){
        meas[i]=truth[i].unrotate(B_Global); // meas_L= r_L_to_G'*B_G
    }
    // (3) construct factorgraph of gtsam's MagFactor1 and optimize
    std::vector<gtsam::Rot3> MagFactor1_est=MagFactor1_only_estimation(B_Global, magNoiseModel, meas);
    std::vector<gtsam::Rot3> MagPose3Factor_est=MagPose3Factor_only_estimation(B_Global, magNoiseModel, meas);
    //todo:  (4) construct factorgraph with my factor and optimize
    // get numerical RMSE--both orientation and transformed measurement error
    Eigen::VectorXd err_MagFactor1_yaw(n); Eigen::VectorXd err_MagFactor1_pitch(n); Eigen::VectorXd err_MagFactor1_roll(n);
    Eigen::VectorXd err_MagPose3Factor_yaw(n); Eigen::VectorXd err_MagPose3Factor_pitch(n); Eigen::VectorXd err_MagPose3Factor_roll(n);
    std::vector<gtsam::Point3> B_mf1(n); Eigen::VectorXd B_mf1_err(n);
    std::vector<gtsam::Point3> B_mp3f(n); Eigen::VectorXd B_mp3f_err(n);
    for(int i=0;i<n;i++){
        gtsam::Rot3 err_mf1=(truth[i]*MagFactor1_est[i].inverse());
        gtsam::Rot3 err_mp3f=(truth[i]*MagPose3Factor_est[i].inverse());
        err_MagFactor1_yaw[i]=err_mf1.yaw(); err_MagFactor1_pitch[i]=err_mf1.pitch(); err_MagFactor1_roll[i]=err_mf1.roll();
        err_MagPose3Factor_yaw[i]=err_mp3f.yaw(); err_MagPose3Factor_pitch[i]=err_mp3f.pitch(); err_MagPose3Factor_roll[i]=err_mp3f.roll();
        // how well do you get back to B_Global when using the estimated orientations and measurements?
        B_mf1[i]=MagFactor1_est[i].rotate(meas[i]);
        B_mf1_err[i]=(B_Global-B_mf1[i]).norm();
        B_mp3f[i]=MagPose3Factor_est[i].rotate(meas[i]);
        B_mp3f_err[i]=(B_Global-B_mp3f[i]).norm();
    }
    std::cout<<"---- MagFactor1 error ------"<<std::endl;
    std::cout<<"    orientation error: "<<std::endl;
    double mf1_yaw_rmse=sqrt((err_MagFactor1_yaw.cwiseProduct(err_MagFactor1_yaw)).mean());
    double mf1_pitch_rmse=sqrt((err_MagFactor1_pitch.cwiseProduct(err_MagFactor1_pitch)).mean());
    double mf1_roll_rmse=sqrt((err_MagFactor1_roll.cwiseProduct(err_MagFactor1_roll)).mean());
    std::cout<<"        yaw: "<<mf1_yaw_rmse<<" rad RMSE"<<std::endl;
    std::cout<<"        pitch: "<<mf1_pitch_rmse<<" rad RMSE"<<std::endl;
    std::cout<<"        roll: "<<mf1_roll_rmse<<" rad RMSE"<<std::endl;
    std::cout<<"    transformed measurement error: B_Global - est_orientation.rotate(measurement)"<<std::endl;
    double mf1_B_err=sqrt((B_mf1_err.cwiseProduct(B_mf1_err)).mean());
    std::cout<<"        "<<mf1_B_err<<" uT RMSE"<<std::endl;
    std::cout<<"---- MagPose3Factor error ------"<<std::endl;
    std::cout<<"    orientation error: "<<std::endl;
    double mp3f_yaw_rmse=sqrt((err_MagPose3Factor_yaw.cwiseProduct(err_MagPose3Factor_yaw)).mean());
    double mp3f_pitch_rmse=sqrt((err_MagPose3Factor_pitch.cwiseProduct(err_MagPose3Factor_pitch)).mean());
    double mp3f_roll_rmse=sqrt((err_MagPose3Factor_roll.cwiseProduct(err_MagPose3Factor_roll)).mean());
    std::cout<<"        yaw: "<<mp3f_yaw_rmse<<" rad RMSE"<<std::endl;
    std::cout<<"        pitch: "<<mp3f_pitch_rmse<<" rad RMSE"<<std::endl;
    std::cout<<"        roll: "<<mp3f_roll_rmse<<" rad RMSE"<<std::endl;
    std::cout<<"    transformed measurement error: B_Global - est_orientation.rotate(measurement)"<<std::endl;
    double mp3f_B_err=sqrt((B_mp3f_err.cwiseProduct(B_mp3f_err)).mean());
    std::cout<<"        "<<mp3f_B_err<<" uT RMSE"<<std::endl;
    std::cout<<"-----------------------------"<<std::endl;
    if(mf1_B_err>errorTol){ throw std::runtime_error("error too high"); }
    if(mp3f_B_err>errorTol){ throw std::runtime_error("error too high"); }
    return 0;
}

uint random_factor_tests(uint nTests){
    gtsam::SharedNoiseModel noiseModel=gtsam::noiseModel::Isotropic::Sigmas(gtsam::Vector3(1.0,1.0,1.0));
    for(uint k=0; k<nTests; k++){
        // construct random system
        gtsam::Values vals;
        gtsam::Key poseKey=gtsam::Symbol('a',0);
        gtsam::Pose3 x=testutils::randomPose3(); // random pose
        vals.insert(poseKey,x);
        gtsam::Vector3 meas=testutils::randomVector3(); // random measurement
        gtsam::Vector3 bN=testutils::randomRot3()*meas; // random global mag
        meas.normalize(); bN.normalize(); // normalize for numerical conditioning
        // construct MagPose3Factor
        bioslam::MagPose3Factor fac=bioslam::MagPose3Factor(poseKey,meas,bN.norm(),gtsam::Unit3(bN.normalized()),gtsam::Point3(0.0,0.0,0.0),noiseModel);
        test_derivative_numerically(fac,x);
        // create graph and add factor
        gtsam::NonlinearFactorGraph graph;
        graph.add(fac);
        gtsam::LevenbergMarquardtOptimizer optimizer(graph,vals);
        gtsam::Values estimate=optimizer.optimize();
        // collect output
        gtsam::Pose3 estimatedPose=estimate.at<gtsam::Pose3>(poseKey);
        gtsam::Rot3 R_B_to_N=estimatedPose.rotation();
        gtsam::Vector3 measN=R_B_to_N*meas;
        // test criteria: LM error < 1.0e-5 & norm diff b/w measN and bN small.
        //    exception to test: if angle between measN and bN is ~180 deg, derivative becomes zero. ignore these cases.
        double angBw=mathutils::unsignedAngle(measN,bN);
        if(angBw<M_PI*.99){ // ignore the aforementioned rare case where bN and measN point in opposite directions
            if(optimizer.error()>1.0e-5){
                print_test_results(optimizer.error(),optimizer.iterations(),x,estimatedPose,meas,bN);
                throw std::runtime_error("test failed: optimized error too high.");
            }
            if(angBw>1.0e-5){
                print_test_results(optimizer.error(),optimizer.iterations(),x,estimatedPose,meas,bN);
                throw std::runtime_error("test failed: angle between measN and global mag def too large.");
            }
        }
    }
    return 0;
}

void print_test_results(double err, uint iterations, const gtsam::Pose3& initPose, const gtsam::Pose3& finalPose, const gtsam::Vector3& measB, const gtsam::Vector3& bN){
    gtsam::Vector3 measN=finalPose.rotation()*measB;
    std::cout<<"optimizer: converged error "<<err<<" in "<<iterations<<" iterations"<<std::endl;
    std::cout<<"meas[N]=["<<measN.transpose()<<"], global B=["<<bN.transpose()<<"] (diff = ["<<(measN-bN).transpose()<<"], norm="<<(measN-bN).norm()<<", angle b/w="<<mathutils::unsignedAngle(measN,bN)<<")"<<std::endl;
}

int test_derivative_numerically(const bioslam::MagPose3Factor& fac, const gtsam::Pose3 &x){
    // unit test the Jacobian against GTSAM's numerical derivative
    // () get derived error and jacobians
    gtsam::Matrix derivedH1;
    gtsam::Vector derivedErr=fac.evaluateError(x,derivedH1);
    // () get numerical jacobians
    //    I think to call it it's numericalDerivativeXY where X=number of input variables and Y=which Jacobian you want to test
    //    templates are: <output type (typically gtsam::Vector), then the input argument types in order)
    gtsam::Matrix numericalH1=gtsam::numericalDerivative11<gtsam::Vector,gtsam::Pose3>(
            std::function<gtsam::Vector(const gtsam::Pose3&)>
                    (std::bind(&bioslam::MagPose3Factor::evaluateError,fac,std::placeholders::_1,boost::none)),x,1e-5);
    // now test using gtsam::assert_equal()
    bool testH1=gtsam::assert_equal(derivedH1,numericalH1,1e-9);
    if (!testH1){
        std::cerr<<"H1 did not check out numerically."<<std::endl<<"derivedH1="<<derivedH1<<std::endl<<"numericalH1"<<numericalH1<<std::endl;
        throw std::runtime_error("test failed.");
        return 1;
    }
    return 0;
}

std::vector<gtsam::Rot3> MagPose3Factor_only_estimation(gtsam::Point3 B_global, gtsam::SharedNoiseModel magNoiseModel, std::vector<gtsam::Point3> magMeas){
    unsigned char sym='a';
    gtsam::Values est; // Values class for this factorgraph
    gtsam::NonlinearFactorGraph graph;
    double magScale=sqrt(pow(B_global.x(),2)+pow(B_global.y(),2)+pow(B_global.z(),2));
    gtsam::Unit3 mag_dir_G(B_global.x(),B_global.y(),B_global.z());
    gtsam::Point3 magBias(0.0,0.0,0.0);
    for(int i=0;i<magMeas.size();i++){
        graph += bioslam::MagPose3Factor(gtsam::Symbol(sym,uint64_t(i)),magMeas[i],magScale,mag_dir_G,magBias,magNoiseModel);
        est.insert(gtsam::Symbol(sym,uint64_t(i)), gtsam::Pose3());
    }
    gtsam::LevenbergMarquardtOptimizer opt(graph,est); // setup opt
    est=opt.optimize();
    //cout<<"finished optimization in "<<opt.iterations()<<" iterations with final error of "<<opt.error()<<endl;
    std::vector<gtsam::Rot3> rotVec=get_vector_orientation_from_Pose3_Values(est); // pull out results;
    return rotVec;
}

std::vector<gtsam::Rot3> MagFactor1_only_estimation(gtsam::Point3 B_global, gtsam::SharedNoiseModel magNoiseModel, std::vector<gtsam::Point3> magMeas){
    unsigned char rot3_sym='a';
    gtsam::Values est; // Values class for this factorgraph
    gtsam::NonlinearFactorGraph graph;
    double magScale=sqrt(pow(B_global.x(),2)+pow(B_global.y(),2)+pow(B_global.z(),2));
    gtsam::Unit3 mag_dir_G(B_global.x(),B_global.y(),B_global.z());
    gtsam::Point3 magBias(0.0,0.0,0.0);
    for(int i=0;i<magMeas.size();i++){
        graph += gtsam::MagFactor1(gtsam::Symbol(rot3_sym,uint64_t(i)),magMeas[i],magScale,mag_dir_G,magBias,magNoiseModel);
        est.insert(gtsam::Symbol(rot3_sym,uint64_t(i)), gtsam::Rot3());
    }
    gtsam::LevenbergMarquardtOptimizer opt(graph,est); // setup opt
    est=opt.optimize();
    //cout<<"finished optimization in "<<opt.iterations()<<" iterations with final error of "<<opt.error()<<endl;
    std::vector<gtsam::Rot3> rotVec=get_vector_orientation_from_Rot3_Values(est); // pull out results;
    return rotVec;
}

std::vector<gtsam::Rot3> get_vector_orientation_from_Rot3_Values(gtsam::Values vals){
    // todo: clean this up and use better methods to filter
    auto filtered_values_Pose3 = vals.filter<gtsam::Rot3>();
    std::vector<gtsam::Rot3> vector_Pose3(filtered_values_Pose3.size());
    std::vector<gtsam::Rot3> vector_Rot3(filtered_values_Pose3.size());
    int i =0;
    for (auto v: filtered_values_Pose3){
        vector_Rot3[i]= (gtsam::Rot3) v.value;
        i++;
    }
    return vector_Rot3;
}

std::vector<gtsam::Rot3> get_vector_orientation_from_Pose3_Values(gtsam::Values vals){
    // todo: clean this up and use better methods to filter
    auto filtered_values_Pose3 = vals.filter<gtsam::Pose3>();
    std::vector<gtsam::Pose3> vector_Pose3(filtered_values_Pose3.size());
    std::vector<gtsam::Rot3> vector_Rot3(filtered_values_Pose3.size());
    int i =0;
    for (auto v: filtered_values_Pose3){
        vector_Pose3[i]= (gtsam::Pose3) v.value;
        vector_Rot3[i]= vector_Pose3[i].rotation();
        i++;
    }
    return vector_Rot3;
}
