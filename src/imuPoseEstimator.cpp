// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include "imuPoseEstimator.h"
#include "factors/MagPose3Factor.h"
#include "mathutils.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include "VarStrToCharMap.h" // includes gtsam::Values
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include "imu/imuNoiseModelHandler.h"
#include <numeric>
#include <iostream>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsamutils.h>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <factors/Pose3Priors.h>

using Eigen::SelfAdjointEigenSolver; // one of the eigenvalue solvers

imuPoseEstimator::imuPoseEstimator(const imu& imuIn, const std::string& id){
    m_imu=imuIn;
    // now, from m_strId, set the variable names and add them to the VarStrToCharMap
    m_strId=id;
    m_poseVarStr=m_strId+"_ImuPose";
    m_velVarStr=m_strId+"_ImuVel";
    m_imuBiasVarStr=m_strId+"_ImuBias";
    VarStrToCharMap::insert(m_poseVarStr);
    VarStrToCharMap::insert(m_velVarStr);
    VarStrToCharMap::insert(m_imuBiasVarStr);
    m_poseVarChar=VarStrToCharMap::getChar(m_poseVarStr);
    m_velVarChar=VarStrToCharMap::getChar(m_velVarStr);
    m_imuBiasVarChar=VarStrToCharMap::getChar(m_imuBiasVarStr);
}

void imuPoseEstimator::setOptimizedValuesToInitialValues(){
    m_estimate=gtsam::Values(m_initialValues);
    setMemberStatesFromValues(m_estimate);
}

void imuPoseEstimator::setup(){
    // sets up the factorgraph for inference
    if(imuBiasModelMode<0 || imuBiasModelMode>1){ std::cerr<<"ERROR: imuBiasModelMode not recognized."<<std::endl; }
    switch(imuBiasModelMode){
        case 0: // imu factor with static bias
            std::cout << "setting up with static bias and ImuFactor" << std::endl;
            setupImuFactorStaticBias();
            break;
        case 1: // combinedimufactor
            std::cout << "setting up with dynamic bias and CombinedImuFactor" << std::endl;
            setupCombinedImuFactor();
            break;
    }
}

std::vector<uint> imuPoseEstimator::getImuIndecesCorrespondingToKeyframes() const {
    // return an array of the indeces in the imu j which go with keyframes k. zero indexed
    std::vector<uint> imuIdxArray(m_poseKeys.size());
    imuIdxArray[0]=0;
    for(uint k=1; k<imuIdxArray.size();k++){
        imuIdxArray[k]=k*m_numMeasToPreint-1;
    }
    return imuIdxArray;
}

void imuPoseEstimator::setNoiseModelsFromMemberNoiseVariables() {
    // a function which sets all of the member noise models from member noise variables (typically vectors or doubles)
    // note: this does not set noise models for CombinedImuFactors or ImuFactors. Those are set through preintegration in their respective setup functions.
    m_magnetometerNoiseModel=gtsam::noiseModel::Isotropic::Sigmas(m_magnetometerNoise,true);
}

std::tuple<std::vector<gtsam::Pose3>,gtsam::Matrix,std::vector<gtsam::imuBias::ConstantBias>> imuPoseEstimator::getInitializationData() const {
    // get initialization data
    // NOTE: there is an issue with Eigen and std containers, see: https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html
    // so the set of vectors is done as a matrix
    std::vector<gtsam::Pose3> initPose(getExpectedNumberOfKeyframesFromImuData());
    gtsam::Matrix initVel(getExpectedNumberOfKeyframesFromImuData(),3);
    std::vector<gtsam::imuBias::ConstantBias> initBias(getExpectedNumberOfKeyframesFromImuData());
    if(m_initializationScheme==0){
        // just set zeros for everything
        for(uint k=0; k<getExpectedNumberOfKeyframesFromImuData();k++){
            initPose[k]=gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0,0.0,0.0),gtsam::Point3(0.0,0.0,0.0));
            initVel.block<1,3>(k,0)=gtsam::Vector3::Zero();
            initBias[k]=gtsam::imuBias::ConstantBias();
        }
    }else if(m_initializationScheme==1){
        // run forward/backward EKF
        // --- settings ---
        uint numFBPasses=3;
        // ----------------
        double tic=clock();
        std::cout<<"running F/B EKF with "<<numFBPasses<<" F/B passes to initialize IMU orientation... ";
        Eigen::MatrixXd q_B_to_N=mathutils::simpleImuOrientationForwardBackwardEkfWrapper(gtsamutils::gyroMatrix(m_imu), gtsamutils::accelMatrix(m_imu), m_imu.getDeltaT(), numFBPasses);
        // set zeros for position, velocity, and imu bias
        std::vector<uint> imuIdxArray = getImuIndecesCorrespondingToKeyframes();
        for(uint k=0;k<getExpectedNumberOfKeyframesFromImuData();k++){
            gtsam::Rot3 R_N_to_B=gtsam::Rot3(q_B_to_N(imuIdxArray[k],0),q_B_to_N(imuIdxArray[k],1),q_B_to_N(imuIdxArray[k],2),q_B_to_N(imuIdxArray[k],3));
            //    important to remember! EKF outputs orientations [N->B], so invert them on the next line as you put them into the optimizer. our problems wants it as B->N
            gtsam::Rot3 R_B_to_N=R_N_to_B.inverse();
            initPose[k]=gtsam::Pose3(R_B_to_N,gtsam::Point3(0.0,0.0,0.0));
            initVel.block<1,3>(k,0)=gtsam::Vector3::Zero();
            initBias[k]=gtsam::imuBias::ConstantBias();
        }
        std::cout<<"complete! ("<<(clock()-tic)/CLOCKS_PER_SEC<<" sec)"<<std::endl;
    }else{
        throw std::runtime_error("unknown initilization scheme. choose 0 or 1.");
    }
    return std::make_tuple(initPose,initVel,initBias);
}

uint imuPoseEstimator::getExpectedNumberOfKeyframesFromImuData() const{
    uint nKeyframes=0;
    if(m_imu.length()%m_numMeasToPreint == 0){
        nKeyframes=floor(m_imu.length()/m_numMeasToPreint);
    } else{
        nKeyframes=floor(m_imu.length()/m_numMeasToPreint)+1;
    }
    return nKeyframes;
}

void imuPoseEstimator::setAnyRequestedPriors(bool verbose){
    // optional: add prior factors on this first keyframe
    if((m_usePositionPrior || m_useVelocityPrior || m_useImuBiasPrior) && verbose){std::cout << "added the following priors to the first keyframe:" << std::endl;}
    if(m_usePositionPrior){ // add the prior on position
        gtsam::SharedNoiseModel prior_pos_noise_model = gtsam::noiseModel::Diagonal::Sigmas( m_priorPositionNoise);
        m_graph += bioslam::Pose3TranslationPrior(m_poseKeys[0], m_priorPosition, prior_pos_noise_model);
        if(verbose){std::cout<<"    prior position: mean: ["<<m_priorPosition.transpose()<<"], sigmas=["<<m_priorPositionNoise.transpose()<<"]"<<std::endl;}
    }
    if(m_useVelocityPrior){ // add the prior on velocity
        gtsam::SharedNoiseModel prior_velocity_noise_model = gtsam::noiseModel::Diagonal::Sigmas(m_priorVelocityNoise); // m/s
        m_graph += gtsam::PriorFactor<gtsam::Vector3>(m_velKeys[0], m_priorVelocity, prior_velocity_noise_model);
        if(verbose){std::cout<<"    prior velocity: mean=["<<m_priorVelocity.transpose()<<"], sigmas=["<<m_priorVelocityNoise.transpose()<<"]"<<std::endl;}
    }
    if(m_useImuBiasPrior){ // add the prior on imubias
        gtsam::SharedNoiseModel priorImuBiasNoiseModel=gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << m_priorAccelBiasConstantNoise,m_priorGyroBiasConstantNoise).finished());
        m_graph += gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(m_imuBiasKeys[0], gtsam::imuBias::ConstantBias(m_priorAccelBias,m_priorGyroBias), priorImuBiasNoiseModel);
        if(verbose){std::cout<<"    prior imu bias: mean: ["<<m_priorGyroBias.transpose()<<", "<<m_priorAccelBias.transpose()<<"], sigmas=["<<m_priorAccelBiasConstantNoise.transpose()<<","<<m_priorGyroBiasConstantNoise.transpose()<<"]"<<std::endl;}
    }
    if(m_useCompassPrior){ // add compass angle prior (like setting a yaw prior)
        gtsam::SharedNoiseModel priorCompassNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(m_priorCompassAngleNoise),true);
        m_graph += bioslam::Pose3CompassPrior(m_poseKeys[0],m_refVecLocal,m_refVecNav,-1.0*m_globalAcc.normalized(),m_priorCompassAngle,priorCompassNoiseModel);
        if(verbose){std::cout<<"    prior compass angle: mean: "<<m_priorCompassAngle<<" rad, sigma="<<m_priorCompassAngleNoise<<" rad | compass system: refVec[B]=["<<m_refVecLocal.transpose()<<"], refVec[N]=["<<m_refVecNav.transpose()<<"], upVec[N]=["<<-1.0*m_globalAcc.normalized().transpose()<<"]"<<std::endl;}
    }
}

void imuPoseEstimator::setupCombinedImuFactor(){
    // sets up imu pose estimator according to CombinedImuFactor with standard (dynamic) imu bias
    // ------ settings -------
    double dt = m_imu.getDeltaT();
    int printSetupDebugEveryManyIterations=100000; // arbitrarily large number => no printing
    // -----------------------
    setNoiseModelsFromMemberNoiseVariables();
    m_nKeyframes=getExpectedNumberOfKeyframesFromImuData();
    // now set KeyVector to hold variables keys
    m_poseKeys.resize(m_nKeyframes), m_velKeys.resize(m_nKeyframes), m_imuBiasKeys.resize(m_nKeyframes);
    for(uint k=0; k<m_nKeyframes; k++) {
        m_poseKeys[k] = gtsam::Symbol(m_poseVarChar, k);
        m_velKeys[k] = gtsam::Symbol(m_velVarChar, k);
        m_imuBiasKeys[k] = gtsam::Symbol(m_imuBiasVarChar, k);
    }
    double mag_scale_G=m_globalB.norm();
    gtsam::Unit3 mag_dir_NWU=gtsam::Unit3(m_globalB.normalized());
    //Unit3 mag_dir_NWU(1.0,0.0,0.0);
    gtsam::Point3 mag_bias(0.0,0.0,0.0); // todo: implement mag bias model? right now just assuming zero.
    // () get initialization data from m_initializationScheme
    std::vector<gtsam::Pose3> initPose; gtsam::Matrix initVel; std::vector<gtsam::imuBias::ConstantBias> initImuBias;
    std::tie(initPose,initVel,initImuBias) = imuPoseEstimator::getInitializationData();
    // setup first keyframe's Values
    m_initialValues.insert(m_poseKeys[0], initPose[0]); // <- insert them into initialValues
    m_initialValues.insert(m_velKeys[0], (gtsam::Vector3)initVel.row(0)); // remember that initVel is a gtsam::Matrix, and so you have to static cast it to a gtsam::Vector after calling Eigen's row()
    m_initialValues.insert(m_imuBiasKeys[0], initImuBias[0]);
    // resize time vector of the estimated time domain, m_time, and set its first keyframe value
    m_time.resize(m_nKeyframes); m_time[0]=m_imu.relTimeSec[0];
    std::cout<<"setup initial states (t="<<m_time[0]<<" sec):"<<std::endl;
    std::cout<<"    orientation: q=["<<initPose[0].rotation().quaternion()[0]<<", "<<initPose[0].rotation().quaternion()[1]<<", "<<initPose[0].rotation().quaternion()[2]<<", "<<initPose[0].rotation().quaternion()[3]<<"]"<<std::endl;
    std::cout<<"    position: p=["<<initPose[0].translation().transpose()<<"]"<<std::endl;
    std::cout<<"    velocity: v=["<<initVel.row(0)<<"]"<<std::endl;
    std::cout<<"    imu bias: accel=["<<m_priorAccelBias.transpose()<<"], gyro = ["<<m_priorGyroBias.transpose()<<"]"<<std::endl;
    // setup any priors
    setAnyRequestedPriors(true);
    // setup preintegration params object
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p =boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>(new gtsam::PreintegratedCombinedMeasurements::Params(m_globalAcc));
    // setup imuNoiseHandler and apply that to the preintegration params
    imuNoiseModelHandler sensorBiasModel; sensorBiasModel.Opalv2();
    sensorBiasModel.applyModelToPreintMeasParams(p);
    // resize member variables for this loop. set timer doubles.
    double timeBeforeLoop=clock(), avgSetupLoopDuration=0.0;
    m_preintImus.resize(m_nKeyframes-1); // vector to hold the gtsam::PreintegratedCombinedMeasurements objects. This can be useful for debugging later.
    m_beginPreintIdxInImuData.resize(m_nKeyframes-1); m_endPreintIdxInImuData.resize(m_nKeyframes-1); // vector to hold the idxs in the IMU measurements that start and stop the preintegration period
    m_gyroCombinations.resize(m_nKeyframes-1); // vector to hold average gyro rate across preintegration periods
    uint nPreintInLoop=0; // counts the number of preintegrations made
    uint j=0; // j indexes the IMU measurements: j=[0, ..., m_imu.length()-1]
    // --------------------- main setup loop --------------------- //
    for(uint k=0; k<(m_nKeyframes-1); k++){ // k indexes the keyframes. remember you go to K-1 here because you're setting up factors and adding initial values at k+1 in the loop below.
        gtsam::PreintegratedCombinedMeasurements imu_preintegrated_(p, initImuBias[k]); // construct new preintegrated imu from params p and initial imu bias
        // ++++++++++ inner loop: imu preintegration ++++++++++ //
        m_beginPreintIdxInImuData[k]=j; // mark first idx in imu data for this preintegration cycle
        while(nPreintInLoop<m_numMeasToPreint){ // integrate imu
            gtsam::Vector3 acc(m_imu.ax[j], m_imu.ay[j], m_imu.az[j]);
            gtsam::Vector3 gyros(m_imu.gx[j], m_imu.gy[j], m_imu.gz[j]);
            imu_preintegrated_.integrateMeasurement(acc, gyros, dt); // add measurements
            nPreintInLoop++; j++;
        }
        m_endPreintIdxInImuData[k]=j; // mark last idx in imu data for this preintegration cycle
        m_preintImus[k]=imu_preintegrated_; // store this preintegrated imu for posterity
        m_gyroCombinations[k]=(imu_preintegrated_.deltaRij().rpy())/(dt*m_numMeasToPreint); // avg gyro rate across preintegration. note: gyro comb ~= preint meas deltaRij.rpy() (approx equal, maybe preint object accounts for gyro bias?)
        nPreintInLoop=0; // reset number of measurements in the preintegration for next loop
        // ++++++++++++++++++++++++++++++++++++++++++++++++++++ //
        m_graph+=gtsam::CombinedImuFactor(m_poseKeys[k], m_velKeys[k],m_poseKeys[k+1], m_velKeys[k+1], m_imuBiasKeys[k],m_imuBiasKeys[k+1], imu_preintegrated_);
        if(m_useMagnetometer) {
            gtsam::Point3 magMeas=gtsam::Point3(gtsamutils::mags_Vector3(m_imu,j)); m_downsampledMagMeas.push_back(magMeas);
            //std::cout<<"mag meas = "<<magMeas.transpose()<<std::endl;
            m_graph += bioslam::MagPose3Factor(m_poseKeys[k+1], magMeas, mag_scale_G, mag_dir_NWU, mag_bias,m_magnetometerNoiseModel);
        }
        // add initial values at k+1 (remember you set everything at k=0 before this loop)
        m_initialValues.insert(m_poseKeys[k+1],initPose[k]);
        m_initialValues.insert(m_velKeys[k+1], (gtsam::Vector3)initVel.row(k));
        m_initialValues.insert(m_imuBiasKeys[k+1], initImuBias[k]);
        m_time[k+1]=m_imu.relTimeSec[j]; // add estimated time
        if((k+1) % printSetupDebugEveryManyIterations == 0){ // print!
            std::cout<<"\radded "<<k+1<<"/"<<m_nKeyframes<<" states to graph (remaining time = "<<avgSetupLoopDuration*(m_nKeyframes-k)<<" sec)";
        }
        avgSetupLoopDuration=((clock()-timeBeforeLoop)/CLOCKS_PER_SEC)/(k); // recompute running average of time per setup loop
    }
    // ----------------------------------------------------------- //
    std::cout<<"\radded all "<<m_nKeyframes<<" keyframes to graph ("<<(clock()-timeBeforeLoop)/CLOCKS_PER_SEC<<" sec) with the following types of variables: Pose (x"<<m_poseKeys.size()<<"), Vel (x"<<m_velKeys.size()<<"), imuBias (x"<<m_imuBiasKeys.size()<<")"<<std::endl;
}

void imuPoseEstimator::setupImuFactorStaticBias() {
    // sets up imu pose estimator according to CombinedImuFactor with standard (dynamic) imu bias
    // ------ settings -------
    double dt = m_imu.getDeltaT();
    uint printSetupDebugEveryManyIterations=100000; // arbitrarily large number => no printing
    // -----------------------
    setNoiseModelsFromMemberNoiseVariables();
    m_nKeyframes=getExpectedNumberOfKeyframesFromImuData();
    // now set KeyVector to hold variables keys
    m_poseKeys.resize(m_nKeyframes), m_velKeys.resize(m_nKeyframes), m_imuBiasKeys.resize(1);
    m_imuBiasKeys[0] = gtsam::Symbol(m_imuBiasVarChar, 0); // only one key for imu bias
    for(uint k=0; k<m_nKeyframes; k++) {
        m_poseKeys[k] = gtsam::Symbol(m_poseVarChar, k);
        m_velKeys[k] = gtsam::Symbol(m_velVarChar, k);
    }
    double mag_scale_G=m_globalB.norm();
    gtsam::Unit3 mag_dir_NWU=gtsam::Unit3(m_globalB.normalized());
    //Unit3 mag_dir_NWU(1.0,0.0,0.0);
    gtsam::Point3 mag_bias(0.0,0.0,0.0); // todo: implement mag bias model? right now just assuming zero.
    // () get initialization data from m_initializationScheme
    std::vector<gtsam::Pose3> initPose; gtsam::Matrix initVel; std::vector<gtsam::imuBias::ConstantBias> initImuBias;
    std::tie(initPose,initVel,initImuBias) = imuPoseEstimator::getInitializationData();
    // setup first keyframe's Values
    m_initialValues.insert(m_poseKeys[0], initPose[0]); // <- insert them into initialValues
    m_initialValues.insert(m_velKeys[0], (gtsam::Vector3)initVel.row(0)); // remember that initVel is a gtsam::Matrix, and so you have to static cast it to a gtsam::Vector after calling Eigen's row()
    m_initialValues.insert(m_imuBiasKeys[0], initImuBias[0]);
    // resize time vector of the estimated time domain, m_time, and set its first keyframe value
    m_time.resize(m_nKeyframes); m_time[0]=m_imu.relTimeSec[0];
    std::cout<<"setup initial states (t="<<m_time[0]<<" sec):"<<std::endl;
    std::cout<<"    orientation: q=["<<initPose[0].rotation().quaternion()[0]<<", "<<initPose[0].rotation().quaternion()[1]<<", "<<initPose[0].rotation().quaternion()[2]<<", "<<initPose[0].rotation().quaternion()[3]<<"]"<<std::endl;
    std::cout<<"    position: p=["<<initPose[0].translation().transpose()<<"]"<<std::endl;
    std::cout<<"    velocity: v=["<<initVel.row(0)<<"]"<<std::endl;
    std::cout<<"    imu bias: accel=["<<m_priorAccelBias.transpose()<<"], gyro = ["<<m_priorGyroBias.transpose()<<"]"<<std::endl;
    // setup any priors
    setAnyRequestedPriors(true);
    // setup preintegration params object
    boost::shared_ptr<gtsam::PreintegrationParams> p =boost::shared_ptr<gtsam::PreintegrationParams>(new gtsam::PreintegrationParams(m_globalAcc));
    // setup imuNoiseHandler and apply that to the preintegration params
    imuNoiseModelHandler sensorBiasModel; sensorBiasModel.Opalv2();
    sensorBiasModel.applyModelToPreintMeasParams(p);
    // resize member variables for this loop. set timer doubles.
    double timeBeforeLoop=clock(), avgSetupLoopDuration=0.0;
    m_preintImus.resize(m_nKeyframes-1); // vector to hold the gtsam::PreintegratedCombinedMeasurements objects. This can be useful for debugging later.
    m_beginPreintIdxInImuData.resize(m_nKeyframes-1); m_endPreintIdxInImuData.resize(m_nKeyframes-1); // vector to hold the idxs in the IMU measurements that start and stop the preintegration period
    m_gyroCombinations.resize(m_nKeyframes-1); // vector to hold average gyro rate across preintegration periods
    uint nPreintInLoop=0; // counts the number of preintegrations made
    uint j=0; // j indexes the IMU measurements: j=[0, ..., m_imu.length()-1]
    // --------------------- main setup loop --------------------- //
    for(uint k=0; k<(m_nKeyframes-1); k++){ // k indexes the keyframes. remember you go to K-1 here because you're setting up factors and adding initial values at k+1 in the loop below.
        gtsam::PreintegratedImuMeasurements imu_preintegrated_(p, initImuBias[0]); // construct new preintegrated imu from params p and initial imu bias (since this is static, stay at initImuBias[0])
        // ++++++++++ inner loop: imu preintegration ++++++++++ //
        m_beginPreintIdxInImuData[k]=j; // mark first idx in imu data for this preintegration cycle
        while(nPreintInLoop<m_numMeasToPreint){ // integrate imu
            gtsam::Vector3 acc(m_imu.ax[j], m_imu.ay[j], m_imu.az[j]);
            gtsam::Vector3 gyros(m_imu.gx[j], m_imu.gy[j], m_imu.gz[j]);
            imu_preintegrated_.integrateMeasurement(acc, gyros, dt); // add measurements
            nPreintInLoop++; j++;
        }
        m_endPreintIdxInImuData[k]=j; // mark last idx in imu data for this preintegration cycle
        //m_preintImus[k]=imu_preintegrated_; // store this preintegrated imu for posterity //todo: this is for CombinedPreintegratedMeasurements only right now.
        m_gyroCombinations[k]=(imu_preintegrated_.deltaRij().rpy())/(dt*m_numMeasToPreint); // avg gyro rate across preintegration. note: gyro comb ~= preint meas deltaRij.rpy() (approx equal, maybe preint object accounts for gyro bias?)
        nPreintInLoop=0; // reset number of measurements in the preintegration for next loop
        // ++++++++++++++++++++++++++++++++++++++++++++++++++++ //
        m_graph+=gtsam::ImuFactor(m_poseKeys[k], m_velKeys[k],m_poseKeys[k+1], m_velKeys[k+1], m_imuBiasKeys[0], imu_preintegrated_);
        if(m_useMagnetometer) {
            gtsam::Point3 magMeas=gtsam::Point3(gtsamutils::mags_Vector3(m_imu,j)); m_downsampledMagMeas.push_back(magMeas);
            //std::cout<<"mag meas = "<<magMeas.transpose()<<std::endl;
            m_graph += bioslam::MagPose3Factor(m_poseKeys[k+1], magMeas, mag_scale_G, mag_dir_NWU, mag_bias,m_magnetometerNoiseModel);
        }
        // add initial values at k+1 (remember you set everything at k=0 before this loop)
        m_initialValues.insert(m_poseKeys[k+1],initPose[k]);
        m_initialValues.insert(m_velKeys[k+1], (gtsam::Vector3)initVel.row(k));
        m_time[k+1]=m_imu.relTimeSec[j]; // add estimated time
        if((k+1) % printSetupDebugEveryManyIterations == 0){ // print!
            std::cout<<"\radded "<<k+1<<"/"<<m_nKeyframes<<" states to graph (remaining time = "<<avgSetupLoopDuration*(m_nKeyframes-k)<<" sec)";
        }
        avgSetupLoopDuration=((clock()-timeBeforeLoop)/CLOCKS_PER_SEC)/(k); // recompute running average of time per setup loop
    }
    // ----------------------------------------------------------- //
    std::cout<<"\radded all "<<m_nKeyframes<<" keyframes to graph ("<<(clock()-timeBeforeLoop)/CLOCKS_PER_SEC<<" sec) with the following types of variables: Pose (x"<<m_poseKeys.size()<<"), Vel (x"<<m_velKeys.size()<<"), imuBias (x"<<m_imuBiasKeys.size()<<")"<<std::endl;
}

void imuPoseEstimator::fastOptimize(){
    // use the iterate() method and print error in between. don't retrieve values (expensive)
    // ---- settings ---- //
    bool doCustomIterationPrinting=true;
    // ------------------- //
    std::cout<<std::endl<<"*** imuPoseEstimator::fastOptimize() results for "<<m_strId<<" ***"<<std::endl;
    std::cout<<"    convergence criteria: abs. error decrease < "<<m_absErrDecreaseLimit<<" OR rel. error decrease < "<<m_relErrDecreaseLimit*100<<"% OR max iterations = "<<m_maxIterations<<std::endl;
    gtsam::LevenbergMarquardtParams params; params.setVerbosityLM("SUMMARY"); params.setVerbosity("TERMINATION"); params.setlambdaLowerBound(1.0e-8); params.setlambdaUpperBound(1.0e10);
    params.setAbsoluteErrorTol(m_absErrDecreaseLimit); params.setRelativeErrorTol(m_relErrDecreaseLimit); params.setMaxIterations(m_maxIterations);
    gtsam::LevenbergMarquardtOptimizer optimizer(m_graph,m_initialValues,params);
    //gtsam::GaussNewtonOptimizer optimizer(m_graph,m_initialValues);
    double currentError=optimizer.error(), errorInit=currentError, previousError, absErrorDecrease=9.0e9, relErrorDecrease=9.0e9;
    uint nIterations=0;
    bool iterationsSuccessful=true;
    // print initial state
    std::cout<<" --- initial state ----"<<std::endl;
    std::cout<<"    error: "<<currentError<<std::endl;
    gtsamutils::printErrorsInGraphByFactorType(m_graph,m_initialValues);
    double timeBeforeIterations=clock();
    m_optimizationTotalError.push_back(currentError); m_optimizationTotalTime.push_back(0.0);
    try {
        while(relErrorDecrease>m_relErrDecreaseLimit && absErrorDecrease>m_absErrDecreaseLimit && currentError>m_absErrLimit && optimizer.iterations()<m_maxIterations){
            double iterationStart=clock();
            // set previous error values
            previousError=currentError;
            // perform iteration and checks
            optimizer.iterate(); // should have updated things in here now
            if(optimizer.iterations()!=nIterations+1){ // why didn't this tick up?
                std::cout<<"WARNING: iteration number did not increase! exiting..."<<std::endl;
                iterationsSuccessful=false;
                break;
            }else{nIterations=optimizer.iterations();}
            // extract current state
            currentError=optimizer.error();
            // compute change in errors
            absErrorDecrease=previousError-currentError; relErrorDecrease=absErrorDecrease/previousError;
            // print iteration results
            if(doCustomIterationPrinting){
                std::cout<<"--- end iteration #"<<optimizer.iterations()-1<<" ("<<(clock()-iterationStart)/CLOCKS_PER_SEC<<" sec) ---"<<std::endl;
                std::cout<<"        total error: "<<currentError<<"   (decrease: "<<absErrorDecrease<<" || "<<relErrorDecrease*100<<"%)"<<std::endl;
            }
            m_optimizationTotalError.push_back(currentError); m_optimizationTotalTime.push_back((clock()-timeBeforeIterations)/CLOCKS_PER_SEC);
        }
    } catch (const gtsam::IndeterminantLinearSystemException& e){
        // catch an indeterminate linear system exception
        std::cout<<"--- Caught an indeterminant linear system exception! ---"<<std::endl;
        std::cout<<"    message was: "<<e.what()<<std::endl;
        gtsam::GaussianFactorGraph lingraph=*m_graph.linearize(optimizer.values()); // linearized graph
        // now, try the debugging methods to inspect this linear system exception
        gtsam::Matrix sparseJac=lingraph.sparseJacobian_();
        boost::filesystem::path sparseJacobianOnErrorFilename=(boost::filesystem::current_path()).append("sparseJacobianOnError.csv");
        const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
        gtsamutils::writeEigenMatrixToCsvFile(sparseJacobianOnErrorFilename.string(),sparseJac.transpose(),CSVFormat);
        std::cout<<"saved sparse jacobian to file: "<<sparseJacobianOnErrorFilename<<", it can be analyzed using MATLAB's sparse()."<<std::endl;
        std::cout<<"--------------------------------------------------------"<<std::endl;
    }
    // print convergence condition
    std::cout<<"--- Convergence condition: ---"<<std::endl;
    if(relErrorDecrease<m_relErrDecreaseLimit){ std::cout<<"    CONVERGED: rel. error decrease < limit ("<<relErrorDecrease<<" < "<<m_relErrDecreaseLimit<<")"<<std::endl;}
    else if(absErrorDecrease<m_absErrDecreaseLimit){ std::cout<<"    CONVERGED: abs. error decrease < limit ("<<absErrorDecrease<<" < "<<m_absErrDecreaseLimit<<")"<<std::endl;}
    else if(optimizer.iterations()==m_maxIterations) { std::cout << "    exiting. iteration maximum reached (" << m_maxIterations << ")" << std::endl;}
    else if(currentError<m_absErrLimit){std::cout<<"    CONVERGED. err ("<<currentError<<") is less than total limit ("<<m_absErrLimit<<")"<<std::endl;}
    else if(!iterationsSuccessful){ std::cout<<"    optimizer did not iterate when requested."<<std::endl;}
    else{ std::cout<<"    no convergence criteria met."<<std::endl; }
    std::cout<<"----------------------"<<std::endl;
    std::cout<<"Final state ("<<optimizer.iterations()<<" iterations, total time "<<(clock()-timeBeforeIterations)/CLOCKS_PER_SEC<<" sec):"<<std::endl;
    std::cout<<"    error: "<<currentError<<"    (init. "<<errorInit<<")"<<std::endl;
    std::cout<<"********** end fast optimization **********"<<std::endl<<std::endl;
    // pull out estimated Values and print graph errors by factor type
    m_estimate=optimizer.values();
    gtsamutils::printErrorsInGraphByFactorType(m_graph,m_estimate); // final errors in graph by type
    setMemberStatesFromValues(m_estimate); // set member states from values
    //testHeadingCanBeSpunWithNoChangeToError(m_graph,m_estimate,false);
}

void imuPoseEstimator::setMemberStatesFromValues(const gtsam::Values& vals){
    // take in a Values and set the arrays of member states from it
    m_pose=vectorizePoses(vals,m_poseKeys);
    m_orientation=vectorizeOrientations(m_pose);
    m_position=vectorizePositions(m_pose);
    m_velocity=vectorizeVelocities(vals,m_velKeys);
    // now set imu biases
    m_accelBias.resize(m_imuBiasKeys.size()); m_gyroBias.resize(m_imuBiasKeys.size());
    for(uint k=0;k<m_imuBiasKeys.size();k++){
        m_gyroBias[k]=vals.at<gtsam::imuBias::ConstantBias>(m_imuBiasKeys[k]).gyroscope();
        m_accelBias[k]=vals.at<gtsam::imuBias::ConstantBias>(m_imuBiasKeys[k]).accelerometer();
    }
    if(imuBiasModelMode==0){ // static bias, so go ahead and print estimated bias
        std::cout<<"estimated static IMU bias: gyro=["<<m_gyroBias[0].transpose()<<"] rad/s, accel=["<<m_accelBias[0].transpose()<<"] m/s^2"<<std::endl;
    }
}

void imuPoseEstimator::robustOptimize(bool printIntermediateResultsToFile){
    // use the iterate() method and print status in between
    // ---- settings ---- //
    //bool alwaysReturnLowestErrorValues=true; // if true, disallow optimizer from increasing error
    // ------------------- //
    std::cout<<std::endl<<"*** imuPoseEstimator::robustOptimize() results for "<<m_strId<<" ***"<<std::endl;
    std::cout<<"    convergence criteria: abs. error decrease < "<<m_absErrDecreaseLimit<<" OR rel. error decrease < "<<m_relErrDecreaseLimit*100<<"% OR max iterations = "<<m_maxIterations<<std::endl;
    //gtsam::LevenbergMarquardtParams params; params.setVerbosity("TRYLAMBDA");
    gtsam::LevenbergMarquardtOptimizer optimizer(m_graph,m_initialValues);
    //gtsam::GaussNewtonOptimizer optimizer(m_graph,m_estimate);
    //gtsam::Values lowestErrorValues=optimizer.values();
    double currentError=optimizer.error(), previousError, absErrorDecrease=9.0e9, relErrorDecrease=9.0e9;
    uint nIterations=0;
    std::string imuFactorTypeString;
    int nImuFactors;
    if(imuBiasModelMode==0){
        imuFactorTypeString="ImuFactor";
        nImuFactors=gtsamutils::getNumberOfFactorByType<gtsam::ImuFactor>(m_graph);
    }else{
        imuFactorTypeString="CombinedImuFactor";
        nImuFactors=gtsamutils::getNumberOfFactorByType<gtsam::CombinedImuFactor>(m_graph);
    }
    int nMagFactors=gtsamutils::getNumberOfFactorByType<bioslam::MagPose3Factor>(m_graph);
    bool hasMagPose3Factor=gtsamutils::getNumberOfFactorByType<bioslam::MagPose3Factor>(m_graph)>0;
    // extract initial state
    double magFactorSumNormdErr=getMagFactorErrorAtState(optimizer.values());
    double prevMagFactorErr, absMagFactorErrDecrease, relMagFactorErrDecrease;
    double imuFactorSumNormdErr=getGenericImuFactorErrorAtState(optimizer.values());
    double prevImuFactorErr, absImuFactorErrDecrease, relImuFactorErrDecrease;
    // save initial state in different name to reprint at end
    double magFactorSumNormdErrInit=magFactorSumNormdErr, errorInit=currentError, imuFactorSumNormdErrInit=imuFactorSumNormdErr;
    // print initial state
    std::cout<<" --- initial state ----"<<std::endl;
    std::cout<<"    error: "<<currentError<<std::endl;
    std::cout<<"    sum norms of "<<imuFactorTypeString<<" error="<<imuFactorSumNormdErr<<std::endl;
    if(hasMagPose3Factor){std::cout<<"    sum norms of MagPose3Factor error="<<magFactorSumNormdErr<<std::endl;}
    imuPoseEstimator::robustOptimizePrintImuBias(optimizer.values(), 4);
    double timeBeforeIterations=clock();
    try {
        while(relErrorDecrease>m_relErrDecreaseLimit && absErrorDecrease>m_absErrDecreaseLimit && optimizer.iterations()<m_maxIterations){
            double iterationStart=clock();
            // set previous error values
            previousError=currentError;
            prevMagFactorErr=magFactorSumNormdErr;
            prevImuFactorErr=imuFactorSumNormdErr;
            // perform iteration and checks
            optimizer.iterate(); // should have updated things in here now
            if(optimizer.iterations()!=nIterations+1){ // why didn't this tick up?
                std::cout<<"WARNING: iteration number did not increase! exiting..."<<std::endl;
                break;
            }else{nIterations=(int)optimizer.iterations();}
            // extract current state
            currentError=optimizer.error();
            magFactorSumNormdErr=getMagFactorErrorAtState(optimizer.values());
            imuFactorSumNormdErr=getGenericImuFactorErrorAtState(optimizer.values());
            // compute change in errors
            absErrorDecrease=previousError-currentError; relErrorDecrease=absErrorDecrease/previousError;
            absMagFactorErrDecrease=prevMagFactorErr-magFactorSumNormdErr; relMagFactorErrDecrease=absMagFactorErrDecrease/prevMagFactorErr;
            absImuFactorErrDecrease=prevImuFactorErr-imuFactorSumNormdErr; relImuFactorErrDecrease=absImuFactorErrDecrease/prevImuFactorErr;
            // print iteration results
            std::cout<<"--- Iteration #"<<optimizer.iterations()<<" ("<<(clock()-iterationStart)/CLOCKS_PER_SEC<<" sec) ---"<<std::endl;
            std::cout<<"        total error: "<<currentError<<"   (decrease: "<<absErrorDecrease<<" || "<<relErrorDecrease*100<<"%)"<<std::endl;
            std::cout<<"        "<<imuFactorTypeString<<" error: sum norms="<<imuFactorSumNormdErr<<" (decrease: "<<absImuFactorErrDecrease<<" || "<<relImuFactorErrDecrease*100<<"%) | avg norms="<<imuFactorSumNormdErr/nImuFactors<<std::endl;
            if(hasMagPose3Factor){std::cout<<"        MagPose3Factor error: sum norms="<<magFactorSumNormdErr<<" (decrease: "<<absMagFactorErrDecrease<<" || "<<relMagFactorErrDecrease*100<<"%) | avg norms="<<magFactorSumNormdErr/nMagFactors<<std::endl;}
            imuPoseEstimator::robustOptimizePrintImuBias(optimizer.values(), 8);
        }
    } catch (const gtsam::IndeterminantLinearSystemException& e){
        // catch an indeterminate linear system exception
        std::cout<<"Caught an indeterminate linear system exception!"<<std::endl;
        gtsam::GaussianFactorGraph lingraph=*m_graph.linearize(optimizer.values()); // linearized graph
        // now, try the debugging methods to inspect this linear system exception
        gtsam::Matrix m=lingraph.sparseJacobian_();
        std::string sparseJacobianOnErrorFilename="sparseJacobianOnError.csv";
        gtsamutils::saveMatrixToFile(m,",",sparseJacobianOnErrorFilename);
        std::cout<<"saved jacobian to file: "<<sparseJacobianOnErrorFilename<<std::endl;
    }
    std::cout<<"----------------------"<<std::endl;
    // print convergence condition
    std::cout<<"Convergence condition:"<<std::endl;
    if(relErrorDecrease<m_relErrDecreaseLimit){
        std::cout<<"    CONVERGED: rel. error decrease < limit ("<<relErrorDecrease<<" < "<<m_relErrDecreaseLimit<<")"<<std::endl;
    }else if(absErrorDecrease<m_absErrDecreaseLimit){
        std::cout<<"    CONVERGED: abs. error decrease < limit ("<<absErrorDecrease<<" < "<<m_absErrDecreaseLimit<<")"<<std::endl;
    }else if(optimizer.iterations()==m_maxIterations){
        std::cout<<"    exiting. iteration maximum reached ("<<m_maxIterations<<")"<<std::endl;
    }else{
        std::cout<<"    no convergence criteria met."<<std::endl;
    }
    std::cout<<"----------------------"<<std::endl;
    std::cout<<"Final state ("<<optimizer.iterations()<<" iterations, total time "<<(clock()-timeBeforeIterations)/CLOCKS_PER_SEC<<" sec):"<<std::endl;
    std::cout<<"    error: "<<currentError<<"    (init. "<<errorInit<<")"<<std::endl;
    std::cout<<"    sum norms of ImuFactor error="<<imuFactorSumNormdErr<<" (init. "<<imuFactorSumNormdErrInit<<") with avg norm error="<<imuFactorSumNormdErr/nImuFactors<<" (init. "<<imuFactorSumNormdErrInit/nImuFactors<<")"<<std::endl;
    if(hasMagPose3Factor){std::cout<<"    sum norms of MagPose3Factor error="<<magFactorSumNormdErr<<" (init. "<<magFactorSumNormdErrInit<<") with avg norm error="<<magFactorSumNormdErr/nMagFactors<<" (init. "<<magFactorSumNormdErrInit/nMagFactors<<")"<<std::endl;}
    imuPoseEstimator::robustOptimizePrintImuBias(optimizer.values(), 4);
    std::cout<<"********** end robust optimization **********"<<std::endl<<std::endl;

    m_estimate=optimizer.values();
    setMemberStatesFromValues(m_estimate);
}

void imuPoseEstimator::robustOptimizePrintInitialState(const int &nIndentSpaces, double& initImuFactorErr) {
    initImuFactorErr=234.345;
    std::cout<<"printing initial state"<<std::endl;
}

void imuPoseEstimator::robustOptimizePrintImuBias(const gtsam::Values& vals, int nSpacesToIndent){
    // because imu bias can be static or dynamic, we create this function to print a one-liner summary of the imu bias
    int nImuBiasKeys=gtsamutils::getNumberOfKeysByType<gtsam::imuBias::ConstantBias>(vals);
    if(imuBiasModelMode==0 && nImuBiasKeys==1){ // you set static bias and indeed have only one bias
        gtsam::Key imuBiasKey=gtsam::Symbol(m_imuBiasVarChar,0); // it should be just the zeroth key for imubias
        gtsam::imuBias::ConstantBias imuBias=vals.at<gtsam::imuBias::ConstantBias>(imuBiasKey);
        std::cout<<std::string((size_t)nSpacesToIndent,' ')<<"static bias: gyro=["<<imuBias.gyroscope().transpose()<<"] | accel=["<<imuBias.accelerometer().transpose()<<"]"<<std::endl;
    }else if(imuBiasModelMode==1 && nImuBiasKeys>1){ // you set for dynamic bias and indeed have multiple biases
        std::vector<double> gyroBiasX(nImuBiasKeys), gyroBiasY(nImuBiasKeys), gyroBiasZ(nImuBiasKeys), accelBiasX(nImuBiasKeys), accelBiasY(nImuBiasKeys), accelBiasZ(nImuBiasKeys);
        for(int i=0; i<nImuBiasKeys; i++){ // iterate through and collect values for imu bias, do math
            gtsam::imuBias::ConstantBias imuBias=vals.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol(m_imuBiasVarChar,i));
            gtsam::Vector3 accelBias=imuBias.accelerometer(); accelBiasX[i]=accelBias[0]; accelBiasY[i]=accelBias[1]; accelBiasZ[i]=accelBias[2];
            gtsam::Vector3 gyroBias=imuBias.gyroscope(); gyroBiasX[i]=gyroBias[0]; gyroBiasY[i]=gyroBias[1]; gyroBiasZ[i]=gyroBias[2];
        }
        // float average = accumulate( v.begin(), v.end(), 0.0)/v.size();
        double gyroBiasXAvg=std::accumulate(gyroBiasX.begin(), gyroBiasX.end(), 0.0)/gyroBiasX.size();
        double gyroBiasYAvg=std::accumulate(gyroBiasY.begin(), gyroBiasY.end(), 0.0)/gyroBiasY.size();
        double gyroBiasZAvg=std::accumulate(gyroBiasZ.begin(), gyroBiasZ.end(), 0.0)/gyroBiasZ.size();
        double accelBiasXAvg=std::accumulate(accelBiasX.begin(), accelBiasX.end(), 0.0)/accelBiasX.size();
        double accelBiasYAvg=std::accumulate(accelBiasY.begin(), accelBiasY.end(), 0.0)/accelBiasY.size();
        double accelBiasZAvg=std::accumulate(accelBiasZ.begin(), accelBiasZ.end(), 0.0)/accelBiasZ.size();
        std::cout<<std::string((size_t)nSpacesToIndent,' ')<<"dynamic bias: avg. gyro=["<<gyroBiasXAvg<<", "<<gyroBiasYAvg<<", "<<gyroBiasZAvg<<"] | avg. accel=["<<accelBiasXAvg<<", "<<accelBiasYAvg<<", "<<accelBiasZAvg<<"]"<<std::endl;
    }else{std::cerr<<"number of imubias keys found ("<<gtsamutils::getNumberOfFactorByType<gtsam::imuBias::ConstantBias>(m_graph)<<") is in consistent with imu bias mode that was set ("<<imuBiasModelMode<<")"<<std::endl;}
}

double imuPoseEstimator::getMagFactorErrorAtState(const gtsam::Values& vals){
    std::vector<double> error, errorSq;
    // get keys from thigh and shank pose problems---will need to compare with keys from this problem to map which is in the thigh pose problem and which is shank
    gtsam::KeyVector imuPose3Keys=((gtsam::Values)this->m_estimate.filter<gtsam::Pose3>()).keys();
    // loop over all knee graph factors, finding gyro rates and poses associated with them and run evaluateError()
    for (gtsam::NonlinearFactor::shared_ptr & factor : m_graph) { // you could change this to the graph from the imu pose problem to only loop over those keys, I think.
        auto a = boost::dynamic_pointer_cast<bioslam::MagPose3Factor>(factor);
        if (a) { // you found a MagPose3Factor
            gtsam::Pose3 imuPose;
            gtsam::KeyVector allKeys=a->keys(); // keys attached to this mag factor
            for(gtsam::Key & k: allKeys){
                // now loop over the variables keys, finding the thighImu and shankImu pose variables attached to this factor
                auto it = find(imuPose3Keys.begin(), imuPose3Keys.end(), k);
                if(it!=imuPose3Keys.end()) { // you found the key in the imu pose problem
                    imuPose = vals.at<gtsam::Pose3>(k);
                }else{
                    //cout<<"ERROR: could not find key "<<k<<" in imu pose problem"<<endl; // this means it was in the other imu's pose problem
                }
            } // finish loop over keys
            // now compute error
            gtsam::Vector err=a->evaluateError(imuPose);
            error.push_back(err.norm()); errorSq.push_back(pow(err.norm(),2));
        }
    }
    //  after loop, compute RMSE from error vector and return
    //double rmse = sqrt(std::accumulate(errorSq.begin(), errorSq.end(), 0.0));
    double SumNormdError = std::accumulate(error.begin(), error.end(), 0.0);
    return SumNormdError;
}

double imuPoseEstimator::getGenericImuFactorErrorAtState(const gtsam::Values& vals){
    // switch what type of factor to get error for
    double ImuFactorError=0.0;
    if(imuBiasModelMode==0){ // static bias, use ImuFactor
        ImuFactorError=getImuFactorErrorAtState(vals);
    }else if(imuBiasModelMode==1){ // dynamic bias, use CombinedImuFactor
        ImuFactorError=getCombinedImuFactorErrorAtState(vals);
    }else{std::cerr<<"This should not happen. Choose correct imuBiasModelMode"<<std::endl;}
    return ImuFactorError;
}

double imuPoseEstimator::getCombinedImuFactorErrorAtState(const gtsam::Values& vals){
    std::vector<double> error, errorSq;
    gtsam::KeyVector imuPose3Keys=((gtsam::Values)this->m_estimate.filter<gtsam::Pose3>()).keys();
    gtsam::KeyVector imuBiasKeys=((gtsam::Values)this->m_estimate.filter<gtsam::imuBias::ConstantBias>()).keys();
    // loop over all knee graph factors, finding gyro rates and poses associated with them and run evaluateError()
    for (gtsam::NonlinearFactor::shared_ptr & factor : m_graph) {
        auto a = boost::dynamic_pointer_cast<gtsam::CombinedImuFactor>(factor);
        if (a) { // you found a ImuFactor
            gtsam::Pose3 imuPose_i, imuPose_j;
            gtsam::Vector3 vel_i, vel_j;
            gtsam::imuBias::ConstantBias bias_i, bias_j;
            gtsam::KeyVector allKeys=a->keys();
            // assign keys
            gtsam::Key imuPose_i_key=allKeys[0];
            gtsam::Key vel_i_key=allKeys[1];
            gtsam::Key imuPose_j_key=allKeys[2];
            gtsam::Key vel_j_key=allKeys[3];
            gtsam::Key imuBias_i_key=allKeys[4];
            gtsam::Key imuBias_j_key=allKeys[5];
            // now check to make sure they're in the right graph
            bool foundKeys=false;
            auto it_pose_i=find(imuPose3Keys.begin(), imuPose3Keys.end(), imuPose_i_key);
            if(it_pose_i!=imuPose3Keys.end()){
                foundKeys=true;
                //cout<<"found these keys in the imu problem"<<endl;
            }else{
                //cout<<"could not find keys in this imu problem"<<endl;
            }
            if(foundKeys){
                // now compute error
                imuPose_i=vals.at<gtsam::Pose3>(imuPose_i_key);
                imuPose_j=vals.at<gtsam::Pose3>(imuPose_j_key);
                vel_i=vals.at<gtsam::Vector3>(vel_i_key);
                vel_j=vals.at<gtsam::Vector3>(vel_j_key);
                bias_i=vals.at<gtsam::imuBias::ConstantBias>(imuBias_i_key);
                bias_j=vals.at<gtsam::imuBias::ConstantBias>(imuBias_j_key);
                gtsam::Vector err=a->evaluateError(imuPose_i,vel_i,imuPose_j,vel_j,bias_i,bias_j);
                error.push_back(err.norm()); errorSq.push_back(pow(err.norm(),2));
            }
        }
    }
    //  after loop, compute RMSE from error vector and return
    //double rmse = sqrt(std::accumulate(errorSq.begin(), errorSq.end(), 0.0));
    double sumNormdErr=0.0;
    sumNormdErr = std::accumulate(error.begin(), error.end(), 0.0);
    return sumNormdErr;
}

double imuPoseEstimator::getImuFactorErrorAtState(const gtsam::Values& vals){
    std::vector<double> error, errorSq;
    // get keys from thigh and shank pose problems---will need to compare with keys from this problem to map which is in the thigh pose problem and which is shank
    gtsam::KeyVector imuPose3Keys=((gtsam::Values)this->m_estimate.filter<gtsam::Pose3>()).keys();
    // loop over all knee graph factors, finding gyro rates and poses associated with them and run evaluateError()
    for (gtsam::NonlinearFactor::shared_ptr & factor : m_graph) {
        auto a = boost::dynamic_pointer_cast<gtsam::ImuFactor>(factor);
        if (a) { // you found a ImuFactor
            gtsam::Pose3 imuPose_i, imuPose_j;
            gtsam::Vector3 vel_i, vel_j;
            gtsam::imuBias::ConstantBias bias;
            gtsam::KeyVector allKeys=a->keys();
            // assign keys
            gtsam::Key imuPose_i_key=allKeys[0];
            gtsam::Key vel_i_key=allKeys[1];
            gtsam::Key imuPose_j_key=allKeys[2];
            gtsam::Key vel_j_key=allKeys[3];
            gtsam::Key imuBias_key=allKeys[4];
            // now check to make sure they're in the right graph
            bool foundKeys=false;
            auto it_pose_i=find(imuPose3Keys.begin(), imuPose3Keys.end(), imuPose_i_key);
            if(it_pose_i!=imuPose3Keys.end()){
                foundKeys=true;
                //cout<<"found these keys in the imu problem"<<endl;
            }else{
                //cout<<"could not find keys in this imu problem"<<endl;
            }
            if(foundKeys){
                // now compute error
                imuPose_i=vals.at<gtsam::Pose3>(imuPose_i_key);
                imuPose_j=vals.at<gtsam::Pose3>(imuPose_j_key);
                vel_i=vals.at<gtsam::Vector3>(vel_i_key);
                vel_j=vals.at<gtsam::Vector3>(vel_j_key);
                bias=vals.at<gtsam::imuBias::ConstantBias>(imuBias_key);
                gtsam::Vector err=a->evaluateError(imuPose_i,vel_i,imuPose_j,vel_j,bias);
                error.push_back(err.norm()); errorSq.push_back(pow(err.norm(),2));
            }
        }
    }
    //  after loop, compute RMSE from error vector and return
    //double rmse = sqrt(std::accumulate(errorSq.begin(), errorSq.end(), 0.0));
    double sumNormdErr = std::accumulate(error.begin(), error.end(), 0.0);
    return sumNormdErr;
}

void imuPoseEstimator::defaultOptimize(){
    std::cout<<std::endl<<"*** imuPoseEstimator::defaultOptimize() results for "<<m_strId<<" ***"<<std::endl;
    std::cout<<"    convergence criteria: abs. error decrease < "<<m_absErrDecreaseLimit<<" OR rel. error decrease < "<<m_relErrDecreaseLimit*100<<"% OR max iterations = "<<m_maxIterations<<std::endl;
    gtsam::LevenbergMarquardtParams params; params.setVerbosityLM("SUMMARY"); params.setVerbosity("TERMINATION"); params.setlambdaLowerBound(1.0e-8); params.setlambdaUpperBound(1.0e10);
    params.setAbsoluteErrorTol(m_absErrDecreaseLimit); params.setRelativeErrorTol(m_relErrDecreaseLimit); params.setMaxIterations(m_maxIterations);
    gtsam::LevenbergMarquardtOptimizer optimizer(m_graph,m_initialValues,params);
    // print initial state
    std::cout<<" --- initial state ----"<<std::endl;
    std::cout<<"    error: "<<optimizer.error()<<std::endl;
    gtsamutils::printErrorsInGraphByFactorType(m_graph,m_initialValues);
    double timeBeforeIterations=clock();
    m_estimate=optimizer.optimize(); // run optimizer!
    std::cout<<"----------------------"<<std::endl;
    std::cout<<"Final state ("<<optimizer.iterations()<<" iterations, total time "<<(clock()-timeBeforeIterations)/CLOCKS_PER_SEC<<" sec):"<<std::endl;
    std::cout<<"    error: "<<optimizer.error()<<std::endl;
    std::cout<<"********** end fast optimization **********"<<std::endl<<std::endl;
    // pull out estimated Values and print graph errors by factor type
    gtsamutils::printErrorsInGraphByFactorType(m_graph,m_estimate); // final errors in graph by type
    setMemberStatesFromValues(m_estimate); // set member states from values
}

void imuPoseEstimator::print(){
    std::cout<<"printing imuPoseEstimator problem setup for IMU: "<<std::endl;
    std::cout<<"    NonlinearFactorGraph with "<<m_graph.size()<<" variables"<<std::endl;
}

gtsam::Rot3 imuPoseEstimator::singleSampleDavenportQMethod(const Eigen::Vector3d& accelBody, const Eigen::Vector3d& magBody, const Eigen::Vector3d& accelNWU, const Eigen::Vector3d& magNWU, double wa, double wm){
    // Davenport's q method
    // Davenport, P., “A vector approach to the algebra of rotations with applications,” NASA, X-546-65-437, 1965.
    // This method serves to compute attitude matrix A[nav->body] s.t.,
    // A*r=b
    // the attitude determination problem is finding the orthogonal matrix A that minimizes Wahba's loss function:
    // L(A)=1/2*SUM_i=1:n(w*|b(i)-A*r(i)|^2)
    // this optimal solution is given by the eigenvector (representing a quaternion q=[x y z w]) corresponding to the largest
    //      eigenvector of the 4x4 matrix K:
    // K=[B+B'-trace(B)*eye(3), z; z', trace(B)];
    //    where:  B=SUM_OVER_MEASUREMENTS(w*b*r') [3x3] -- there are two measurements here (accel and mag)
    //            z=[B(2,3)-B(3,2); B(3,1)-B(1,3); B(1,2)-B(2,1)];  [3x1]
    // weights must sum to unity
    // measurements b and r are [3x1]
    // optional: print data
    //std::cout<<"accelVec=["<<accelBody.transpose()<<"] | magVec=["<<magBody.transpose()<<"]"<<std::endl;
    // Step (1): make sure weights sum to unity
    wa=wa/(wa+wm); wm=wm/(wa+wm);
    // (2) construct matrix B
    Eigen::Matrix3d B;
    Eigen::Matrix3d Kul3; // the upper left 3x3 matrix entry in K
    Eigen::Vector3d z;
    Eigen::MatrixXd K(4,4);
    B=wa*accelBody*accelNWU.transpose()+wm*magBody*magNWU.transpose();
    // (3) construct upper left 3x3 entry in K and z
    Kul3=B+B.transpose()-Eigen::MatrixXd::Identity(3,3)*B.trace();
    z[0]=B(1,2)-B(2,1); z[1]=B(2,0)-B(0,2); z[2]=B(0,1)-B(1,0);
    // (4) construct 4x4 K matrix
    K(0,0)=Kul3(0,0); K(0,1)=Kul3(0,1); K(0,2)=Kul3(0,2);
    K(1,0)=Kul3(1,0); K(1,1)=Kul3(1,1); K(1,2)=Kul3(1,2);
    K(2,0)=Kul3(2,0); K(2,1)=Kul3(2,1); K(2,2)=Kul3(2,2);
    K(0,3)=z[0]; K(1,3)=z[1]; K(2,3)=z[2];
    K(3,0)=z[0]; K(3,1)=z[1]; K(3,2)=z[2];
    K(3,3)=B.trace();
    // (5) find the 4 eigenvalues of K
    Eigen::EigenSolver<Eigen::MatrixXd> es(K);
    Eigen::VectorXcd eigvals=K.eigenvalues();
    Eigen::MatrixXcd eigvecs=es.eigenvectors();
    // (6) now find largest eigenvalue
    int maxIdx=0; double maxVal=0;
    for (int i=0;i<4;i++){
        // verify that eigenvalues are all real
        double absImag=abs(eigvals[i].imag()); // abs value of imaginary part
        if(absImag>1e-5) {
            std::cout<<"K=\n" << K <<std::endl;
            std::cout<<"eigenvalues(K)=\n" << eigvals <<std::endl;
            std::cout<<"eigenvectors(K)=\n" << eigvecs <<std::endl;
            assert(absImag < 1e-5);
        }
        if(eigvals[i].real()>maxVal){
            maxVal=eigvals[i].real();
            maxIdx=i;
        }
    }
    // (7) now pull out the eigenvector corresponding to maxVal
    // NOTE: this comes out with a scalar-last quaternion. switch to scalar-first for gtsam usage.
    Eigen::Vector4d q_opt;
    q_opt[0]=eigvecs(3,maxIdx).real();
    q_opt[1]=eigvecs(0,maxIdx).real();
    q_opt[2]=eigvecs(1,maxIdx).real();
    q_opt[3]=eigvecs(2,maxIdx).real();
    return gtsam::Rot3(q_opt[0],q_opt[1],q_opt[2],q_opt[3]);
}

void imuPoseEstimator::clear(){
    // clear any stored results
    m_orientation.clear();
    m_position.clear();
    m_velocity.clear();
    m_time.clear();
}


Eigen::MatrixXd imuPoseEstimator::getOrientationPrincipalVariance(){
    gtsam::Marginals marginals(m_graph,m_estimate);
    uint nGraphSteps=m_imu.length()/m_numMeasToPreint;
    Eigen::MatrixXd prinVariance(nGraphSteps,3);
    for(int i=0;i<nGraphSteps;i++){
        Eigen::MatrixXd covariance=marginals.marginalCovariance(gtsam::Symbol(m_poseVarChar, i));
        prinVariance(i,0)=covariance(0,0);
        prinVariance(i,1)=covariance(1,1);
        prinVariance(i,2)=covariance(2,2);
    }
    return prinVariance;
}

Eigen::MatrixXd imuPoseEstimator::getPositionPrincipalVariance(){
    gtsam::Marginals marginals(m_graph,m_estimate);
    uint nGraphSteps=m_imu.length()/m_numMeasToPreint;
    Eigen::MatrixXd prinVariance(nGraphSteps,3);
    for(int i=0;i<nGraphSteps;i++){
        Eigen::MatrixXd covariance=marginals.marginalCovariance(gtsam::Symbol(m_poseVarChar, i));
        prinVariance(i,0)=covariance(3,3);
        prinVariance(i,1)=covariance(4,4);
        prinVariance(i,2)=covariance(5,5);
    }
    return prinVariance;
}

void imuPoseEstimator::printYprToFile(std::string filename) const {
    std::ofstream myfile;
    myfile.open(filename);
    int nSteps=m_imu.length()/m_numMeasToPreint;
    for(int i=0;i<nSteps;i++){
        gtsam::Vector3 ypr=m_orientation[i].ypr();
        //myfile<<m_time[i]<<","<<ypr[0]<<","<<ypr[1]<<","<<ypr[2]<<","<<m_orientationPrincipalVariance3(i,0)<<","<<m_orientationPrincipalVariance3(i,1)<<","<<m_orientationPrincipalVariance3(i,2)<<endl;
        // in my LM smoother I don't get principal variance, so don't print that.
        myfile<<m_time[i]<<","<<ypr[0]<<","<<ypr[1]<<","<<ypr[2]<<std::endl;
    }
    myfile.close();
}

void imuPoseEstimator::printQuatToFile(std::string filename) const {
    std::ofstream myfile;
    myfile.open(filename);
    int nSteps=m_imu.length()/m_numMeasToPreint;
    for(int i=0;i<nSteps;i++){
        gtsam::Vector q=m_orientation[i].quaternion();
        //Rot3 varAsRot3=Rot3::ypr(m_orientationPrincipalVariance3(i,0),m_orientationPrincipalVariance3(i,1),m_orientationPrincipalVariance3(i,2));
        //Vector qVar=varAsRot3.quaternion(); // that variance represented as a differential quaternion
        // note: don't print variance right now; my LM estimator doesn't allow for it.
        myfile<<m_time[i]<<","<<q[0]<<","<<q[1]<<","<<q[2]<<","<<q[3]<<std::endl;
    }
    myfile.close();
}

void imuPoseEstimator::printDcmToFile(std::string filename) const {
    // print as a flatted 9x1 DCM instead of a quaternion. print by rows.
    std::ofstream myfile;
    myfile.open(filename);
    int nSteps=m_imu.length()/m_numMeasToPreint;
    for(int i=0;i<nSteps;i++){
        gtsam::Matrix33 r=m_orientation[i].matrix();
        //Rot3 varAsRot3=Rot3::ypr(m_orientationPrincipalVariance3(i,0),m_orientationPrincipalVariance3(i,1),m_orientationPrincipalVariance3(i,2));
        //Vector qVar=varAsRot3.quaternion(); // that variance represented as a differential quaternion
        // note: don't print variance right now; my LM estimator doesn't allow for it.
        myfile<<m_time[i]<<","<<r(0,0)<<","<<r(0,1)<<","<<r(0,2)<<","<<r(1,0)<<","<<r(1,1)<<","<<r(1,2)<<","<<r(2,0)<<","<<r(2,1)<<","<<r(2,2)<<std::endl;
    }
    myfile.close();
}


void imuPoseEstimator::printPosToFile(std::string filename) const {
    std::ofstream myfile;
    myfile.open(filename);
    int nSteps=m_imu.length()/m_numMeasToPreint;
    for(int i=0;i<nSteps;i++){
        gtsam::Vector3 pos=m_position[i];
        myfile<<m_time[i]<<","<<pos[0]<<","<<pos[1]<<","<<pos[2]<<std::endl;
        // note: took out position variance; can't get that from my LM estimator.
    }
    myfile.close();
}

void imuPoseEstimator::printAccelBiasToFile(std::string filename) const {
    std::ofstream myfile;
    myfile.open(filename);
    uint nSteps=m_imu.length()/m_numMeasToPreint;
    size_t nAccelBiasStates=m_accelBias.size();
    if(nAccelBiasStates==1) { // is statis bias
        gtsam::Vector3 bias=m_accelBias[0];
        myfile<<0.0<<","<<bias[0]<<","<<bias[1]<<","<<bias[2]<<std::endl;
    }else if(nAccelBiasStates==m_orientation.size()) { // dynamic bias
        for(size_t i=0;i<nSteps;i++){
            gtsam::Vector3 bias=m_accelBias[i];
            myfile<<m_time[i]<<","<<bias[0]<<","<<bias[1]<<","<<bias[2]<<std::endl;
            // note: took out position variance; can't get that from my LM estimator.
        }
    }else{std::cout<<"ERROR: must have either 1 IMU bias state (static) or the same number as orientations ("<<m_orientation.size()<<", dynamic), but had "<<nAccelBiasStates<<std::endl;}
    myfile.close();
}

void imuPoseEstimator::printGyroBiasToFile(std::string filename) const {
    std::ofstream myfile;
    myfile.open(filename);
    uint nSteps=m_imu.length()/m_numMeasToPreint;
    uint nGyroBiasStates=m_gyroBias.size();
    if(nGyroBiasStates==1) { // is statis bias
        gtsam::Vector3 bias=m_gyroBias[0];
        myfile<<0.0<<","<<bias[0]<<","<<bias[1]<<","<<bias[2]<<std::endl;
    }else if(nGyroBiasStates==m_orientation.size()) { // dynamic bias
        for(uint i=0;i<nSteps;i++){
            gtsam::Vector3 bias=m_gyroBias[i];
            myfile<<m_time[i]<<","<<bias[0]<<","<<bias[1]<<","<<bias[2]<<std::endl;
            // note: took out position variance; can't get that from my LM estimator.
        }
    }else{std::cout<<"ERROR: must have either 1 IMU bias state (static) or the same number as orientations ("<<m_orientation.size()<<", dynamic), but had "<<nGyroBiasStates<<std::endl;}
    myfile.close();
}

void imuPoseEstimator::setImuBiasModelMode(uint modelMode) {
    imuBiasModelMode=modelMode;
}

uint imuPoseEstimator::getImuBiasModelMode(bool printVerbose=false) {
    return imuBiasModelMode;
}

void imuPoseEstimator::printResultsToConsole() const {
    std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> q=mathutils::Rot3VectorToQuaternionVector(m_orientation);
    std::vector<Eigen::Vector3d> v=m_velocity; std::vector<gtsam::Point3> p=m_position;
    std::vector<Eigen::Vector3d> accelBias=m_accelBias;
    std::vector<Eigen::Vector3d> gyroBias=m_gyroBias;
    bool isGyroBiasStatic=true;
    bool isAccelBiasStatic=true;
    if(gyroBias.size()>1){isGyroBiasStatic=false;}
    if(accelBias.size()>1){isAccelBiasStatic=false;}

    for(size_t i=0;i<m_orientation.size();i++){
        // print all states in a loop
        std::cout<<"t["<<i<<"]="<<m_time[i]<<": q=["<<q[i][0]<<","<<q[i][1]<<","<<q[i][2]<<","<<q[i][3]<<"]";
        std::cout<<" | v=["<<v[i][0]<<","<<v[i][1]<<","<<v[i][2]<<"] | p=["<<p[i].x()<<","<<p[i].y()<<","<<p[i].z()<<"]";
        if(!isGyroBiasStatic){
            std::cout<<" | gyroBias=["<<gyroBias[i][0]<<","<<gyroBias[i][1]<<","<<gyroBias[i][2]<<"]"<<std::endl;
        }
        if(!isAccelBiasStatic) {
            std::cout << " | accelBias=[" << accelBias[i][0] << "," << accelBias[i][1] << "," << accelBias[i][2] << "]";
        }
        std::cout<<std::endl;
    }
    if(isGyroBiasStatic){
        std::cout<<" ... static gyroBias estimate=["<<gyroBias[0][0]<<","<<gyroBias[0][1]<<","<<gyroBias[0][2]<<"]"<<std::endl;
    }
    if(isAccelBiasStatic) {
        std::cout << " ... static accelBias estimate=[" << accelBias[0][0] << "," << accelBias[0][1] << "," << accelBias[0][2] << "]"<<std::endl;
    }
}

void imuPoseEstimator::adjustImuTrajectoryByDeltaRot(std::vector<gtsam::Pose3>& imuPoseTrajectory, std::vector<gtsam::Vector3>& imuVelTrajectory, const gtsam::Rot3& deltaRot){
    // vectorized version of function of same name
    if(imuPoseTrajectory.size()!=imuVelTrajectory.size()){throw std::runtime_error("wrong size");}
    for(uint k=0;k<imuPoseTrajectory.size();k++){
        adjustImuTrajectoryByDeltaRot(imuPoseTrajectory[k], imuVelTrajectory[k], deltaRot);
    }
}

void imuPoseEstimator::adjustImuTrajectoryByDeltaRot(std::vector<gtsam::Pose3>& imuPoseTrajectory, std::vector<gtsam::Vector3>& imuVelTrajectory, const std::vector<gtsam::Rot3>& deltaRot){
    // vectorized version of function of same name
    if(imuPoseTrajectory.size()!=imuVelTrajectory.size()){throw std::runtime_error("wrong size");}
    if(imuPoseTrajectory.size()!=deltaRot.size()){throw std::runtime_error("wrong size");}
    for(uint k=0;k<imuPoseTrajectory.size();k++){
        adjustImuTrajectoryByDeltaRot(imuPoseTrajectory[k], imuVelTrajectory[k], deltaRot[k]);
    }
}

std::vector<gtsam::Vector3> imuPoseEstimator::consistentVelocitySetFromPositions(const std::vector<gtsam::Pose3>& poses, const double& dt){
    return imuPoseEstimator::consistentVelocitySetFromPositions(gtsamutils::Pose3VectorToPoint3Vector(poses),dt);
}

std::vector<gtsam::Vector3> imuPoseEstimator::consistentVelocitySetFromPositions(const std::vector<gtsam::Point3>& pos, const double& dt){
    // from the input set of positions, compute velocities as the discrete derivatives of position
    // assumes velocity begins at zero and dt is input in seconds
    std::vector<gtsam::Vector3> vel(pos.size());
    vel[0]=gtsam::Vector3(0.0,0.0,0.0);
    for(uint k=1; k<pos.size(); k++){
        vel[k]=(pos[k]-pos[k-1])/dt;
    }
    return vel;
}

void imuPoseEstimator::adjustImuTrajectoryByDeltaRot(gtsam::Pose3& imuPoseTrajectory, gtsam::Vector3& imuVelTrajectory, const gtsam::Rot3& deltaRot){
    // deltaRot is assumed to be R[B->B2], where B2 is the new coordinate system
    // originally, you have R[B->N]. you want to construct R[B2->N], where B2 is a coordiante system very similar to B, but slightly rotated about the global vertical axis (assuming +Z)
    // update all orientations by the heading rotation offset
    gtsam::Rot3 origRot3=imuPoseTrajectory.rotation(); // this is R[B->N]
    gtsam::Rot3 newRot3=(deltaRot*origRot3.inverse()).inverse(); // this is R[B2->N]: reconstruct the same Rot3 but with a small change to yaw
    // update positions
    gtsam::Point3 newPoint3=deltaRot*imuPoseTrajectory.translation();
    gtsam::Pose3 newPose3=gtsam::Pose3(newRot3,newPoint3);
    // update velocities
    gtsam::Vector3 newVel=deltaRot*(imuVelTrajectory);
    // now overwrite input variables
    imuPoseTrajectory=newPose3; imuVelTrajectory=newVel;
}

std::vector<gtsam::Pose3> imuPoseEstimator::vectorizePoses(const gtsam::Values& vals, const gtsam::KeyVector& poseKeys){
    // put requested type in a std::vector
    std::vector<gtsam::Pose3> poses(poseKeys.size());
    for(uint k=0; k<poseKeys.size(); k++){
        poses[k]=vals.at<gtsam::Pose3>(poseKeys[k]);
    }
    return poses;
}

std::vector<gtsam::Rot3> imuPoseEstimator::vectorizeOrientations(const gtsam::Values& vals, const gtsam::KeyVector& poseKeys){
    // put requested type in a std::vector
    std::vector<gtsam::Pose3> poses=vectorizePoses(vals,poseKeys);
    std::vector<gtsam::Rot3> orientations(poses.size());
    for(uint k=0; k<poses.size(); k++){
        orientations[k]= poses[k].rotation() ;
    }
    return orientations;
}

std::vector<gtsam::Rot3> imuPoseEstimator::vectorizeOrientations(const std::vector<gtsam::Pose3>& poses){
    // put requested type in a std::vector
    std::vector<gtsam::Rot3> orientations(poses.size());
    for(uint k=0; k<poses.size(); k++){
        orientations[k]= poses[k].rotation() ;
    }
    return orientations;
}

std::vector<gtsam::Point3> imuPoseEstimator::vectorizePositions(const gtsam::Values& vals, const gtsam::KeyVector& poseKeys){
    // put requested type in a std::vector
    std::vector<gtsam::Pose3> poses=vectorizePoses(vals,poseKeys);
    std::vector<gtsam::Point3> pos(poses.size());
    for(uint k=0; k<poses.size(); k++){
        pos[k]= poses[k].translation();
    }
    return pos;
}

std::vector<gtsam::Point3> imuPoseEstimator::vectorizePositions(const std::vector<gtsam::Pose3>& poses){
    // put requested type in a std::vector
    std::vector<gtsam::Point3> pos(poses.size());
    for(uint k=0; k<poses.size(); k++){
        pos[k]= poses[k].translation();
    }
    return pos;
}

std::vector<gtsam::Vector3> imuPoseEstimator::vectorizeVelocities(const gtsam::Values& vals, const gtsam::KeyVector& velKeys){
    // put requested type in a std::vector
    std::vector<gtsam::Vector3> vel(velKeys.size());
    for(uint k=0; k<velKeys.size(); k++){
        vel[k]= vals.at<gtsam::Vector3>(velKeys[k]);
    }
    return vel;
}

std::vector<gtsam::Vector3> imuPoseEstimator::vectorizeGyroBiases(const gtsam::Values& vals, const gtsam::KeyVector& biasKeys){
    // put requested type in a std::vector
    // warning: this assumes dynamic biases (length of biasKeys>1)
    std::vector<gtsam::Vector3> gyrobiases(biasKeys.size());
    for(uint k=0; k<biasKeys.size(); k++){
        gyrobiases[k] = (vals.at<gtsam::imuBias::ConstantBias>(biasKeys[k])).gyroscope();
    }
    return gyrobiases;
}
std::vector<gtsam::Vector3> imuPoseEstimator::vectorizeAccelBiases(const gtsam::Values& vals, const gtsam::KeyVector& biasKeys){
    // put requested type in a std::vector
    // warning: this assumes dynamic biases (length of biasKeys>1)
    std::vector<gtsam::Vector3> accelbiases(biasKeys.size());
    for(uint k=0; k<biasKeys.size(); k++){
        accelbiases[k] = (vals.at<gtsam::imuBias::ConstantBias>(biasKeys[k])).accelerometer();
    }
    return accelbiases;
}

std::vector<gtsam::Vector3> imuPoseEstimator::getImuLinearAccel() const {
    // post-hoc, return the linear body accelerations (or "gravity-subtracted" acceleration measurements). Also subtract accel bias.
    // see Eq. 21 in Carlone and Forster's paper: http://www.roboticsproceedings.org/rss11/p06.pdf
    // model is: accel meas = R[B->N]'*(linear_accel[N] - g[N]) + bias + noise
    // therefore linear accel = R[B->N] (meas - bias + R[B->N]'*g[N])
    std::vector<Eigen::Vector3d> accelBias=m_accelBias;
    std::vector<gtsam::Rot3> R_B_to_N=m_orientation; // should be R[B->N]
    gtsam::Vector3 gN=m_globalAcc;
    bool isAccelBiasStatic=true;
    if(accelBias.size()>1){isAccelBiasStatic=false;}
    // now loop through key frames, pull out closest accelerometer measurement, and then apply the above equation
    uint nearestIdx;
    std::vector<gtsam::Vector3> linearAccel(m_orientation.size());
    for(size_t i=0;i<m_orientation.size();i++){
        // first get closest index in the measurement time domain
        nearestIdx=mathutils::findIdxOfNearestValInArray(m_imu.relTimeSec, m_time[i]);
        gtsam::Vector3 accelMeas(m_imu.ax[nearestIdx],m_imu.ay[nearestIdx],m_imu.az[nearestIdx]);
        gtsam::Vector3 myBias;
        if(isAccelBiasStatic){
            myBias=accelBias[0];
        }else{
            myBias=accelBias[i];
        }
        gtsam::Vector3 gB=R_B_to_N[i].inverse()*gN; // gravity rotated into body frame
        linearAccel[i]=R_B_to_N[i]*(accelMeas-myBias+gB);
        //std::cout<<"R[B->N]=..., accelMeas[B]=["<<accelMeas[0]<<", "<<accelMeas[1]<<", "<<accelMeas[2]<<"], bias[B]=["<<myBias[0]<<", "<<myBias[1]<<", "<<myBias[2]<<"], g[B]=["<<gB[0]<<", "<<gB[1]<<", "<<gB[2]<<"]"<<std::endl;
    }
    return linearAccel;
}

bool imuPoseEstimator::testHeadingCanBeSpunWithNoChangeToError(const gtsam::NonlinearFactorGraph& inputgraph, const gtsam::Values& inputvals, bool verbose) const {
    // tests intuition that we can spin an IMU pose around in yaw with no change to error in the graph
    // take in a graph and a set of values, removes everything that isn't a CombinedImuFactor
    // --- current state of this test is ---
    // it works fine. a static offset rotation pureYawR (see other function) is constructed, and updating the position and velocities to the new yawed orientation is simply equivalent to applying pureYawR to each of the positions and velocities.
    // --- GTSAM notes on frames from NavState.h: ---
    // Rot3 R_; ///< Rotation nRb, rotates points/velocities in body to points/velocities in nav
    // Point3 t_; ///< position n_t, in nav frame
    // Velocity3 v_; ///< velocity n_v in nav frame
    // when you construct a NavState from a Pose3 and velocity, it simply copies them over with no transformations. So I think this description of the frames in NavState is reliable.
    // --- Conclusions from Tim on frames: ---
    // orientations in Pose3 and NavState are R[B->N]
    // (still working on figuring this one out) positions and velocities are expressed in the nav frame, but are the vector from the local frame to the global frame [B->N] (?)
    //     current unanswered question: we know that the positions and velocities are represented in the nav frame, but do they point from the body frame to the global frame origin or vice versa? (I *think* it's from global origin to body origin)
    // ------- remember in GTSAM that composition of two Pose3s is ------
    // // given a new Pose3 T...
    //Pose3 operator*(const Pose3& T) const {
    //    return Pose3(R_ * T.R_, t_ + R_ * T.t_);
    //}
    // call our old frame B, and the new frame B2. then via deduction, the transform T has to represent the orientation R[B2->B]
    //     --> b/c orientation is computed as: R[B2->N] = R[B->N] * R[B2->B]
    //     position is computed as: new pos = (oldpos) + R[B->N]*(delta t)
    // -----------------------------------------------------------------
    // (1) copy graph and values
    gtsam::NonlinearFactorGraph graph=inputgraph;
    gtsam::Values vals=inputvals;
    // (2) now edit graph to remove everything that isn't a combinedimufactor
    gtsamutils::removeAllFactorsExceptTypeInPlace<gtsam::CombinedImuFactor>(graph, false);
    gtsamutils::printErrorsInGraphByFactorType(graph,vals);
    // (3) now loop through the values, update them, and test new error
    double headingAngleDeltaRad=0.1;
    double origGraphErr=graph.error(vals);
    uint numTries=40;
    bool doesTestPass=true;
    for(uint i=0; i<numTries; i++){
        yawImuStatesByConstantValueInPlace(vals, headingAngleDeltaRad, m_poseKeys, m_velKeys); // performs values update
        // () now check error. is it the same?
        double graphErr=graph.error(vals);
        if(abs(graphErr-origGraphErr)<1.0e-5){ // same, as expected
            doesTestPass=true;
            if(verbose){std::cout<<"    iteration "<<i<<": error = "<<graphErr<<" (same as original)"<<std::endl;}
        }else{ // different, not good!
            doesTestPass=false;
            std::cerr<<"    iteration "<<i<<": error = "<<graphErr<<" (DIFFERENT from original. test fails!)"<<std::endl;
        }
    }
    if(verbose){std::cout<<"does test pass = "<<doesTestPass<<std::endl;}
    if(!doesTestPass){std::cerr<<"test failed"<<std::endl;}
    return doesTestPass;
}

void imuPoseEstimator::yawImuStatesByConstantValueInPlace(gtsam::Values& vals, const double& headingAngleDeltaRad, const gtsam::KeyVector& poseKeys, const gtsam::KeyVector& velKeys){
    // in place edits a set of Values (vals) to yaw just the imu states which are described by keys poseKeys and velKeys.
    gtsam::Rot3 pureYawR=gtsam::Rot3::Ypr(headingAngleDeltaRad,0.0,0.0); // note: this is equivalent to gtsam::Rot3::Rz(headingAngleDeltaRad)
    for (uint k=0; k<poseKeys.size(); k++){
        // originally, you have R[B->N]. you want to construct R[B2->N], where B2 is a coordiante system very similar to B, but slightly rotated about the global vertical axis (assuming +Z)
        // update all orientations by the heading rotation offset
        gtsam::Pose3 origPose3=vals.at<gtsam::Pose3>(poseKeys[k]);
        gtsam::Rot3 origRot3=origPose3.rotation(); // this is R[B->N]
        gtsam::Rot3 newRot3=gtsam::Rot3::Ypr(origRot3.yaw()+headingAngleDeltaRad,origRot3.pitch(),origRot3.roll()); // this is R[B2->N]: reconstruct the same Rot3 but with a small change to yaw
        // update positions
        gtsam::Point3 origPoint3=origPose3.translation();
        gtsam::Point3 newPoint3=pureYawR*(origPoint3);
        gtsam::Pose3 newPose3=gtsam::Pose3(newRot3,newPoint3);
        vals.update(poseKeys[k], newPose3 ); // <-- update
        // update velocities
        gtsam::Vector3 origVel=vals.at<gtsam::Vector3>(velKeys[k]);
        gtsam::Vector3 newVel=pureYawR*(origVel);
        vals.update(velKeys[k], newVel); // <-- update
    }
}

gtsam::Values imuPoseEstimator::yawImuStatesByConstantValueWithCopy(const gtsam::Values& vals, const double& headingAngleDeltaRad, const gtsam::KeyVector& poseKeys, const gtsam::KeyVector& velKeys){
    // same as similar function, but makes a copy of the values and returns it.
    gtsam::Values newvals=gtsam::Values(vals);
    yawImuStatesByConstantValueInPlace(newvals,headingAngleDeltaRad,poseKeys,velKeys);
    return newvals;
}

void imuPoseEstimator::printStateAtKeyframe(const gtsam::Values& vals, uint k, const std::string& tag){
    gtsam::NavState state_j=gtsam::NavState(vals.at<gtsam::Pose3>(gtsam::Symbol(m_poseVarChar, k)), vals.at<gtsam::Vector3>(gtsam::Symbol(m_velVarChar, k))); // state @ k
    gtsam::imuBias::ConstantBias bias_j = vals.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol(m_imuBiasVarChar, k));
    std::cout << tag << ": q[" << k << "]: [" << state_j.pose().rotation().quaternion().transpose();
    std::cout << "]  |  p[" << k << "]: [" << state_j.pose().translation().transpose();
    std::cout << "]  |  v[" << k << "]: [" << state_j.velocity().transpose();
    std::cout << "]  |  biasGyro[" << k << "]: [" << bias_j.gyroscope().transpose();
    std::cout << "]  |  biasAcc[" << k << "]: [" << bias_j.accelerometer().transpose() << "]" << std::endl;
}

