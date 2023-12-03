// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include <VarStrToCharMap.h>
#include <gtsam/inference/Symbol.h>
#include <factors/ConstrainedJointCenterPositionFactor.h>
#include <factors/SegmentLengthMagnitudeFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <factors/AngularVelocityFactor.h>
#include <factors/HingeJointFactors.h>
#include <factors/ConstrainedJointCenterVelocityFactor.h>
#include <fstream>
#include <factors/SegmentLengthDiscrepancyFactor.h>
#include <factors/AngleBetweenAxisAndSegmentFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <boost/filesystem.hpp>
#include "gtsamutils.h"
#include "bioutils.h"
#include "lowerBodyPoseEstimator.h"
#include "highfive/H5Easy.hpp"
#include "factors/Pose3Priors.h"
#include "factors/Point3Priors.h"

void lowerBodyPoseEstimator::addInstantaneousAngVelEstimationToImuPoses(uint initMode) {
    // you have 7 IMUs. Each has a trajectory of poses, related through preintegration of the gyroscope measurements.
    // this method adds both the new variables and the constraints to estimate angular velocity at keyframe k
    // this is closely related to the measured angular velocity, but with bias removed and a noise distribution
    // this is only in the bioestimator functions -- not imuPoseEstimator, because it's useless for a single IMU.
    // --- options for initMode --- //
    // 0: init to zero, 1: init to measured gyro rate, 2: init to measured gyro rate - bias
    // ---------------------------- //
    uint N=m_sacrumImuPoseProblem.m_poseKeys.size();
    // () make sure imubias keys are either of size 1 or the same size as pose keys
    if(m_sacrumImuPoseProblem.m_imuBiasKeys.size()!=1 && m_sacrumImuPoseProblem.m_imuBiasKeys.size()!=m_sacrumImuPoseProblem.m_poseKeys.size()){throw std::runtime_error("wrong size imu bias keys for sacrum imu.");}
    if(m_rthighImuPoseProblem.m_imuBiasKeys.size()!=1 && m_rthighImuPoseProblem.m_imuBiasKeys.size()!=m_rthighImuPoseProblem.m_poseKeys.size()){throw std::runtime_error("wrong size imu bias keys for rthigh imu.");}
    if(m_rshankImuPoseProblem.m_imuBiasKeys.size()!=1 && m_rshankImuPoseProblem.m_imuBiasKeys.size()!=m_rshankImuPoseProblem.m_poseKeys.size()){throw std::runtime_error("wrong size imu bias keys for rshank imu.");}
    if(m_rfootImuPoseProblem.m_imuBiasKeys.size()!=1 && m_rfootImuPoseProblem.m_imuBiasKeys.size()!=m_rfootImuPoseProblem.m_poseKeys.size()){throw std::runtime_error("wrong size imu bias keys for rfoot imu.");}
    if(m_lthighImuPoseProblem.m_imuBiasKeys.size()!=1 && m_lthighImuPoseProblem.m_imuBiasKeys.size()!=m_lthighImuPoseProblem.m_poseKeys.size()){throw std::runtime_error("wrong size imu bias keys for lthigh imu.");}
    if(m_lshankImuPoseProblem.m_imuBiasKeys.size()!=1 && m_lshankImuPoseProblem.m_imuBiasKeys.size()!=m_lshankImuPoseProblem.m_poseKeys.size()){throw std::runtime_error("wrong size imu bias keys for lshank imu.");}
    if(m_lfootImuPoseProblem.m_imuBiasKeys.size()!=1 && m_lfootImuPoseProblem.m_imuBiasKeys.size()!=m_lfootImuPoseProblem.m_poseKeys.size()){throw std::runtime_error("wrong size imu bias keys for lfoot imu.");}
    // () get var chars and strs for the new estimated angular velocity variable
    m_sacrumImuAngVelVarStr = m_strId + "_sacrumImuAngVel";
    m_sacrumImuAngVelVarChar = VarStrToCharMap::insert(m_sacrumImuAngVelVarStr);
    m_rthighImuAngVelVarStr = m_strId + "_rthighImuAngVel";
    m_rthighImuAngVelVarChar = VarStrToCharMap::insert(m_rthighImuAngVelVarStr);
    m_rshankImuAngVelVarStr = m_strId + "_rshankImuAngVel";
    m_rshankImuAngVelVarChar = VarStrToCharMap::insert(m_rshankImuAngVelVarStr);
    m_rfootImuAngVelVarStr = m_strId + "_rfootImuAngVel";
    m_rfootImuAngVelVarChar = VarStrToCharMap::insert(m_rfootImuAngVelVarStr);
    m_lthighImuAngVelVarStr = m_strId + "_lthighImuAngVel";
    m_lthighImuAngVelVarChar = VarStrToCharMap::insert(m_lthighImuAngVelVarStr);
    m_lshankImuAngVelVarStr = m_strId + "_lshankImuAngVel";
    m_lshankImuAngVelVarChar = VarStrToCharMap::insert(m_lshankImuAngVelVarStr);
    m_lfootImuAngVelVarStr = m_strId + "_lfootImuAngVel";
    m_lfootImuAngVelVarChar = VarStrToCharMap::insert(m_lfootImuAngVelVarStr);
    // () now construct the arrays to hold the keys
    m_sacrumImuAngVelKeys.resize(m_sacrumImuPoseProblem.m_poseKeys.size());
    m_rthighImuAngVelKeys.resize(m_rthighImuPoseProblem.m_poseKeys.size());
    m_rshankImuAngVelKeys.resize(m_rshankImuPoseProblem.m_poseKeys.size());
    m_rfootImuAngVelKeys.resize(m_rfootImuPoseProblem.m_poseKeys.size());
    m_lthighImuAngVelKeys.resize(m_lthighImuPoseProblem.m_poseKeys.size());
    m_lshankImuAngVelKeys.resize(m_lshankImuPoseProblem.m_poseKeys.size());
    m_lfootImuAngVelKeys.resize(m_lfootImuPoseProblem.m_poseKeys.size());
    for (uint k = 0; k < m_sacrumImuAngVelKeys.size(); k++) {
        m_sacrumImuAngVelKeys[k] = gtsam::Symbol(m_sacrumImuAngVelVarChar, k);
        m_rthighImuAngVelKeys[k] = gtsam::Symbol(m_rthighImuAngVelVarChar, k);
        m_rshankImuAngVelKeys[k] = gtsam::Symbol(m_rshankImuAngVelVarChar, k);
        m_rfootImuAngVelKeys[k] = gtsam::Symbol(m_rfootImuAngVelVarChar, k);
        m_lthighImuAngVelKeys[k] = gtsam::Symbol(m_lthighImuAngVelVarChar, k);
        m_lshankImuAngVelKeys[k] = gtsam::Symbol(m_lshankImuAngVelVarChar, k);
        m_lfootImuAngVelKeys[k] = gtsam::Symbol(m_lfootImuAngVelVarChar, k);
    }
    // () setup array to deal with imu bias indexes. if dynamic bias, the array will simply be 1:N. if static, it'll be an N-vector of zeros.
    std::vector<uint> sacrumImuBiasIdxs(N), rthighImuBiasIdxs(N), rshankImuBiasIdxs(N), rfootImuBiasIdxs(N), lthighImuBiasIdxs(N), lshankImuBiasIdxs(N), lfootImuBiasIdxs(N);
    if(m_sacrumImuPoseProblem.m_imuBiasKeys.size()==1){for(uint k=0; k<N; k++){sacrumImuBiasIdxs[k]=0;}}else{for(uint k=0; k<N; k++){sacrumImuBiasIdxs[k]=k;}}
    if(m_rthighImuPoseProblem.m_imuBiasKeys.size()==1){for(uint k=0; k<N; k++){rthighImuBiasIdxs[k]=0;}}else{for(uint k=0; k<N; k++){rthighImuBiasIdxs[k]=k;}}
    if(m_rshankImuPoseProblem.m_imuBiasKeys.size()==1){for(uint k=0; k<N; k++){rshankImuBiasIdxs[k]=0;}}else{for(uint k=0; k<N; k++){rshankImuBiasIdxs[k]=k;}}
    if(m_rfootImuPoseProblem.m_imuBiasKeys.size()==1){for(uint k=0; k<N; k++){rfootImuBiasIdxs[k]=0;}}else{for(uint k=0; k<N; k++){rfootImuBiasIdxs[k]=k;}}
    if(m_lthighImuPoseProblem.m_imuBiasKeys.size()==1){for(uint k=0; k<N; k++){lthighImuBiasIdxs[k]=0;}}else{for(uint k=0; k<N; k++){lthighImuBiasIdxs[k]=k;}}
    if(m_lshankImuPoseProblem.m_imuBiasKeys.size()==1){for(uint k=0; k<N; k++){lshankImuBiasIdxs[k]=0;}}else{for(uint k=0; k<N; k++){lshankImuBiasIdxs[k]=k;}}
    if(m_lfootImuPoseProblem.m_imuBiasKeys.size()==1){for(uint k=0; k<N; k++){lfootImuBiasIdxs[k]=0;}}else{for(uint k=0; k<N; k++){lfootImuBiasIdxs[k]=k;}}
    // () now loop and setup factors and insert initial values`
    std::vector<uint> imuIdxArray = m_sacrumImuPoseProblem.getImuIndecesCorrespondingToKeyframes(); // assume these are all the same
    for (uint k = 0; k < m_sacrumImuAngVelKeys.size(); k++){
        gtsam::Vector3 sacrumImuGyros(m_sacrumImuPoseProblem.m_imu.gx[imuIdxArray[k]],m_sacrumImuPoseProblem.m_imu.gy[imuIdxArray[k]],m_sacrumImuPoseProblem.m_imu.gz[imuIdxArray[k]] );
        gtsam::Vector3 rthighImuGyros(m_rthighImuPoseProblem.m_imu.gx[imuIdxArray[k]],m_rthighImuPoseProblem.m_imu.gy[imuIdxArray[k]],m_rthighImuPoseProblem.m_imu.gz[imuIdxArray[k]]);
        gtsam::Vector3 rshankImuGyros(m_rshankImuPoseProblem.m_imu.gx[imuIdxArray[k]],m_rshankImuPoseProblem.m_imu.gy[imuIdxArray[k]],m_rshankImuPoseProblem.m_imu.gz[imuIdxArray[k]]);
        gtsam::Vector3 rfootImuGyros(m_rfootImuPoseProblem.m_imu.gx[imuIdxArray[k]],m_rfootImuPoseProblem.m_imu.gy[imuIdxArray[k]],m_rfootImuPoseProblem.m_imu.gz[imuIdxArray[k]]);
        gtsam::Vector3 lthighImuGyros(m_lthighImuPoseProblem.m_imu.gx[imuIdxArray[k]],m_lthighImuPoseProblem.m_imu.gy[imuIdxArray[k]],m_lthighImuPoseProblem.m_imu.gz[imuIdxArray[k]]);
        gtsam::Vector3 lshankImuGyros(m_lshankImuPoseProblem.m_imu.gx[imuIdxArray[k]],m_lshankImuPoseProblem.m_imu.gy[imuIdxArray[k]],m_lshankImuPoseProblem.m_imu.gz[imuIdxArray[k]]);
        gtsam::Vector3 lfootImuGyros(m_lfootImuPoseProblem.m_imu.gx[imuIdxArray[k]],m_lfootImuPoseProblem.m_imu.gy[imuIdxArray[k]],m_lfootImuPoseProblem.m_imu.gz[imuIdxArray[k]]);
        m_graph.push_back(bioslam::AngularVelocityFactor(m_sacrumImuAngVelKeys[k],m_sacrumImuPoseProblem.m_imuBiasKeys[sacrumImuBiasIdxs[k]],sacrumImuGyros,m_angVelNoiseModel));
        m_graph.push_back(bioslam::AngularVelocityFactor(m_rthighImuAngVelKeys[k],m_rthighImuPoseProblem.m_imuBiasKeys[rthighImuBiasIdxs[k]],rthighImuGyros,m_angVelNoiseModel));
        m_graph.push_back(bioslam::AngularVelocityFactor(m_rshankImuAngVelKeys[k],m_rshankImuPoseProblem.m_imuBiasKeys[rshankImuBiasIdxs[k]],rshankImuGyros,m_angVelNoiseModel));
        m_graph.push_back(bioslam::AngularVelocityFactor(m_rfootImuAngVelKeys[k],m_rfootImuPoseProblem.m_imuBiasKeys[rfootImuBiasIdxs[k]],rfootImuGyros,m_angVelNoiseModel));
        m_graph.push_back(bioslam::AngularVelocityFactor(m_lthighImuAngVelKeys[k],m_lthighImuPoseProblem.m_imuBiasKeys[lthighImuBiasIdxs[k]],lthighImuGyros,m_angVelNoiseModel));
        m_graph.push_back(bioslam::AngularVelocityFactor(m_lshankImuAngVelKeys[k],m_lshankImuPoseProblem.m_imuBiasKeys[lshankImuBiasIdxs[k]],lshankImuGyros,m_angVelNoiseModel));
        m_graph.push_back(bioslam::AngularVelocityFactor(m_lfootImuAngVelKeys[k],m_lfootImuPoseProblem.m_imuBiasKeys[lfootImuBiasIdxs[k]],lfootImuGyros,m_angVelNoiseModel));
        // --- initialize values --- //
        gtsam::Vector3 initSacrumAngVel, initRThighAngVel, initRShankAngVel, initRFootAngVel, initLThighAngVel, initLShankAngVel, initLFootAngVel;
        if(initMode==0){ // set to zero
            initSacrumAngVel=gtsam::Vector3::Zero(), initRThighAngVel=gtsam::Vector3::Zero(), initRShankAngVel=gtsam::Vector3::Zero(), initRFootAngVel=gtsam::Vector3::Zero(), initLThighAngVel=gtsam::Vector3::Zero(), initLShankAngVel=gtsam::Vector3::Zero(), initLFootAngVel=gtsam::Vector3::Zero();
        }else if(initMode==1){ // set to measured gyro rate
            initSacrumAngVel=sacrumImuGyros;
            initRThighAngVel=rthighImuGyros;
            initRShankAngVel=rshankImuGyros;
            initRFootAngVel=rfootImuGyros;
            initLThighAngVel=lthighImuGyros;
            initLShankAngVel=lshankImuGyros;
            initLFootAngVel=lfootImuGyros;
        }else if(initMode==2){
            // set initial value equal to measured gyro - bias (error = true + bias - meas ~ N(0,sigma) --> true ~ meas - bias)
            //      this means that the error in the graph due to AngularVelocityFactor should always start at zero!
            initSacrumAngVel=sacrumImuGyros - m_sacrumImuPoseProblem.m_gyroBias[sacrumImuBiasIdxs[k]];
            initRThighAngVel=rthighImuGyros - m_rthighImuPoseProblem.m_gyroBias[rthighImuBiasIdxs[k]];
            initRShankAngVel=rshankImuGyros - m_rshankImuPoseProblem.m_gyroBias[rshankImuBiasIdxs[k]];
            initRFootAngVel=rfootImuGyros - m_rfootImuPoseProblem.m_gyroBias[rfootImuBiasIdxs[k]];
            initLThighAngVel=lthighImuGyros - m_lthighImuPoseProblem.m_gyroBias[lthighImuBiasIdxs[k]];
            initLShankAngVel=lshankImuGyros - m_lshankImuPoseProblem.m_gyroBias[lshankImuBiasIdxs[k]];
            initLFootAngVel=lfootImuGyros - m_lfootImuPoseProblem.m_gyroBias[lfootImuBiasIdxs[k]];
        }
        m_initialValues.insert(m_sacrumImuAngVelKeys[k],initSacrumAngVel);
        m_initialValues.insert(m_rthighImuAngVelKeys[k],initRThighAngVel);
        m_initialValues.insert(m_rshankImuAngVelKeys[k],initRShankAngVel);
        m_initialValues.insert(m_rfootImuAngVelKeys[k],initRFootAngVel);
        m_initialValues.insert(m_lthighImuAngVelKeys[k],initLThighAngVel);
        m_initialValues.insert(m_lshankImuAngVelKeys[k],initLShankAngVel);
        m_initialValues.insert(m_lfootImuAngVelKeys[k],initLFootAngVel);
    }
}

void lowerBodyPoseEstimator::addImuProblemSolutionsToInitialValues(){
    // pull out values from individual pose problems and add them to initial estimate
    m_initialValues.insert(m_sacrumImuPoseProblem.m_estimate);
    m_initialValues.insert(m_rthighImuPoseProblem.m_estimate);
    m_initialValues.insert(m_rshankImuPoseProblem.m_estimate);
    m_initialValues.insert(m_rfootImuPoseProblem.m_estimate);
    m_initialValues.insert(m_lthighImuPoseProblem.m_estimate);
    m_initialValues.insert(m_lshankImuPoseProblem.m_estimate);
    m_initialValues.insert(m_lfootImuPoseProblem.m_estimate);
}

void lowerBodyPoseEstimator::addImuProblemGraphsToGraph(){
    m_graph.push_back(m_sacrumImuPoseProblem.m_graph);
    m_graph.push_back(m_rthighImuPoseProblem.m_graph);
    m_graph.push_back(m_rshankImuPoseProblem.m_graph);
    m_graph.push_back(m_rfootImuPoseProblem.m_graph);
    m_graph.push_back(m_lthighImuPoseProblem.m_graph);
    m_graph.push_back(m_lshankImuPoseProblem.m_graph);
    m_graph.push_back(m_lfootImuPoseProblem.m_graph);
}

void lowerBodyPoseEstimator::add6JointSharedPosConstraints(){
    // adds shared joint centers of rotation models for all 6 joints to the factor graph
    if(m_jointPosFactorDim == 3){ // use ConstrainedJointCenterPositionFactor
        for(uint i=0; i<m_rthighImuPoseProblem.m_poseKeys.size();i++){ // loop through, making factor connections
            // add position constraint factor to joint rotation centers
            m_graph +=bioslam::ConstrainedJointCenterPositionFactor(m_sacrumImuPoseProblem.m_poseKeys[i], m_rthighImuPoseProblem.m_poseKeys[i], m_sacrumImuToRHipCtrKey, m_rthighImuToHipCtrKey, m_hipJointCtrConnectionNoiseModel); // rhip constraint
            m_graph +=bioslam::ConstrainedJointCenterPositionFactor(m_rthighImuPoseProblem.m_poseKeys[i], m_rshankImuPoseProblem.m_poseKeys[i], m_rthighImuToKneeCtrKey, m_rshankImuToKneeCtrKey, m_kneeJointCtrConnectionNoiseModel); // rknee
            m_graph +=bioslam::ConstrainedJointCenterPositionFactor(m_rshankImuPoseProblem.m_poseKeys[i], m_rfootImuPoseProblem.m_poseKeys[i], m_rshankImuToAnkleCtrKey, m_rfootImuToAnkleCtrKey, m_ankleJointCtrConnectionNoiseModel); // rankle
            m_graph +=bioslam::ConstrainedJointCenterPositionFactor(m_sacrumImuPoseProblem.m_poseKeys[i], m_lthighImuPoseProblem.m_poseKeys[i], m_sacrumImuToLHipCtrKey, m_lthighImuToHipCtrKey, m_hipJointCtrConnectionNoiseModel); // lhip constraint
            m_graph +=bioslam::ConstrainedJointCenterPositionFactor(m_lthighImuPoseProblem.m_poseKeys[i], m_lshankImuPoseProblem.m_poseKeys[i], m_lthighImuToKneeCtrKey, m_lshankImuToKneeCtrKey, m_kneeJointCtrConnectionNoiseModel); // lknee
            m_graph +=bioslam::ConstrainedJointCenterPositionFactor(m_lshankImuPoseProblem.m_poseKeys[i], m_lfootImuPoseProblem.m_poseKeys[i], m_lshankImuToAnkleCtrKey, m_lfootImuToAnkleCtrKey, m_ankleJointCtrConnectionNoiseModel); // lankle
        }
    }else if(m_jointPosFactorDim == 1){ // use ImuPoseToJointCtrNormFactor
        for(uint i=0; i<m_rthighImuPoseProblem.m_poseKeys.size();i++){ // loop through, making factor connections
            // add position constraint factor to joint rotation centers
            m_graph +=bioslam::ConstrainedJointCenterNormPositionFactor(m_sacrumImuPoseProblem.m_poseKeys[i], m_rthighImuPoseProblem.m_poseKeys[i], m_sacrumImuToRHipCtrKey, m_rthighImuToHipCtrKey, m_hipJointCtrConnectionNoiseModel); // rhip constraint
            m_graph +=bioslam::ConstrainedJointCenterNormPositionFactor(m_rthighImuPoseProblem.m_poseKeys[i], m_rshankImuPoseProblem.m_poseKeys[i], m_rthighImuToKneeCtrKey, m_rshankImuToKneeCtrKey, m_kneeJointCtrConnectionNoiseModel); // rknee
            m_graph +=bioslam::ConstrainedJointCenterNormPositionFactor(m_rshankImuPoseProblem.m_poseKeys[i], m_rfootImuPoseProblem.m_poseKeys[i], m_rshankImuToAnkleCtrKey, m_rfootImuToAnkleCtrKey, m_ankleJointCtrConnectionNoiseModel); // rankle
            m_graph +=bioslam::ConstrainedJointCenterNormPositionFactor(m_sacrumImuPoseProblem.m_poseKeys[i], m_lthighImuPoseProblem.m_poseKeys[i], m_sacrumImuToLHipCtrKey, m_lthighImuToHipCtrKey, m_hipJointCtrConnectionNoiseModel); // lhip constraint
            m_graph +=bioslam::ConstrainedJointCenterNormPositionFactor(m_lthighImuPoseProblem.m_poseKeys[i], m_lshankImuPoseProblem.m_poseKeys[i], m_lthighImuToKneeCtrKey, m_lshankImuToKneeCtrKey, m_kneeJointCtrConnectionNoiseModel); // lknee
            m_graph +=bioslam::ConstrainedJointCenterNormPositionFactor(m_lshankImuPoseProblem.m_poseKeys[i], m_lfootImuPoseProblem.m_poseKeys[i], m_lshankImuToAnkleCtrKey, m_lfootImuToAnkleCtrKey, m_ankleJointCtrConnectionNoiseModel); // lankle
        }
    }else{throw std::runtime_error("unknown factor dimension");}
}

void lowerBodyPoseEstimator::add6JointSharedVelConstraints(){
    // adds shared joint velocities for all 6 joints to the factor graph
    if(m_jointVelFactorDim == 3){ // use vector error model: ConstrainedJointCenterVelocityFactor
        for(uint k=0; k<m_sacrumImuPoseProblem.m_poseKeys.size(); k++){
            m_graph += bioslam::ConstrainedJointCenterVelocityFactor(m_sacrumImuPoseProblem.m_poseKeys[k],m_sacrumImuPoseProblem.m_velKeys[k],m_sacrumImuAngVelKeys[k],m_sacrumImuToRHipCtrKey,m_rthighImuPoseProblem.m_poseKeys[k],m_rthighImuPoseProblem.m_velKeys[k],m_rthighImuAngVelKeys[k],m_rthighImuToHipCtrKey,m_jointCtrVelConnectionNoiseModel);
            m_graph += bioslam::ConstrainedJointCenterVelocityFactor(m_rthighImuPoseProblem.m_poseKeys[k],m_rthighImuPoseProblem.m_velKeys[k],m_rthighImuAngVelKeys[k],m_rthighImuToKneeCtrKey,m_rshankImuPoseProblem.m_poseKeys[k],m_rshankImuPoseProblem.m_velKeys[k],m_rshankImuAngVelKeys[k],m_rshankImuToKneeCtrKey,m_jointCtrVelConnectionNoiseModel);
            m_graph += bioslam::ConstrainedJointCenterVelocityFactor(m_rshankImuPoseProblem.m_poseKeys[k],m_rshankImuPoseProblem.m_velKeys[k],m_rshankImuAngVelKeys[k],m_rshankImuToAnkleCtrKey,m_rfootImuPoseProblem.m_poseKeys[k],m_rfootImuPoseProblem.m_velKeys[k],m_rfootImuAngVelKeys[k],m_rfootImuToAnkleCtrKey,m_jointCtrVelConnectionNoiseModel);
            m_graph += bioslam::ConstrainedJointCenterVelocityFactor(m_sacrumImuPoseProblem.m_poseKeys[k],m_sacrumImuPoseProblem.m_velKeys[k],m_sacrumImuAngVelKeys[k],m_sacrumImuToLHipCtrKey,m_lthighImuPoseProblem.m_poseKeys[k],m_lthighImuPoseProblem.m_velKeys[k],m_lthighImuAngVelKeys[k],m_lthighImuToHipCtrKey,m_jointCtrVelConnectionNoiseModel);
            m_graph += bioslam::ConstrainedJointCenterVelocityFactor(m_lthighImuPoseProblem.m_poseKeys[k],m_lthighImuPoseProblem.m_velKeys[k],m_lthighImuAngVelKeys[k],m_lthighImuToKneeCtrKey,m_lshankImuPoseProblem.m_poseKeys[k],m_lshankImuPoseProblem.m_velKeys[k],m_lshankImuAngVelKeys[k],m_lshankImuToKneeCtrKey,m_jointCtrVelConnectionNoiseModel);
            m_graph += bioslam::ConstrainedJointCenterVelocityFactor(m_lshankImuPoseProblem.m_poseKeys[k],m_lshankImuPoseProblem.m_velKeys[k],m_lshankImuAngVelKeys[k],m_lshankImuToAnkleCtrKey,m_lfootImuPoseProblem.m_poseKeys[k],m_lfootImuPoseProblem.m_velKeys[k],m_lfootImuAngVelKeys[k],m_lfootImuToAnkleCtrKey,m_jointCtrVelConnectionNoiseModel);
        }
    }else if(m_jointVelFactorDim == 1){ // use norm error model: ConstrainedJointCenterNormVelocityFactor
        for(uint k=0; k<m_sacrumImuPoseProblem.m_poseKeys.size(); k++){
            m_graph += bioslam::ConstrainedJointCenterNormVelocityFactor(m_sacrumImuPoseProblem.m_poseKeys[k], m_sacrumImuPoseProblem.m_velKeys[k], m_sacrumImuAngVelKeys[k], m_sacrumImuToRHipCtrKey, m_rthighImuPoseProblem.m_poseKeys[k], m_rthighImuPoseProblem.m_velKeys[k], m_rthighImuAngVelKeys[k], m_rthighImuToHipCtrKey, m_jointCtrVelConnectionNoiseModel);
            m_graph += bioslam::ConstrainedJointCenterNormVelocityFactor(m_rthighImuPoseProblem.m_poseKeys[k], m_rthighImuPoseProblem.m_velKeys[k], m_rthighImuAngVelKeys[k], m_rthighImuToKneeCtrKey, m_rshankImuPoseProblem.m_poseKeys[k], m_rshankImuPoseProblem.m_velKeys[k], m_rshankImuAngVelKeys[k], m_rshankImuToKneeCtrKey, m_jointCtrVelConnectionNoiseModel);
            m_graph += bioslam::ConstrainedJointCenterNormVelocityFactor(m_rshankImuPoseProblem.m_poseKeys[k], m_rshankImuPoseProblem.m_velKeys[k], m_rshankImuAngVelKeys[k], m_rshankImuToAnkleCtrKey, m_rfootImuPoseProblem.m_poseKeys[k], m_rfootImuPoseProblem.m_velKeys[k], m_rfootImuAngVelKeys[k], m_rfootImuToAnkleCtrKey, m_jointCtrVelConnectionNoiseModel);
            m_graph += bioslam::ConstrainedJointCenterNormVelocityFactor(m_sacrumImuPoseProblem.m_poseKeys[k], m_sacrumImuPoseProblem.m_velKeys[k], m_sacrumImuAngVelKeys[k], m_sacrumImuToLHipCtrKey, m_lthighImuPoseProblem.m_poseKeys[k], m_lthighImuPoseProblem.m_velKeys[k], m_lthighImuAngVelKeys[k], m_lthighImuToHipCtrKey, m_jointCtrVelConnectionNoiseModel);
            m_graph += bioslam::ConstrainedJointCenterNormVelocityFactor(m_lthighImuPoseProblem.m_poseKeys[k], m_lthighImuPoseProblem.m_velKeys[k], m_lthighImuAngVelKeys[k], m_lthighImuToKneeCtrKey, m_lshankImuPoseProblem.m_poseKeys[k], m_lshankImuPoseProblem.m_velKeys[k], m_lshankImuAngVelKeys[k], m_lshankImuToKneeCtrKey, m_jointCtrVelConnectionNoiseModel);
            m_graph += bioslam::ConstrainedJointCenterNormVelocityFactor(m_lshankImuPoseProblem.m_poseKeys[k], m_lshankImuPoseProblem.m_velKeys[k], m_lshankImuAngVelKeys[k], m_lshankImuToAnkleCtrKey, m_lfootImuPoseProblem.m_poseKeys[k], m_lfootImuPoseProblem.m_velKeys[k], m_lfootImuAngVelKeys[k], m_lfootImuToAnkleCtrKey, m_jointCtrVelConnectionNoiseModel);
        }
    }else{throw std::runtime_error("unknown factor dimension");}
}

void lowerBodyPoseEstimator::addHipHingeConstraints(){
    // adds all hip hinge constraints to graph given program options
    // --- setup keys to variables --- //
    addHipHingeAxisVariableKeys();
    // ---- add proximal frame (sacrum IMU) factors ---- //
    if(m_hingeAxisFactorDim==3){ // use vector error model
        for(uint k=0; k<m_rthighImuPoseProblem.m_poseKeys.size(); k++){
            m_graph += bioslam::HingeJointConstraintVecErrEstAngVel(m_sacrumImuPoseProblem.m_poseKeys[k], m_sacrumImuAngVelKeys[k], m_rthighImuPoseProblem.m_poseKeys[k], m_rthighImuAngVelKeys[k], m_hipAxisSacrumKey, m_hipHingeAxisNoiseModel);
            m_graph += bioslam::HingeJointConstraintVecErrEstAngVel(m_sacrumImuPoseProblem.m_poseKeys[k], m_sacrumImuAngVelKeys[k], m_lthighImuPoseProblem.m_poseKeys[k], m_lthighImuAngVelKeys[k], m_hipAxisSacrumKey, m_hipHingeAxisNoiseModel);
            m_graph += bioslam::HingeJointConstraintVecErrEstAngVel(m_rthighImuPoseProblem.m_poseKeys[k], m_rthighImuAngVelKeys[k], m_sacrumImuPoseProblem.m_poseKeys[k], m_sacrumImuAngVelKeys[k], m_rhipAxisThighKey, m_hipHingeAxisNoiseModel);
            m_graph += bioslam::HingeJointConstraintVecErrEstAngVel(m_lthighImuPoseProblem.m_poseKeys[k], m_lthighImuAngVelKeys[k], m_sacrumImuPoseProblem.m_poseKeys[k], m_sacrumImuAngVelKeys[k], m_lhipAxisThighKey, m_hipHingeAxisNoiseModel);
        }
    }else if(m_hingeAxisFactorDim==1){ // use norm error model
        for(uint k=0; k<m_rthighImuPoseProblem.m_poseKeys.size(); k++){
            m_graph += bioslam::HingeJointConstraintNormErrEstAngVel(m_sacrumImuPoseProblem.m_poseKeys[k], m_sacrumImuAngVelKeys[k], m_rthighImuPoseProblem.m_poseKeys[k], m_rthighImuAngVelKeys[k], m_hipAxisSacrumKey, m_hipHingeAxisNoiseModel);
            m_graph += bioslam::HingeJointConstraintNormErrEstAngVel(m_sacrumImuPoseProblem.m_poseKeys[k], m_sacrumImuAngVelKeys[k], m_lthighImuPoseProblem.m_poseKeys[k], m_lthighImuAngVelKeys[k], m_hipAxisSacrumKey, m_hipHingeAxisNoiseModel);
            m_graph += bioslam::HingeJointConstraintNormErrEstAngVel(m_rthighImuPoseProblem.m_poseKeys[k], m_rthighImuAngVelKeys[k], m_sacrumImuPoseProblem.m_poseKeys[k], m_sacrumImuAngVelKeys[k], m_rhipAxisThighKey, m_hipHingeAxisNoiseModel);
            m_graph += bioslam::HingeJointConstraintNormErrEstAngVel(m_lthighImuPoseProblem.m_poseKeys[k], m_lthighImuAngVelKeys[k], m_sacrumImuPoseProblem.m_poseKeys[k], m_sacrumImuAngVelKeys[k], m_lhipAxisThighKey, m_hipHingeAxisNoiseModel);
        }
    }else{throw std::runtime_error("unknown factor dimension");}
    // add initial values for hip hinge axes
    m_initialValues.insert(m_rhipAxisThighKey,priorRHipAxisThigh);
    m_initialValues.insert(m_lhipAxisThighKey,priorLHipAxisThigh);
    m_initialValues.insert(m_hipAxisSacrumKey,priorHipAxisSacrum);
    // if requested, add priors on these axes
    if(m_usePriorsOnHipHingeAxes){
        m_graph.push_back(gtsam::PriorFactor<gtsam::Unit3>(m_rhipAxisThighKey,priorRHipAxisThigh,gtsam::noiseModel::Diagonal::Sigmas(rhipHingeAxisThighPriorStd, true)));
        m_graph.push_back(gtsam::PriorFactor<gtsam::Unit3>(m_lhipAxisThighKey,priorLHipAxisThigh,gtsam::noiseModel::Diagonal::Sigmas(lhipHingeAxisThighPriorStd, true)));
        m_graph.push_back(gtsam::PriorFactor<gtsam::Unit3>(m_hipAxisSacrumKey,priorHipAxisSacrum,gtsam::noiseModel::Diagonal::Sigmas(hipHingeAxisSacrumPriorStd, true)));
    }
}

void lowerBodyPoseEstimator::addKneeHingeConstraints(){
    // adds all knee hinge constraints to graph given program options
    uint nKeyframes=m_rthighImuPoseProblem.m_poseKeys.size();
    if(m_hingeAxisFactorDim==3){ // use vector model
        for(uint k=0; k<nKeyframes; k++){
            m_graph += bioslam::HingeJointConstraintVecErrEstAngVel(m_rthighImuPoseProblem.m_poseKeys[k], m_rthighImuAngVelKeys[k], m_rshankImuPoseProblem.m_poseKeys[k], m_rshankImuAngVelKeys[k], m_rkneeAxisThighKey, m_kneeHingeAxisNoiseModel);
            m_graph += bioslam::HingeJointConstraintVecErrEstAngVel(m_rshankImuPoseProblem.m_poseKeys[k], m_rshankImuAngVelKeys[k], m_rthighImuPoseProblem.m_poseKeys[k], m_rthighImuAngVelKeys[k], m_rkneeAxisShankKey, m_kneeHingeAxisNoiseModel);
            m_graph += bioslam::HingeJointConstraintVecErrEstAngVel(m_lthighImuPoseProblem.m_poseKeys[k], m_lthighImuAngVelKeys[k], m_lshankImuPoseProblem.m_poseKeys[k], m_lshankImuAngVelKeys[k], m_lkneeAxisThighKey, m_kneeHingeAxisNoiseModel);
            m_graph += bioslam::HingeJointConstraintVecErrEstAngVel(m_lshankImuPoseProblem.m_poseKeys[k], m_lshankImuAngVelKeys[k], m_lthighImuPoseProblem.m_poseKeys[k], m_lthighImuAngVelKeys[k], m_lkneeAxisShankKey, m_kneeHingeAxisNoiseModel);
        }
    }else if(m_hingeAxisFactorDim==1){ // use norm model
        for(uint k=0; k<nKeyframes; k++){
            m_graph += bioslam::HingeJointConstraintNormErrEstAngVel(m_rthighImuPoseProblem.m_poseKeys[k], m_rthighImuAngVelKeys[k], m_rshankImuPoseProblem.m_poseKeys[k], m_rshankImuAngVelKeys[k], m_rkneeAxisThighKey, m_kneeHingeAxisNoiseModel);
            m_graph += bioslam::HingeJointConstraintNormErrEstAngVel(m_rshankImuPoseProblem.m_poseKeys[k], m_rshankImuAngVelKeys[k], m_rthighImuPoseProblem.m_poseKeys[k], m_rthighImuAngVelKeys[k], m_rkneeAxisShankKey, m_kneeHingeAxisNoiseModel);
            m_graph += bioslam::HingeJointConstraintNormErrEstAngVel(m_lthighImuPoseProblem.m_poseKeys[k], m_lthighImuAngVelKeys[k], m_lshankImuPoseProblem.m_poseKeys[k], m_lshankImuAngVelKeys[k], m_lkneeAxisThighKey, m_kneeHingeAxisNoiseModel);
            m_graph += bioslam::HingeJointConstraintNormErrEstAngVel(m_lshankImuPoseProblem.m_poseKeys[k], m_lshankImuAngVelKeys[k], m_lthighImuPoseProblem.m_poseKeys[k], m_lthighImuAngVelKeys[k], m_lkneeAxisShankKey, m_kneeHingeAxisNoiseModel);
        }
    }else{throw std::runtime_error("unknown factor dimension");}
}

void lowerBodyPoseEstimator::addAnkleHingeConstraints(){
    // adds all ankle hinge constraints to graph given program options
    // --- setup keys to variables --- //
    addAnkleHingeAxisVariableKeys();
    // ---- add distal frame (foot IMU) factors ---- //
    if(m_hingeAxisFactorDim==3){ // use vector error model
        for(uint k=0; k<m_rshankImuPoseProblem.m_poseKeys.size(); k++){
            m_graph += bioslam::HingeJointConstraintVecErrEstAngVel(m_rfootImuPoseProblem.m_poseKeys[k], m_rfootImuAngVelKeys[k], m_rshankImuPoseProblem.m_poseKeys[k], m_rshankImuAngVelKeys[k], m_rankleAxisFootKey, m_ankleHingeAxisNoiseModel);
            m_graph += bioslam::HingeJointConstraintVecErrEstAngVel(m_lfootImuPoseProblem.m_poseKeys[k], m_lfootImuAngVelKeys[k], m_lshankImuPoseProblem.m_poseKeys[k], m_lshankImuAngVelKeys[k], m_lankleAxisFootKey, m_ankleHingeAxisNoiseModel);
            m_graph += bioslam::HingeJointConstraintVecErrEstAngVel(m_rshankImuPoseProblem.m_poseKeys[k], m_rshankImuAngVelKeys[k], m_rfootImuPoseProblem.m_poseKeys[k], m_rfootImuAngVelKeys[k], m_rankleAxisShankKey, m_ankleHingeAxisNoiseModel);
            m_graph += bioslam::HingeJointConstraintVecErrEstAngVel(m_lshankImuPoseProblem.m_poseKeys[k], m_lshankImuAngVelKeys[k], m_lfootImuPoseProblem.m_poseKeys[k], m_lfootImuAngVelKeys[k], m_lankleAxisShankKey, m_ankleHingeAxisNoiseModel);
        }
    }else if(m_hingeAxisFactorDim==1){ // use norm error model
        for(uint k=0; k<m_rshankImuPoseProblem.m_poseKeys.size(); k++){
            m_graph += bioslam::HingeJointConstraintNormErrEstAngVel(m_rfootImuPoseProblem.m_poseKeys[k], m_rfootImuAngVelKeys[k], m_rshankImuPoseProblem.m_poseKeys[k], m_rshankImuAngVelKeys[k], m_rankleAxisFootKey, m_ankleHingeAxisNoiseModel);
            m_graph += bioslam::HingeJointConstraintNormErrEstAngVel(m_lfootImuPoseProblem.m_poseKeys[k], m_lfootImuAngVelKeys[k], m_lshankImuPoseProblem.m_poseKeys[k], m_lshankImuAngVelKeys[k], m_lankleAxisFootKey, m_ankleHingeAxisNoiseModel);
            m_graph += bioslam::HingeJointConstraintNormErrEstAngVel(m_rshankImuPoseProblem.m_poseKeys[k], m_rshankImuAngVelKeys[k], m_rfootImuPoseProblem.m_poseKeys[k], m_rfootImuAngVelKeys[k], m_rankleAxisShankKey, m_ankleHingeAxisNoiseModel);
            m_graph += bioslam::HingeJointConstraintNormErrEstAngVel(m_lshankImuPoseProblem.m_poseKeys[k], m_lshankImuAngVelKeys[k], m_lfootImuPoseProblem.m_poseKeys[k], m_lfootImuAngVelKeys[k], m_lankleAxisShankKey, m_ankleHingeAxisNoiseModel);
        }
    }else{throw std::runtime_error("unknown factor dimension");}
    // add initial values for ankle hinge axes
    m_initialValues.insert(m_rankleAxisShankKey,priorAnkleAxisRShank);
    m_initialValues.insert(m_lankleAxisShankKey,priorAnkleAxisLShank);
    m_initialValues.insert(m_rankleAxisFootKey,priorAnkleAxisRFoot);
    m_initialValues.insert(m_lankleAxisFootKey,priorAnkleAxisLFoot);
    // if requested, also add priors
    if(m_usePriorsOnAnkleHingeAxes){ // also add priors
        m_graph.push_back(gtsam::PriorFactor<gtsam::Unit3>(m_rankleAxisShankKey,priorAnkleAxisRShank,gtsam::noiseModel::Diagonal::Sigmas(rankleHingeAxisShankPriorStd, true)));
        m_graph.push_back(gtsam::PriorFactor<gtsam::Unit3>(m_lankleAxisShankKey,priorAnkleAxisLShank,gtsam::noiseModel::Diagonal::Sigmas(lankleHingeAxisShankPriorStd, true)));
        m_graph.push_back(gtsam::PriorFactor<gtsam::Unit3>(m_rankleAxisFootKey,priorAnkleAxisRFoot,gtsam::noiseModel::Diagonal::Sigmas(rankleHingeAxisFootPriorStd, true)));
        m_graph.push_back(gtsam::PriorFactor<gtsam::Unit3>(m_lankleAxisFootKey,priorAnkleAxisLFoot,gtsam::noiseModel::Diagonal::Sigmas(lankleHingeAxisFootPriorStd, true)));
    }
}

void lowerBodyPoseEstimator::setNoiseModelsFromMemberNoiseVariables() {
    // a function which sets all of the member noise models from member noise variables (typically vectors or doubles)
    // hinge axis noise models
    if(m_hingeAxisFactorDim==3){ // vector error models, noise should be 3-vector
        m_kneeHingeAxisNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(m_kneeHingeAxisNoise, true);
        m_hipHingeAxisNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(m_hipHingeAxisNoise, true);
        m_ankleHingeAxisNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(m_ankleHingeAxisNoise, true);
    }else if(m_hingeAxisFactorDim==1){ // norm error, noise should be a 3-vector. use the norm of the 3-vector noise
        m_kneeHingeAxisNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(m_kneeHingeAxisNoise.norm()), true);
        m_hipHingeAxisNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(m_hipHingeAxisNoise.norm()), true);
        m_ankleHingeAxisNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(m_ankleHingeAxisNoise.norm()), true);
    }else{throw std::runtime_error("unknown factor dimensionality. use 1 or 3.");}
    // shared position centers of joints contraint
    if(m_jointPosFactorDim == 3){ // vector error models, noise should be 3-vector
        m_hipJointCtrConnectionNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(m_hipJointCtrConnectionNoise, true);
        m_kneeJointCtrConnectionNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(m_kneeJointCtrConnectionNoise, true);
        m_ankleJointCtrConnectionNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(m_ankleJointCtrConnectionNoise, true);
    }else if(m_jointPosFactorDim == 1){ // norm error, noise should be a 3-vector. use the norm of the 3-vector noise
        m_hipJointCtrConnectionNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(m_hipJointCtrConnectionNoise.norm()), true);
        m_kneeJointCtrConnectionNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(m_kneeJointCtrConnectionNoise.norm()), true);
        m_ankleJointCtrConnectionNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(m_ankleJointCtrConnectionNoise.norm()), true);
    }else{throw std::runtime_error("unknown factor dimensionality. use 1 or 3.");}
    // constrained joint velocity
    if(m_jointVelFactorDim == 3){ // vector error models, noise should be 3-vector
        m_jointCtrVelConnectionNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(m_jointCtrVelConnectionNoise,true);
    }else if(m_jointVelFactorDim == 1){ // norm error, noise should be a 3-vector. use the norm of the 3-vector noise
        m_jointCtrVelConnectionNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(m_jointCtrVelConnectionNoise.norm()),true);
    }else{throw std::runtime_error("unknown factor dimensionality. use 1 or 3.");}
    // others
    m_pelvicWidthNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(m_pelvicWidthSigma),true);
    m_rfemurLengthNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(m_rfemurLengthSigma),true);
    m_rtibiaLengthNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(m_rtibiaLengthSigma),true);
    m_lfemurLengthNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(m_lfemurLengthSigma),true);
    m_ltibiaLengthNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(m_ltibiaLengthSigma),true);
    m_angVelNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(m_angVelNoise,true);
    m_segmentLengthMaxNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(m_segmentLengthMaxNoise),true);
    m_segmentLengthMinNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(m_segmentLengthMinNoise),true);
    m_femurLengthDiscrepancyNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(m_femurLengthDiscrepancyNoise),true);
    m_tibiaLengthDiscrepancyNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(m_tibiaLengthDiscrepancyNoise),true);
    m_angBwKneeAxisAndSegmentFemurNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(m_angBwKneeAxisAndSegmentStdFemur), true);
    m_angBwKneeAxisAndSegmentTibiaNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(m_angBwKneeAxisAndSegmentStdTibia), true);
}

void lowerBodyPoseEstimator::printNoiseModelInfo(){
    // prints relevant information about constituent noise models
    std::cout<<"--- noise model information ---"<<std::endl;
    m_kneeHingeAxisNoiseModel->print("kneeHingeNoiseModel (rad/s): ");
    m_hipHingeAxisNoiseModel->print("hipHingeNoiseModel (rad/s): ");
    m_ankleHingeAxisNoiseModel->print("ankleHingeNoiseModel (rad/s): ");
    m_hipJointCtrConnectionNoiseModel->print("hipJointCtrConnectionNoiseModel (m): ");
    m_kneeJointCtrConnectionNoiseModel->print("kneeJointCtrConnectionNoiseModel (m): ");
    m_ankleJointCtrConnectionNoiseModel->print("ankleJointCtrConnectionNoiseModel (m): ");
    m_jointCtrVelConnectionNoiseModel->print("jointCtrVelConnectionNoiseModel (m/s): ");
    m_pelvicWidthNoiseModel->print("pelvicWidthNoiseModel (m): ");
    m_rfemurLengthNoiseModel->print("RFemurLengthNoiseModel (m): ");
    m_rtibiaLengthNoiseModel->print("RTibiaLengthNoiseModel (m): ");
    m_lfemurLengthNoiseModel->print("LFemurLengthNoiseModel (m): ");
    m_ltibiaLengthNoiseModel->print("LTibiaLengthNoiseModel (m): ");
    m_angVelNoiseModel->print("angVelNoiseModel (rad/s): ");
    m_segmentLengthMaxNoiseModel->print("SegmentLengthMaxNoiseModel (m): ");
    m_segmentLengthMinNoiseModel->print("SegmentLengthMinNoiseModel (m): ");
    m_femurLengthDiscrepancyNoiseModel->print("FemurLengthDiscrepancyNoiseModel (m): ");
    m_tibiaLengthDiscrepancyNoiseModel->print("TibiaLengthDiscrepancyNoiseModel (m): ");
    m_angBwKneeAxisAndSegmentFemurNoiseModel->print("angBwKneeAxisAndTibiaProxNoiseModel (rad): ");
    m_angBwKneeAxisAndSegmentFemurNoiseModel->print("angBwKneeAxisAndSegmentFemurNoiseModel (rad): ");
    m_angBwKneeAxisAndSegmentTibiaNoiseModel->print("angBwKneeAxisAndSegmentTibiaNoiseModel(rad): ");
    std::cout<<"-------------------------------"<<std::endl;
}

void lowerBodyPoseEstimator::setup(){
    std::cout<<"setting up LowerBodyPoseEstimator ('"<<m_strId<<"')"<<std::endl;
    double setupTic=clock();
    m_time=m_rthighImuPoseProblem.m_time; // pull up time
    // () first: setup noise models
    setNoiseModelsFromMemberNoiseVariables();
    // print noise model info
    printNoiseModelInfo();
    // add angular velocity estimation to problem
    addInstantaneousAngVelEstimationToImuPoses();
    // setup keys for static variables
    m_rkneeAxisThighKey = gtsam::Symbol(m_rkneeAxisThighVarChar,0), m_rkneeAxisShankKey = gtsam::Symbol(m_rkneeAxisShankVarChar,0);
    m_sacrumImuToRHipCtrKey=gtsam::Symbol(m_sacrumImuToRHipCtrVarChar,0), m_rthighImuToHipCtrKey=gtsam::Symbol(m_rthighImuToHipCtrVarChar,0), m_rthighImuToKneeCtrKey=gtsam::Symbol(m_rthighImuToKneeCtrVarChar,0);
    m_rshankImuToKneeCtrKey=gtsam::Symbol(m_rshankImuToKneeCtrVarChar,0), m_rshankImuToAnkleCtrKey=gtsam::Symbol(m_rshankImuToAnkleCtrVarChar,0), m_rfootImuToAnkleCtrKey=gtsam::Symbol(m_rfootImuToAnkleCtrVarChar,0);
    m_lkneeAxisThighKey = gtsam::Symbol(m_lkneeAxisThighVarChar,0), m_lkneeAxisShankKey = gtsam::Symbol(m_lkneeAxisShankVarChar,0);
    m_sacrumImuToLHipCtrKey=gtsam::Symbol(m_sacrumImuToLHipCtrVarChar,0), m_lthighImuToHipCtrKey=gtsam::Symbol(m_lthighImuToHipCtrVarChar,0), m_lthighImuToKneeCtrKey=gtsam::Symbol(m_lthighImuToKneeCtrVarChar,0);
    m_lshankImuToKneeCtrKey=gtsam::Symbol(m_lshankImuToKneeCtrVarChar,0), m_lshankImuToAnkleCtrKey=gtsam::Symbol(m_lshankImuToAnkleCtrVarChar,0), m_lfootImuToAnkleCtrKey=gtsam::Symbol(m_lfootImuToAnkleCtrVarChar,0);
    // add knee axis initial values and optional priors
    //std::cout<<"initialized knee axes as: RThigh=["<<priorAxisRThigh.point3().transpose()<<"], RShank .... "<<std::endl;
    m_initialValues.insert(m_rkneeAxisThighKey, priorKneeAxisRThigh); m_initialValues.insert(m_rkneeAxisShankKey, priorKneeAxisRShank); m_initialValues.insert(m_lkneeAxisThighKey, priorKneeAxisLThigh); m_initialValues.insert(m_lkneeAxisShankKey, priorKneeAxisLShank);
    if(m_usePriorsOnKneeHingeAxes){
        m_graph.push_back(gtsam::PriorFactor<gtsam::Unit3>(m_rkneeAxisThighKey, priorKneeAxisRThigh, gtsam::noiseModel::Diagonal::Sigmas(rkneeHingeAxisThighPriorStd, true)));
        m_graph.push_back(gtsam::PriorFactor<gtsam::Unit3>(m_rkneeAxisShankKey, priorKneeAxisRShank, gtsam::noiseModel::Diagonal::Sigmas(rkneeHingeAxisShankPriorStd, true)));
        m_graph.push_back(gtsam::PriorFactor<gtsam::Unit3>(m_lkneeAxisThighKey, priorKneeAxisLThigh, gtsam::noiseModel::Diagonal::Sigmas(lkneeHingeAxisThighPriorStd, true)));
        m_graph.push_back(gtsam::PriorFactor<gtsam::Unit3>(m_lkneeAxisShankKey, priorKneeAxisLShank, gtsam::noiseModel::Diagonal::Sigmas(lkneeHingeAxisShankPriorStd, true)));
    }
    // set initial values for static vectors
    m_initialValues.insert(m_sacrumImuToRHipCtrKey,priorSacrumImuToRHipCtr); // insert initial values
    m_initialValues.insert(m_sacrumImuToLHipCtrKey,priorSacrumImuToLHipCtr);
    m_initialValues.insert(m_rthighImuToKneeCtrKey,priorRThighImuToKneeCtr);
    m_initialValues.insert(m_rthighImuToHipCtrKey,priorRThighImuToHipCtr);
    m_initialValues.insert(m_rshankImuToKneeCtrKey,priorRShankImuToKneeCtr);
    m_initialValues.insert(m_rshankImuToAnkleCtrKey,priorRShankImuToAnkleCtr);
    m_initialValues.insert(m_rfootImuToAnkleCtrKey,priorRFootImuToAnkleCtr);
    m_initialValues.insert(m_lthighImuToKneeCtrKey,priorLThighImuToKneeCtr);
    m_initialValues.insert(m_lthighImuToHipCtrKey,priorLThighImuToHipCtr);
    m_initialValues.insert(m_lshankImuToKneeCtrKey,priorLShankImuToKneeCtr);
    m_initialValues.insert(m_lshankImuToAnkleCtrKey,priorLShankImuToAnkleCtr);
    m_initialValues.insert(m_lfootImuToAnkleCtrKey,priorLFootImuToAnkleCtr);
    if(m_usePriorsOnImuToJointCtrVectors){ // add priors
        gtsam::SharedNoiseModel imuToJointCtrVecPriorNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(imuToJointCtrPriorStd, true);
        m_graph.push_back(gtsam::PriorFactor<gtsam::Point3>(m_sacrumImuToRHipCtrKey,priorSacrumImuToRHipCtr,imuToJointCtrVecPriorNoiseModel));
        m_graph.push_back(gtsam::PriorFactor<gtsam::Point3>(m_sacrumImuToLHipCtrKey,priorSacrumImuToLHipCtr,imuToJointCtrVecPriorNoiseModel));
        m_graph.push_back(gtsam::PriorFactor<gtsam::Point3>(m_rthighImuToHipCtrKey,priorRThighImuToHipCtr,imuToJointCtrVecPriorNoiseModel));
        m_graph.push_back(gtsam::PriorFactor<gtsam::Point3>(m_rthighImuToKneeCtrKey,priorRThighImuToKneeCtr,imuToJointCtrVecPriorNoiseModel));
        m_graph.push_back(gtsam::PriorFactor<gtsam::Point3>(m_rshankImuToKneeCtrKey,priorRShankImuToKneeCtr,imuToJointCtrVecPriorNoiseModel));
        m_graph.push_back(gtsam::PriorFactor<gtsam::Point3>(m_rshankImuToAnkleCtrKey,priorRShankImuToAnkleCtr,imuToJointCtrVecPriorNoiseModel));
        m_graph.push_back(gtsam::PriorFactor<gtsam::Point3>(m_rfootImuToAnkleCtrKey,priorRFootImuToAnkleCtr,imuToJointCtrVecPriorNoiseModel));
        m_graph.push_back(gtsam::PriorFactor<gtsam::Point3>(m_lthighImuToHipCtrKey,priorLThighImuToHipCtr,imuToJointCtrVecPriorNoiseModel));
        m_graph.push_back(gtsam::PriorFactor<gtsam::Point3>(m_lthighImuToKneeCtrKey,priorLThighImuToKneeCtr,imuToJointCtrVecPriorNoiseModel));
        m_graph.push_back(gtsam::PriorFactor<gtsam::Point3>(m_lshankImuToKneeCtrKey,priorLShankImuToKneeCtr,imuToJointCtrVecPriorNoiseModel));
        m_graph.push_back(gtsam::PriorFactor<gtsam::Point3>(m_lshankImuToAnkleCtrKey,priorLShankImuToAnkleCtr,imuToJointCtrVecPriorNoiseModel));
        m_graph.push_back(gtsam::PriorFactor<gtsam::Point3>(m_lfootImuToAnkleCtrKey,priorLFootImuToAnkleCtr,imuToJointCtrVecPriorNoiseModel));
    }
    // pull out estimated values from imu pose problems and add them here
    addImuProblemSolutionsToInitialValues();
    // () add the (disparate) IMU graphs to this graph
    addImuProblemGraphsToGraph();
    // () now it's time to weave the connections between these dispartite graphs
    uint nKeyframes=m_rthighImuPoseProblem.m_poseKeys.size();
    // add all hinge constraints to graph
    addKneeHingeConstraints();
    if(m_assumeHipHinge){addHipHingeConstraints();}
    if(m_assumeAnkleHinge){addAnkleHingeConstraints();}
    // add constrained joint centers of rotation
    add6JointSharedPosConstraints();
    // optional: add joint velocity factor
    if(m_useJointVelFactor){ add6JointSharedVelConstraints(); }
    // anthropometric constraints
    if(m_usePelvicWidthFactor){
        m_graph += bioslam::SegmentLengthMagnitudeFactor(m_sacrumImuToRHipCtrKey,m_sacrumImuToLHipCtrKey,m_pelvicWidth,m_pelvicWidthNoiseModel);
        if(m_useMaxAnthroConstraint){
            // add 99th percentile male hip breath which is 40.9cm. source: ANSUR II http://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf
            m_graph.push_back(bioslam::SegmentLengthMaxMagnitudeFactor(m_sacrumImuToLHipCtrKey,m_sacrumImuToRHipCtrKey,0.409,m_segmentLengthMaxNoiseModel));
        }
        if(m_useMinAnthroConstraint){
            // todo: get an actual value for this! just gonna use like 10cm for right now. No one should have a smaller femoral head separation than that.
            m_graph.push_back(bioslam::SegmentLengthMinMagnitudeFactor(m_sacrumImuToLHipCtrKey,m_sacrumImuToRHipCtrKey,0.10,m_segmentLengthMinNoiseModel));
        }
    }
    if(m_useRFemurLengthFactor){
        m_graph += bioslam::SegmentLengthMagnitudeFactor(m_rthighImuToHipCtrKey,m_rthighImuToKneeCtrKey,m_rfemurLength,m_rfemurLengthNoiseModel);
        if(m_useMaxAnthroConstraint) {
            // add 99th percentile male femur length which is 48cm. source: ANSUR II D29 Thigh Link http://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf
            m_graph.push_back(bioslam::SegmentLengthMaxMagnitudeFactor(m_rthighImuToHipCtrKey, m_rthighImuToKneeCtrKey, 0.48, m_segmentLengthMaxNoiseModel));
        }
        if(m_useMinAnthroConstraint){
            // add 1st percentile female femur length which is 32.6cm. source: ANSUR II D29 Thigh Link http://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf
            m_graph.push_back(bioslam::SegmentLengthMinMagnitudeFactor(m_rthighImuToHipCtrKey,m_rthighImuToKneeCtrKey,0.326,m_segmentLengthMinNoiseModel));
        }
    }
    if(m_useRTibiaLengthFactor){
        m_graph += bioslam::SegmentLengthMagnitudeFactor(m_rshankImuToKneeCtrKey,m_rshankImuToAnkleCtrKey,m_rtibiaLength,m_rtibiaLengthNoiseModel);
        if(m_useMaxAnthroConstraint){
           // add 99th percentile male tibia length which is 47.90cm. source: ANSUR II D6 Calf Link http://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf
            m_graph.push_back(bioslam::SegmentLengthMaxMagnitudeFactor(m_rshankImuToKneeCtrKey,m_rshankImuToAnkleCtrKey,0.479,m_segmentLengthMaxNoiseModel));
        }
        if(m_useMinAnthroConstraint){
           // add 1st percentile female tibia length which is 34.4cm. source: ANSUR II D6 Calf Link http://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf
           m_graph.push_back(bioslam::SegmentLengthMinMagnitudeFactor(m_rshankImuToKneeCtrKey,m_rshankImuToAnkleCtrKey,0.344,m_segmentLengthMinNoiseModel));
        }
    }
    if(m_useLFemurLengthFactor){
        m_graph += bioslam::SegmentLengthMagnitudeFactor(m_lthighImuToHipCtrKey,m_lthighImuToKneeCtrKey,m_lfemurLength,m_lfemurLengthNoiseModel);
        if(m_useMaxAnthroConstraint) {
            // add 99th percentile male femur length which is 48cm. source: ANSUR II D29 Thigh Link http://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf
            m_graph.push_back(bioslam::SegmentLengthMaxMagnitudeFactor(m_lthighImuToHipCtrKey, m_lthighImuToKneeCtrKey, 0.48, m_segmentLengthMaxNoiseModel));
        }
        if(m_useMinAnthroConstraint){
            // add 1st percentile female femur length which is 32.6cm. source: ANSUR II D29 Thigh Link http://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf
            m_graph.push_back(bioslam::SegmentLengthMinMagnitudeFactor(m_lthighImuToHipCtrKey,m_lthighImuToKneeCtrKey,0.326,m_segmentLengthMinNoiseModel));
        }
    }
    if(m_useLTibiaLengthFactor){
        m_graph += bioslam::SegmentLengthMagnitudeFactor(m_lshankImuToKneeCtrKey,m_lshankImuToAnkleCtrKey,m_ltibiaLength,m_ltibiaLengthNoiseModel);
        if(m_useMaxAnthroConstraint){
            // add 99th percentile male tibia length which is 47.90cm. source: ANSUR II D6 Calf Link http://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf
            m_graph.push_back(bioslam::SegmentLengthMaxMagnitudeFactor(m_lshankImuToKneeCtrKey,m_lshankImuToAnkleCtrKey,0.479,m_segmentLengthMaxNoiseModel));
        }
        if(m_useMinAnthroConstraint){
            // add 1st percentile female tibia length which is 34.4cm. source: ANSUR II D6 Calf Link http://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf
            m_graph.push_back(bioslam::SegmentLengthMinMagnitudeFactor(m_lshankImuToKneeCtrKey,m_lshankImuToAnkleCtrKey,0.344,m_segmentLengthMinNoiseModel));
        }
    }
    // () optional: constraint femur length discrepancy, i.e., left/right discrepancy of subject femur length
    if(m_useFemurLengthDiscrepancyFactor){
        m_graph.push_back(bioslam::SegmentLengthDiscrepancyFactor(m_rthighImuToHipCtrKey,m_rthighImuToKneeCtrKey,m_lthighImuToHipCtrKey,m_lthighImuToKneeCtrKey,m_femurLengthDiscrepancyNoiseModel));
    }
    // () optional: constraint tibia length discrepancy, i.e., left/right discrepancy of subject tibia length
    if(m_useTibiaLengthDiscrepancyFactor){
        m_graph.push_back(bioslam::SegmentLengthDiscrepancyFactor(m_rshankImuToKneeCtrKey,m_rshankImuToAnkleCtrKey,m_lshankImuToKneeCtrKey,m_lshankImuToAnkleCtrKey,m_tibiaLengthDiscrepancyNoiseModel));
    }
    // () optional: constrain angle between between tibia segment and knee axis as expressed in the shank IMU frame
    if(m_useKneeAxisTibiaOrthogonalityFactor){
       // remember that the axis should point right! So setup these angles assuming the knee axis points right.
       m_graph.push_back(bioslam::AngleBetweenAxisAndSegmentFactor(m_rkneeAxisShankKey, m_rshankImuToKneeCtrKey, m_rshankImuToAnkleCtrKey, m_angBwKneeAxisAndSegmentMeanRTibia, m_angBwKneeAxisAndSegmentTibiaNoiseModel)); // right tibia
       m_graph.push_back(bioslam::AngleBetweenAxisAndSegmentFactor(m_lkneeAxisShankKey, m_lshankImuToKneeCtrKey, m_lshankImuToAnkleCtrKey, m_angBwKneeAxisAndSegmentMeanLTibia, m_angBwKneeAxisAndSegmentTibiaNoiseModel)); // left tibia
    }
    // () optional: constrain angle between between femur segment and knee axis as expressed in the thigh IMU frame
    if(m_useKneeAxisFemurOrthogonalityFactor){
       // remember that the axis should point right! So setup these angles assuming the knee axis points right.
        m_graph.push_back(bioslam::AngleBetweenAxisAndSegmentFactor(m_rkneeAxisThighKey, m_rthighImuToHipCtrKey, m_rthighImuToKneeCtrKey, m_angBwKneeAxisAndSegmentMeanRFemur, m_angBwKneeAxisAndSegmentFemurNoiseModel)); // right femur
        m_graph.push_back(bioslam::AngleBetweenAxisAndSegmentFactor(m_lkneeAxisThighKey, m_lthighImuToHipCtrKey, m_lthighImuToKneeCtrKey, m_angBwKneeAxisAndSegmentMeanLFemur, m_angBwKneeAxisAndSegmentFemurNoiseModel)); // left femur
    }
    // ^--- the graph is now setup!
    // apply initialization scheme
    if(m_initializationScheme==0){ // do nothing, accept whatever values got passed in by the imuPoseEstimators
        std::cout<<"lowerBodyPoseEstimator initialization scheme: none"<<std::endl; // (same as setting zero to IMU pose trajectories)
    }else if(m_initializationScheme==1){ // replace all yaw angles with the sacrum imu's yaw angles
        std::cout<<"lowerBodyPoseEstimator initialization scheme: replace all yaw angles in pose graph with sacrum IMU's yaw angles"<<std::endl;
        alignYawAnglesOfAllImusToSacrumImu(m_initialValues); // in m_initialValues, this function edits the values in place to align all yaw angles to the sacrum IMU
    }else if(m_initializationScheme==2){
        std::cout<<"lowerBodyPoseEstimator initialization scheme: searching for reasonable a priori joint angles via editing yaw"<<std::endl;
        searchYawAnglesForInboundsJointAngles(m_initialValues);
    }else if(m_initializationScheme==3) { // debug with internal precession angles
    }else if(m_initializationScheme==4){ // init imu values via (a) adjust distal imu orientation at each joint to make joint angles in bounds and (b) straighten positions to make joint centers consistent
        std::cout<<"initializing graph with orientations based on joint angle limits and positions based on consistent static vectors from IMUs to neighboring joint centers";
        double priorErr=m_graph.error(m_initialValues);
        setImuOrientationsBasedOnJointAngleLimits(m_initialValues,false,false,false);
        setImuPositionsBasedOnConsistentInitialStaticVecsToJointCtrs(m_initialValues);
        setAllImuVelocitiesAsImuPositionDerivatives(m_initialValues);
        double newErr=m_graph.error(m_initialValues);
        std::cout<<", prior error="<<priorErr<<", new error="<<newErr<<std::endl;
    }else if(m_initializationScheme==5){ // initialize from file
        if(!m_initializationFile.empty()){
            setValuesFromFile(m_initialValues, m_initializationFile);
        }else{
            std::cerr<<"you requested to initialize from a file, but the file was not set!"<<std::endl;
        }
        // also need to update prior on sacrum IMU pose from zero position to whatever this vicon frame measurement was
        gtsamutils::clearGtsamFactorByKey<gtsam::PriorFactor<gtsam::Pose3>>(m_graph,m_sacrumImuPoseProblem.m_poseKeys[0]);
        gtsamutils::clearGtsamFactorByKey<bioslam::Pose3TranslationPrior>(m_graph,m_sacrumImuPoseProblem.m_poseKeys[0]);
        gtsam::Vector6 std=gtsam::Vector6::Ones()*1.0e-3;
        m_graph+=gtsam::PriorFactor<gtsam::Pose3>(m_sacrumImuPoseProblem.m_poseKeys[0],m_initialValues.at<gtsam::Pose3>(m_sacrumImuPoseProblem.m_poseKeys[0]),gtsam::noiseModel::Diagonal::Sigmas(std,true));
    }else{
        std::cerr<<"unknown initialization scheme"<<std::endl;
    }
    std::cout<<"    setup complete! ("<<(clock()-setupTic)/CLOCKS_PER_SEC<<" sec)"<<std::endl;
}

void lowerBodyPoseEstimator::addHipHingeAxisVariableKeys(){
    // creates new variables, one for each hip axis variable key
    // be default will only create single variable: the hip axis in the sacrum frame. change input argument to setup thigh keys.
    // () setup hipAxisSacrum
    m_hipAxisSacrumVarStr = m_strId+"_hipAxisSacrum";
    m_hipAxisSacrumVarChar=VarStrToCharMap::insert(m_hipAxisSacrumVarStr);
    m_hipAxisSacrumKey = gtsam::Symbol(m_hipAxisSacrumVarChar,0);
    // () also setup hipAxisThigh keys
    m_rhipAxisThighVarStr = m_strId+"_rhipAxisThigh";
    m_rhipAxisThighVarChar=VarStrToCharMap::insert(m_rhipAxisThighVarStr);
    m_rhipAxisThighKey = gtsam::Symbol(m_rhipAxisThighVarChar,0);
    m_lhipAxisThighVarStr = m_strId+"_lhipAxisThigh";
    m_lhipAxisThighVarChar=VarStrToCharMap::insert(m_lhipAxisThighVarStr);
    m_lhipAxisThighKey = gtsam::Symbol(m_lhipAxisThighVarChar,0);
}

void lowerBodyPoseEstimator::addAnkleHingeAxisVariableKeys(){
    // creates four new variables, one for each ankle axis variable key
    // be default will only create two variables: the ankle axis in the foot frames. change input argument to setup shank keys.
    m_rankleAxisFootVarStr = m_strId+"_rankleAxisFoot"; // set var str
    m_rankleAxisFootVarChar=VarStrToCharMap::insert(m_rankleAxisFootVarStr); // set var char
    m_rankleAxisFootKey = gtsam::Symbol(m_rankleAxisFootVarChar,0); // set key value
    m_lankleAxisFootVarStr = m_strId+"_lankleAxisFoot"; // set var str
    m_lankleAxisFootVarChar=VarStrToCharMap::insert(m_lankleAxisFootVarStr); // set var char
    m_lankleAxisFootKey = gtsam::Symbol(m_lankleAxisFootVarChar,0); // set key value
    // () also setup ankleAxisShank keys
    m_rankleAxisShankVarStr = m_strId+"_rankleAxisShank"; // set var str
    m_rankleAxisShankVarChar=VarStrToCharMap::insert(m_rankleAxisShankVarStr); // set var char
    m_rankleAxisShankKey = gtsam::Symbol(m_rankleAxisShankVarChar,0); // set key value
    m_lankleAxisShankVarStr = m_strId+"_lankleAxisShank"; // set var str
    m_lankleAxisShankVarChar=VarStrToCharMap::insert(m_lankleAxisShankVarStr); // set var char
    m_lankleAxisShankKey = gtsam::Symbol(m_lankleAxisShankVarChar,0); // set key value
}

// constructor
lowerBodyPoseEstimator::lowerBodyPoseEstimator(const imuPoseEstimator& sacrumImuPoseProblem, const imuPoseEstimator& rthighImuPoseProblem, const imuPoseEstimator& rshankImuPoseProblem, const imuPoseEstimator& rfootImuPoseProblem,
                                               const imuPoseEstimator& lthighImuPoseProblem, const imuPoseEstimator& lshankImuPoseProblem, const imuPoseEstimator& lfootImuPoseProblem, const std::string& id){
    // set IMU pose problems
    m_sacrumImuPoseProblem=sacrumImuPoseProblem, m_rthighImuPoseProblem=rthighImuPoseProblem, m_rshankImuPoseProblem=rshankImuPoseProblem, m_rfootImuPoseProblem=rfootImuPoseProblem, m_lthighImuPoseProblem=lthighImuPoseProblem, m_lshankImuPoseProblem=lshankImuPoseProblem, m_lfootImuPoseProblem=lfootImuPoseProblem;
    // set variable strings and chars
    m_strId=id;
    // sacrum IMU
    m_sacrumImuToRHipCtrVarStr = m_strId+"_sacrumImuToRHipCtr";
    VarStrToCharMap::insert(m_sacrumImuToRHipCtrVarStr);
    m_sacrumImuToRHipCtrVarChar = VarStrToCharMap::getChar(m_sacrumImuToRHipCtrVarStr);
    m_sacrumImuToLHipCtrVarStr = m_strId+"_sacrumImuToLHipCtr";
    VarStrToCharMap::insert(m_sacrumImuToLHipCtrVarStr);
    m_sacrumImuToLHipCtrVarChar = VarStrToCharMap::getChar(m_sacrumImuToLHipCtrVarStr);
    // right thigh IMU
    m_rkneeAxisThighVarStr=m_strId+"_rkneeAxisThigh";
    VarStrToCharMap::insert(m_rkneeAxisThighVarStr);
    m_rkneeAxisThighVarChar=VarStrToCharMap::getChar(m_rkneeAxisThighVarStr);
    m_rthighImuToKneeCtrVarStr = m_strId+"_rthighImuToKneeCtr";
    VarStrToCharMap::insert(m_rthighImuToKneeCtrVarStr);
    m_rthighImuToKneeCtrVarChar = VarStrToCharMap::getChar(m_rthighImuToKneeCtrVarStr);
    m_rthighImuToHipCtrVarStr = m_strId+"_rthighImuToHipCtr";
    VarStrToCharMap::insert(m_rthighImuToHipCtrVarStr);
    m_rthighImuToHipCtrVarChar = VarStrToCharMap::getChar(m_rthighImuToHipCtrVarStr);
    // right shank IMU
    m_rkneeAxisShankVarStr=m_strId+"_rkneeAxisShank";
    VarStrToCharMap::insert(m_rkneeAxisShankVarStr);
    m_rkneeAxisShankVarChar=VarStrToCharMap::getChar(m_rkneeAxisShankVarStr);
    m_rshankImuToKneeCtrVarStr = m_strId+"_rshankImuToKneeCtr";
    VarStrToCharMap::insert(m_rshankImuToKneeCtrVarStr);
    m_rshankImuToKneeCtrVarChar = VarStrToCharMap::getChar(m_rshankImuToKneeCtrVarStr);
    m_rshankImuToAnkleCtrVarStr = m_strId+"_rshankImuToAnkleCtr";
    VarStrToCharMap::insert(m_rshankImuToAnkleCtrVarStr);
    m_rshankImuToAnkleCtrVarChar = VarStrToCharMap::getChar(m_rshankImuToAnkleCtrVarStr);
    // right foot IMU
    m_rfootImuToAnkleCtrVarStr = m_strId+"_rfootImuToAnkleCtr";
    VarStrToCharMap::insert(m_rfootImuToAnkleCtrVarStr);
    m_rfootImuToAnkleCtrVarChar = VarStrToCharMap::getChar(m_rfootImuToAnkleCtrVarStr);
    // left thigh IMU
    m_lkneeAxisThighVarStr=m_strId+"_lkneeAxisThigh";
    VarStrToCharMap::insert(m_lkneeAxisThighVarStr);
    m_lkneeAxisThighVarChar=VarStrToCharMap::getChar(m_lkneeAxisThighVarStr);
    m_lthighImuToKneeCtrVarStr = m_strId+"_lthighImuToKneeCtr";
    VarStrToCharMap::insert(m_lthighImuToKneeCtrVarStr);
    m_lthighImuToKneeCtrVarChar = VarStrToCharMap::getChar(m_lthighImuToKneeCtrVarStr);
    m_lthighImuToHipCtrVarStr = m_strId+"_lthighImuToHipCtr";
    VarStrToCharMap::insert(m_lthighImuToHipCtrVarStr);
    m_lthighImuToHipCtrVarChar = VarStrToCharMap::getChar(m_lthighImuToHipCtrVarStr);
    // left shank IMU
    m_lkneeAxisShankVarStr=m_strId+"_lkneeAxisShank";
    VarStrToCharMap::insert(m_lkneeAxisShankVarStr);
    m_lkneeAxisShankVarChar=VarStrToCharMap::getChar(m_lkneeAxisShankVarStr);
    m_lshankImuToKneeCtrVarStr = m_strId+"_lshankImuToKneeCtr";
    VarStrToCharMap::insert(m_lshankImuToKneeCtrVarStr);
    m_lshankImuToKneeCtrVarChar = VarStrToCharMap::getChar(m_lshankImuToKneeCtrVarStr);
    m_lshankImuToAnkleCtrVarStr = m_strId+"_lshankImuToAnkleCtr";
    VarStrToCharMap::insert(m_lshankImuToAnkleCtrVarStr);
    m_lshankImuToAnkleCtrVarChar = VarStrToCharMap::getChar(m_lshankImuToAnkleCtrVarStr);
    // left foot IMU
    m_lfootImuToAnkleCtrVarStr = m_strId+"_lfootImuToAnkleCtr";
    VarStrToCharMap::insert(m_lfootImuToAnkleCtrVarStr);
    m_lfootImuToAnkleCtrVarChar = VarStrToCharMap::getChar(m_lfootImuToAnkleCtrVarStr);
}

void lowerBodyPoseEstimator::fastOptimize(double prevGlobalErr){
    // use the iterate() method and print status in between
    std::cout<<std::endl<<"*** lowerBodyPoseEstimator::fastOptimize() results for "<<m_strId<<" ***"<<std::endl;
    std::cout<<"    convergence criteria: abs. error decrease < "<<m_absErrDecreaseLimit<<" OR rel. error decrease < "<<m_relErrDecreaseLimit*100<<"% OR max iterations = "<<m_maxIterations<<std::endl;
    // setup parameters for optimization
    gtsam::LevenbergMarquardtParams params; params.setVerbosityLM("SUMMARY"); params.setVerbosity("TERMINATION");
    params.setlambdaLowerBound(m_lambdaLowerBound); params.setlambdaUpperBound(m_lambdaUpperBound); params.setlambdaFactor(m_lambdaFactor); params.setlambdaInitial(m_lambdaInitial);
    params.setRelativeErrorTol(1.0e-20); params.setAbsoluteErrorTol(1.0e-20); params.setMaxIterations(m_maxIterations); //<-set abs and rel params to an extremely small number since you're manually gonna manage the convergence
    params.setLinearSolverType(m_linearSolverType);
    // create optimizer and go
    gtsam::LevenbergMarquardtOptimizer optimizer(m_graph,m_initialValues,params);
    //gtsam::GaussNewtonOptimizer optimizer(m_graph,m_initialValues);
    double currentError=optimizer.error(), previousError=9.0e9, absErrorDecrease=9.0e9, relErrorDecrease=9.0e9, timeBeforeIterations=clock();
    std::cout<<"    initial error = "<<optimizer.error()<<std::endl;
    gtsamutils::printErrorsInGraphByFactorType(m_graph, m_initialValues);
    uint consecSmallIter=0;
    m_optimizationTotalError.push_back(currentError); m_optimizationTotalTime.push_back(0.0);
    while(consecSmallIter<m_convergeAtConsecSmallIter && optimizer.iterations()<m_maxIterations) {
        double iterationStart=clock();
        // perform iteration and checks
        optimizer.iterate(); // should have updated things in here now
        previousError=currentError, currentError=optimizer.error(); absErrorDecrease=previousError-currentError; relErrorDecrease=absErrorDecrease/previousError; // update error metrics
        if(absErrorDecrease<m_absErrDecreaseLimit || relErrorDecrease<m_relErrDecreaseLimit || currentError<m_absErrLimit){ // count this as a "small" iteration
            consecSmallIter++;
        }else{consecSmallIter=0;} // else reset count to zero
        // print iteration results
        std::cout<<"  --> end iteration #"<<optimizer.iterations()-1<<" ("<<(clock()-iterationStart)/CLOCKS_PER_SEC<<" sec). current error: "<<currentError<<" (decrease: "<<absErrorDecrease<<" || "<<relErrorDecrease*100<<"%) | converged iteration "<<consecSmallIter<<"/"<<m_convergeAtConsecSmallIter<<std::endl;
        m_optimizationTotalError.push_back(currentError); m_optimizationTotalTime.push_back((clock()-timeBeforeIterations)/CLOCKS_PER_SEC);
    }
    // check for restart condition on hip I/E drift
    double rhipIeSlope, lhipIeSlope;
    computeHipIeRotAngleRegressionSlopesFromValues(optimizer.values(), rhipIeSlope, lhipIeSlope);
    bool doRestart = ((abs(rhipIeSlope) > m_hipIeAngDriftRestartThreshold || abs(lhipIeSlope) > m_hipIeAngDriftRestartThreshold) && optimizer.error()<prevGlobalErr*0.99 && m_nRestarts<m_maxNumRestarts); // condition for restart: either hip I/E is drifted AND current optimized error is at least 1% less than prevGlobalErr
    if(doRestart){ // you found a drifted hip I/E angle. restart optimization after fixing hip I/E angle.
        m_nRestarts++;
        uint restartNum=m_nRestarts;
        std::cout<<"+++ rerunning optimization after fixing hips (optimization restart #"<<restartNum<<") +++"<<std::endl;
        if(abs(rhipIeSlope)>m_hipIeAngDriftRestartThreshold){std::cout<<"\tRHip I/E slope ("<<rhipIeSlope<<") is greater than threshold for restart ("<<m_hipIeAngDriftRestartThreshold<<")."<<std::endl;}
        if(abs(lhipIeSlope)>m_hipIeAngDriftRestartThreshold){std::cout<<"\tLHip I/E slope ("<<lhipIeSlope<<") is greater than threshold for restart ("<<m_hipIeAngDriftRestartThreshold<<")."<<std::endl;}
        gtsam::Values vals = gtsam::Values(optimizer.values()); // copy out Values
        // DEBUG: print vals to file
        //setMemberEstimatedStatesFromValues(vals), setDerivedJointAnglesFromEstimatedStates(), saveResultsToH5File(boost::filesystem::current_path().append(m_strId+"_preRestartNum"+std::to_string(restartNum)+".h5").string());
        // perform heuristic
        adjustLegsForCorrectedHipIeAngles(vals,true);
        // DEBUG: print vals to file
        //setMemberEstimatedStatesFromValues(vals), setDerivedJointAnglesFromEstimatedStates(), saveResultsToH5File(boost::filesystem::current_path().append(m_strId+"_postRestartNum"+std::to_string(restartNum)+".h5").string());
        // set initial values to vals and reoptimize
        m_initialValues=vals; // set initial values before running next optimization
        fastOptimize(optimizer.error()); // re-optimize
        std::cout<<"+-- end optimization restart #"<<m_nRestarts<<" --+"<<std::endl;
    }else{ // not doing restart, or this is the final restart in the recursion
        std::cout<<" --- iterations complete --- "<<std::endl;
        std::cout<<"    final error = "<<optimizer.error()<<", total time = "<<(clock()-timeBeforeIterations)/CLOCKS_PER_SEC<<" sec)"<<std::endl;
        // print convergence condition
        std::cout<<"Convergence condition:"<<std::endl;
        if(relErrorDecrease<m_relErrDecreaseLimit){ std::cout<<"    CONVERGED: rel. error decrease < limit ("<<relErrorDecrease<<" < "<<m_relErrDecreaseLimit<<")"<<std::endl; }
        else if(absErrorDecrease<m_absErrDecreaseLimit){std::cout<<"    CONVERGED: abs. error decrease < limit ("<<absErrorDecrease<<" < "<<m_absErrDecreaseLimit<<")"<<std::endl;}
        else if(optimizer.iterations()==m_maxIterations){std::cout<<"    exiting. iteration maximum reached ("<<m_maxIterations<<")"<<std::endl;}
        else{std::cout<<"    no convergence criteria met."<<std::endl;}
        m_estimate=optimizer.values();
        gtsamutils::printErrorsInGraphByFactorType(m_graph, m_estimate);
        setMemberEstimatedStatesFromValues(m_estimate);
        postHocAxesCheck(true); // flip sign of axes to point right
        setDerivedJointAnglesFromEstimatedStates();
        printStaticStatesSummary();
        setAllInternalPrecessionAngles();
        //compareHeadingsOfVectorSetsOfOrientations(m_sacrumImuOrientation,m_rthighImuOrientation);
        printYawSlopesAndInterceptsFromMemberOrientations();
        //testImusCanBeSpunWithOnlyChangeToIntExtRotAngle(m_graph,m_estimate, true);
    }
} // fastOptimize()

bool lowerBodyPoseEstimator::postHocAxesCheck(bool verbose) {
    // this method *for both legs*, checks:
    // (1) that both knee axes are pointed in the same direction (b/w thigh and shank frames)
    //   - done by checking average angle between. should be less than 90. if not, flips shank axis.
    // (2) that both axes are pointed to the subject's right (ISB definition).
    //   - done by checking the range of the knee flexion/extension angle. should be between 20 deg and -180 deg.
    // --- settings --- //
    double maxFlexExLimRad=20.0*M_PI/180.0; // max limit in radians to compare to
    double minFlexExLimRad=-179.0*M_PI/180.0; // min limit in radians to compare to
    // -----------------
    bool anyChangesMade=false; // if changes made, this goes to true. it will rerun this method just to make sure no changes are made the second time around.
    // --- right leg first ---
    // () are both axes pointing the same direction in the world frame?
    gtsam::Vector3 rThighAxis=m_rkneeAxisThigh.point3(), rShankAxis=m_rkneeAxisShank.point3();
    if(verbose) {
        std::cout << "---postHocAxesCheck()---" << std::endl;
        std::cout << "    ***checking right leg first***" << std::endl;
        std::cout<<"    rknee axis, thigh frame: "<<m_rkneeAxisThigh.point3().transpose()<<std::endl;
        std::cout<<"    rknee axis, shank frame: "<<m_rkneeAxisShank.point3().transpose()<<std::endl;
    }
    // get axes in world frame
    Eigen::MatrixXd rThighAxisNav(m_rthighImuOrientation.size(),3), rShankAxisNav(m_rshankImuOrientation.size(),3);
    std::vector<double> angBwRightAxesNavDeg(m_rthighImuOrientation.size());
    for(size_t k=0; k<m_rthighImuOrientation.size();k++) {
        rThighAxisNav.block<1,3>(k, 0)=(m_rthighImuOrientation[k].matrix() * rThighAxis).transpose();
        rShankAxisNav.block<1,3>(k, 0)=(m_rshankImuOrientation[k].matrix() * rShankAxis).transpose();
        angBwRightAxesNavDeg[k]= mathutils::unsignedAngle(rThighAxisNav.block<1,3>(k, 0), rShankAxisNav.block<1,3>(k, 0)) * 180.0 / M_PI; // put in degrees
    }
    if(std::accumulate(angBwRightAxesNavDeg.begin(),angBwRightAxesNavDeg.end(),0.0)/angBwRightAxesNavDeg.size() <= 90.0){ // same side, stay
        if(verbose){std::cout<<"mean of angle b/w: "<<std::accumulate(angBwRightAxesNavDeg.begin(),angBwRightAxesNavDeg.end(),0.0)/angBwRightAxesNavDeg.size()<<" deg < 90 deg, therefore I think they're pointing in the same direction"<<std::endl;}
    }else{
        m_rkneeAxisShank=gtsam::Unit3(-1.0*m_rkneeAxisShank.point3()); anyChangesMade=true;
        if(verbose){std::cout<<"mean of angle b/w: "<<std::accumulate(angBwRightAxesNavDeg.begin(),angBwRightAxesNavDeg.end(),0.0)/angBwRightAxesNavDeg.size()<<" deg > 90 deg, therefore I think they're pointing in opposite directions so I'm flipping the shank axis, which is now: "<<m_rkneeAxisShank.point3().transpose()<<std::endl;}
    }
    // () now from the flex/ex knee angle determine if they are both pointed to the subject's right
    // determine this from the 10th and 90th percentile quantiles for outlier rejection
    std::vector<double> rkneeFlexExAngRad=getRKneeFlexExAngle();
    if(verbose){std::cout<<"rkneeFlexExAngRad: max is "<<*(std::max_element(rkneeFlexExAngRad.begin(),rkneeFlexExAngRad.end()))<<" and min is "<<*(std::min_element(rkneeFlexExAngRad.begin(),rkneeFlexExAngRad.end()))<<std::endl;}
    if( *std::max_element(rkneeFlexExAngRad.begin(),rkneeFlexExAngRad.end())<maxFlexExLimRad && *std::min_element(rkneeFlexExAngRad.begin(),rkneeFlexExAngRad.end())>minFlexExLimRad ){
        // this is the condition you want
        if(verbose){std::cout<<"    I believe both knee axes are pointing in the right direction because the max/min are in bounds"<<std::endl;}
    }else if(true){ // todo: add logic here to figure out if both knee axes are pointing to the subject's left
        std::cout<<"knee angle out of bounds!"<<std::endl;
    }
    // --- now do the same for the left leg ---
    // () are both axes pointing the same direction in the world frame?
    gtsam::Vector3 lThighAxis=m_lkneeAxisThigh.point3(), lShankAxis=m_lkneeAxisShank.point3();
    if(verbose) {
        std::cout << "    *** now checking left leg" << std::endl;
        std::cout<< "    lknee axis, thigh frame: "<<m_lkneeAxisThigh.point3().transpose()<<std::endl;
        std::cout<< "    lknee axis, shank frame: "<<m_lkneeAxisShank.point3().transpose()<<std::endl;
    }
    // get axes in world frame
    Eigen::MatrixXd lThighAxisNav(m_lthighImuOrientation.size(),3), lShankAxisNav(m_lshankImuOrientation.size(),3);
    std::vector<double> angBwLeftAxesNavDeg(m_lthighImuOrientation.size());
    for(size_t k=0; k<m_lthighImuOrientation.size();k++) {
        lThighAxisNav.block<1,3>(k, 0)=(m_lthighImuOrientation[k].matrix() * lThighAxis).transpose();
        lShankAxisNav.block<1,3>(k, 0)=(m_lshankImuOrientation[k].matrix() * lShankAxis).transpose();
        angBwLeftAxesNavDeg[k]= mathutils::unsignedAngle(lThighAxisNav.block<1,3>(k, 0), lShankAxisNav.block<1,3>(k, 0)) * 180.0 / M_PI; // put in degrees
    }
    if(std::accumulate(angBwLeftAxesNavDeg.begin(),angBwLeftAxesNavDeg.end(),0.0)/angBwLeftAxesNavDeg.size() <= 90.0){ // same side, stay
        if(verbose){std::cout<<"mean of angle b/w: "<<std::accumulate(angBwLeftAxesNavDeg.begin(),angBwLeftAxesNavDeg.end(),0.0)/angBwLeftAxesNavDeg.size()<<"deg < 90 deg, therefore I think they're pointing in the same direction"<<std::endl;}
    }else{
        m_lkneeAxisShank=gtsam::Unit3(-1.0*m_lkneeAxisShank.point3()); anyChangesMade=true;
        if(verbose){std::cout<<"mean of angle b/w: "<<std::accumulate(angBwLeftAxesNavDeg.begin(),angBwLeftAxesNavDeg.end(),0.0)/angBwLeftAxesNavDeg.size()<<"deg > 90 deg, therefore I think they're pointing in opposite directions so I'm flipping the shank axis, which is now: "<<m_lkneeAxisShank.point3().transpose()<<std::endl;}
    }
    // () now from the flex/ex knee angle determine if they are both pointed to the subject's right
    std::vector<double> lkneeFlexExAngRad=getLKneeFlexExAngle();
    double maxLKneeFlexEx=*(std::max_element(rkneeFlexExAngRad.begin(),rkneeFlexExAngRad.end())), minLKneeFlexEx=*(std::min_element(rkneeFlexExAngRad.begin(),rkneeFlexExAngRad.end()));
    if(verbose){std::cout<<"rkneeFlexExAngRad: max is "<<maxLKneeFlexEx<<" and min is "<<minLKneeFlexEx<<std::endl;}
    if( maxLKneeFlexEx<maxFlexExLimRad && minLKneeFlexEx>minFlexExLimRad ){
        // this is the condition you want
        if(verbose){std::cout<<"    I believe both knee axes are pointing in the right direction because the max/min are in bounds"<<std::endl;}
    }else if(true){ // todo: add logic here to figure out if both knee axes are pointing to the subject's left
        std::cout<<"knee angle out of bounds!"<<std::endl;
    }
    // () okay now you've done both legs. finally, rerun this one more time and make sure that no changes are made the second time.
    if(anyChangesMade) { // if any changes made, rerun a second time
        bool anyChangesMade2ndPass = postHocAxesCheck(false); // silently run
        if(!anyChangesMade2ndPass) { // good, what you want
            if(verbose){std::cout<<"    ***No changes made on the second pass of this method (as expected!). Exiting."<<std::endl;}
        }else {
            std::cerr<<"Why did something changes on the second pass?"<<std::endl;
        }
    }
    if(verbose){std::cout<<"-----------------------"<<std::endl;}
    return anyChangesMade;
}

void lowerBodyPoseEstimator::setAllInternalPrecessionAngles() {
    // calculates and sets all internal procession angles
    // set internal procession angles
    // todo: for sacrum we're assuming that [0 0 -1] is the up vector. do something better for this.
    m_sacrumImuInternalPrecessionAng=mathutils::imuInternalPrecessionAboutStaticVector(m_sacrumImuOrientation, Eigen::Vector3d(-1.0, 0.0, 0.0), (m_sacrumImuToRHipCtr - m_sacrumImuToLHipCtr).normalized());
    m_rthighImuInternalPrecessionAng=mathutils::imuInternalPrecessionAboutStaticVector(m_rthighImuOrientation, m_rthighImuToHipCtr, m_rkneeAxisThigh.point3());
    m_rshankImuInternalPrecessionAng=mathutils::imuInternalPrecessionAboutStaticVector(m_rshankImuOrientation, m_rshankImuToKneeCtr, m_rkneeAxisShank.point3());
    m_rfootImuInternalPrecessionAng=mathutils::imuInternalPrecessionAboutStaticVector(m_rfootImuOrientation, m_rfootImuToAnkleCtr, m_rankleAxisFoot.point3());
    m_lthighImuInternalPrecessionAng=mathutils::imuInternalPrecessionAboutStaticVector(m_lthighImuOrientation, m_lthighImuToHipCtr, m_lkneeAxisThigh.point3());
    m_lshankImuInternalPrecessionAng=mathutils::imuInternalPrecessionAboutStaticVector(m_lshankImuOrientation, m_lshankImuToKneeCtr, m_lkneeAxisShank.point3());
    m_lfootImuInternalPrecessionAng=mathutils::imuInternalPrecessionAboutStaticVector(m_lfootImuOrientation, m_lfootImuToAnkleCtr, m_lankleAxisFoot.point3());
}

void lowerBodyPoseEstimator::setDerivedJointAnglesFromEstimatedStates(bool verbose){
    // from the assumed-already-set estimated states in this model, set all of the derived joint angles
    // resize all vectors to hold angles
    m_rkneeFlexExAng.resize(m_sacrumImuOrientation.size()), m_rkneeIntExtRotAng.resize(m_sacrumImuOrientation.size()), m_rkneeAbAdAng.resize(m_sacrumImuOrientation.size());
    m_lkneeFlexExAng.resize(m_sacrumImuOrientation.size()), m_lkneeIntExtRotAng.resize(m_sacrumImuOrientation.size()), m_lkneeAbAdAng.resize(m_sacrumImuOrientation.size());
    m_rhipFlexExAng.resize(m_sacrumImuOrientation.size()), m_rhipIntExtRotAng.resize(m_sacrumImuOrientation.size()), m_rhipAbAdAng.resize(m_sacrumImuOrientation.size());
    m_lhipFlexExAng.resize(m_sacrumImuOrientation.size()), m_lhipIntExtRotAng.resize(m_sacrumImuOrientation.size()), m_lhipAbAdAng.resize(m_sacrumImuOrientation.size());
    // get segment coordinate system orientations
    std::vector<gtsam::Rot3> R_Pelvis_to_N=bioutils::get_R_Pelvis_to_N(m_sacrumImuOrientation,gtsam::Vector3(-1.0,0.0,0.0),m_sacrumImuToRHipCtr,m_sacrumImuToLHipCtr);
    std::vector<gtsam::Rot3> R_RFemur_to_N=bioutils::get_R_Segment_to_N(m_rthighImuOrientation,m_rkneeAxisThigh,m_rthighImuToHipCtr,m_rthighImuToKneeCtr,false);
    std::vector<gtsam::Rot3> R_RTibia_to_N=bioutils::get_R_Segment_to_N(m_rshankImuOrientation,m_rkneeAxisShank,m_rshankImuToKneeCtr,m_rshankImuToAnkleCtr);
    std::vector<gtsam::Rot3> R_LFemur_to_N=bioutils::get_R_Segment_to_N(m_lthighImuOrientation,m_lkneeAxisThigh,m_lthighImuToHipCtr,m_lthighImuToKneeCtr,false);
    std::vector<gtsam::Rot3> R_LTibia_to_N=bioutils::get_R_Segment_to_N(m_lshankImuOrientation,m_lkneeAxisShank,m_lshankImuToKneeCtr,m_lshankImuToAnkleCtr);
    // get angles
    Eigen::MatrixXd rkneeAngles=bioutils::clinical3DofJcsAngles(R_RFemur_to_N,R_RTibia_to_N,true,false);
    Eigen::MatrixXd lkneeAngles=bioutils::clinical3DofJcsAngles(R_LFemur_to_N,R_LTibia_to_N,false,false);
    Eigen::MatrixXd rhipAngles=bioutils::clinical3DofJcsAngles(R_Pelvis_to_N,R_RFemur_to_N,true,true);
    Eigen::MatrixXd lhipAngles=bioutils::clinical3DofJcsAngles(R_Pelvis_to_N,R_LFemur_to_N,false,true);
    // set angles to member variables
    m_rkneeFlexExAng=rkneeAngles.col(0), m_rkneeAbAdAng=rkneeAngles.col(1), m_rkneeIntExtRotAng=rkneeAngles.col(2);
    m_lkneeFlexExAng=lkneeAngles.col(0), m_lkneeAbAdAng=lkneeAngles.col(1), m_lkneeIntExtRotAng=lkneeAngles.col(2);
    m_rhipFlexExAng=rhipAngles.col(0), m_rhipAbAdAng=rhipAngles.col(1), m_rhipIntExtRotAng=rhipAngles.col(2);
    m_lhipFlexExAng=lhipAngles.col(0), m_lhipAbAdAng=lhipAngles.col(1), m_lhipIntExtRotAng=lhipAngles.col(2);
    // debug printing
    if(verbose){
        std::cout<<"------- joint angles -------"<<std::endl;
        std::string rhipInfo= "    rhip int/ext rot (deg): " + mathutils::distributionInfoString(180.0 / M_PI * rhipAngles.col(2));
        std::string lhipInfo= "    lhip int/ext rot (deg): " + mathutils::distributionInfoString(180.0 / M_PI * lhipAngles.col(2));
        std::string rkneeInfo= "    rknee int/ext rot (deg): " + mathutils::distributionInfoString(180.0 / M_PI * rkneeAngles.col(2));
        std::string lkneeInfo= "    lknee int/ext rot (deg): " + mathutils::distributionInfoString(180.0 / M_PI * lkneeAngles.col(2));
        std::cout<<rhipInfo<<std::endl<<lhipInfo<<std::endl<<rkneeInfo<<std::endl<<lkneeInfo<<std::endl;
        std::cout<<"----------------------------"<<std::endl;
    }
}

void lowerBodyPoseEstimator::saveResultsToH5File(const std::string& filename) const {
    // save results to .h5 file. similar to MATLAB function of same name.
    double startTic=clock();
    std::cout<<"saving results to .h5 file: "<<filename<<" ... ";
    H5Easy::File file(filename,H5Easy::File::Overwrite); // open file
    // add time to the root directory
    H5Easy::dump(file,"/Time",m_time,H5Easy::DumpMode::Create);
    // create Estimated group and add orientations, positions, velocities, accel & gyro biases
    file.createGroup("Estimated");
    // sacrum IMU states
    Eigen::MatrixXd R_SacrumImu_to_N=gtsamutils::vectorRot3ToFlattedEigenMatrixXd(m_sacrumImuOrientation);
    Eigen::MatrixXd q_SacrumImu_to_N=mathutils::Rot3VectorToQuaternionMatrix(m_sacrumImuOrientation);
    Eigen::MatrixXd p_SacrumImu=gtsamutils::Point3VectorToEigenMatrix(m_sacrumImuPosition);
    Eigen::MatrixXd v_SacrumImu=gtsamutils::Vector3VectorToEigenMatrix(m_sacrumImuVelocity);
    Eigen::MatrixXd gyrobias_SacrumImu=gtsamutils::Vector3VectorToEigenMatrix(m_sacrumImuGyroBias);
    Eigen::MatrixXd accelbias_SacrumImu=gtsamutils::Vector3VectorToEigenMatrix(m_sacrumImuAccelBias);
    H5Easy::dump(file,"/Estimated/R_SacrumImu_to_N",R_SacrumImu_to_N,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/q_SacrumImu_to_N",q_SacrumImu_to_N,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/p_SacrumImu",p_SacrumImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/v_SacrumImu",v_SacrumImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/gyrobias_SacrumImu",gyrobias_SacrumImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/accelbias_SacrumImu",accelbias_SacrumImu,H5Easy::DumpMode::Create);
    // rthigh IMU states
    Eigen::MatrixXd R_RThighImu_to_N=gtsamutils::vectorRot3ToFlattedEigenMatrixXd(m_rthighImuOrientation);
    Eigen::MatrixXd q_RThighImu_to_N=mathutils::Rot3VectorToQuaternionMatrix(m_rthighImuOrientation);
    Eigen::MatrixXd p_RThighImu=gtsamutils::Point3VectorToEigenMatrix(m_rthighImuPosition);
    Eigen::MatrixXd v_RThighImu=gtsamutils::Vector3VectorToEigenMatrix(m_rthighImuVelocity);
    Eigen::MatrixXd gyrobias_RThighImu=gtsamutils::Vector3VectorToEigenMatrix(m_rthighImuGyroBias);
    Eigen::MatrixXd accelbias_RThighImu=gtsamutils::Vector3VectorToEigenMatrix(m_rthighImuAccelBias);
    H5Easy::dump(file,"/Estimated/R_RThighImu_to_N",R_RThighImu_to_N,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/q_RThighImu_to_N",q_RThighImu_to_N,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/p_RThighImu",p_RThighImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/v_RThighImu",v_RThighImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/gyrobias_RThighImu",gyrobias_RThighImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/accelbias_RThighImu",accelbias_RThighImu,H5Easy::DumpMode::Create);
    // rshank IMU states
    Eigen::MatrixXd R_RShankImu_to_N=gtsamutils::vectorRot3ToFlattedEigenMatrixXd(m_rshankImuOrientation);
    Eigen::MatrixXd q_RShankImu_to_N=mathutils::Rot3VectorToQuaternionMatrix(m_rshankImuOrientation);
    Eigen::MatrixXd p_RShankImu=gtsamutils::Point3VectorToEigenMatrix(m_rshankImuPosition);
    Eigen::MatrixXd v_RShankImu=gtsamutils::Vector3VectorToEigenMatrix(m_rshankImuVelocity);
    Eigen::MatrixXd gyrobias_RShankImu=gtsamutils::Vector3VectorToEigenMatrix(m_rshankImuGyroBias);
    Eigen::MatrixXd accelbias_RShankImu=gtsamutils::Vector3VectorToEigenMatrix(m_rshankImuAccelBias);
    H5Easy::dump(file,"/Estimated/R_RShankImu_to_N",R_RShankImu_to_N,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/q_RShankImu_to_N",q_RShankImu_to_N,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/p_RShankImu",p_RShankImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/v_RShankImu",v_RShankImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/gyrobias_RShankImu",gyrobias_RShankImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/accelbias_RShankImu",accelbias_RShankImu,H5Easy::DumpMode::Create);
    // rfoot IMU states
    Eigen::MatrixXd R_RFootImu_to_N=gtsamutils::vectorRot3ToFlattedEigenMatrixXd(m_rfootImuOrientation);
    Eigen::MatrixXd q_RFootImu_to_N=mathutils::Rot3VectorToQuaternionMatrix(m_rfootImuOrientation);
    Eigen::MatrixXd p_RFootImu=gtsamutils::Point3VectorToEigenMatrix(m_rfootImuPosition);
    Eigen::MatrixXd v_RFootImu=gtsamutils::Vector3VectorToEigenMatrix(m_rfootImuVelocity);
    Eigen::MatrixXd gyrobias_RFootImu=gtsamutils::Vector3VectorToEigenMatrix(m_rfootImuGyroBias);
    Eigen::MatrixXd accelbias_RFootImu=gtsamutils::Vector3VectorToEigenMatrix(m_rfootImuAccelBias);
    H5Easy::dump(file,"/Estimated/R_RFootImu_to_N",R_RFootImu_to_N,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/q_RFootImu_to_N",q_RFootImu_to_N,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/p_RFootImu",p_RFootImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/v_RFootImu",v_RFootImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/gyrobias_RFootImu",gyrobias_RFootImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/accelbias_RFootImu",accelbias_RFootImu,H5Easy::DumpMode::Create);
    // lthigh IMU states
    Eigen::MatrixXd R_LThighImu_to_N=gtsamutils::vectorRot3ToFlattedEigenMatrixXd(m_lthighImuOrientation);
    Eigen::MatrixXd q_LThighImu_to_N=mathutils::Rot3VectorToQuaternionMatrix(m_lthighImuOrientation);
    Eigen::MatrixXd p_LThighImu=gtsamutils::Point3VectorToEigenMatrix(m_lthighImuPosition);
    Eigen::MatrixXd v_LThighImu=gtsamutils::Vector3VectorToEigenMatrix(m_lthighImuVelocity);
    Eigen::MatrixXd gyrobias_LThighImu=gtsamutils::Vector3VectorToEigenMatrix(m_lthighImuGyroBias);
    Eigen::MatrixXd accelbias_LThighImu=gtsamutils::Vector3VectorToEigenMatrix(m_lthighImuAccelBias);
    H5Easy::dump(file,"/Estimated/R_LThighImu_to_N",R_LThighImu_to_N,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/q_LThighImu_to_N",q_LThighImu_to_N,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/p_LThighImu",p_LThighImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/v_LThighImu",v_LThighImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/gyrobias_LThighImu",gyrobias_LThighImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/accelbias_LThighImu",accelbias_LThighImu,H5Easy::DumpMode::Create);
    // lshank IMU states
    Eigen::MatrixXd R_LShankImu_to_N=gtsamutils::vectorRot3ToFlattedEigenMatrixXd(m_lshankImuOrientation);
    Eigen::MatrixXd q_LShankImu_to_N=mathutils::Rot3VectorToQuaternionMatrix(m_lshankImuOrientation);
    Eigen::MatrixXd p_LShankImu=gtsamutils::Point3VectorToEigenMatrix(m_lshankImuPosition);
    Eigen::MatrixXd v_LShankImu=gtsamutils::Vector3VectorToEigenMatrix(m_lshankImuVelocity);
    Eigen::MatrixXd gyrobias_LShankImu=gtsamutils::Vector3VectorToEigenMatrix(m_lshankImuGyroBias);
    Eigen::MatrixXd accelbias_LShankImu=gtsamutils::Vector3VectorToEigenMatrix(m_lshankImuAccelBias);
    H5Easy::dump(file,"/Estimated/R_LShankImu_to_N",R_LShankImu_to_N,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/q_LShankImu_to_N",q_LShankImu_to_N,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/p_LShankImu",p_LShankImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/v_LShankImu",v_LShankImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/gyrobias_LShankImu",gyrobias_LShankImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/accelbias_LShankImu",accelbias_LShankImu,H5Easy::DumpMode::Create);
    // lfoot IMU states
    Eigen::MatrixXd R_LFootImu_to_N=gtsamutils::vectorRot3ToFlattedEigenMatrixXd(m_lfootImuOrientation);
    Eigen::MatrixXd q_LFootImu_to_N=mathutils::Rot3VectorToQuaternionMatrix(m_lfootImuOrientation);
    Eigen::MatrixXd p_LFootImu=gtsamutils::Point3VectorToEigenMatrix(m_lfootImuPosition);
    Eigen::MatrixXd v_LFootImu=gtsamutils::Vector3VectorToEigenMatrix(m_lfootImuVelocity);
    Eigen::MatrixXd gyrobias_LFootImu=gtsamutils::Vector3VectorToEigenMatrix(m_lfootImuGyroBias);
    Eigen::MatrixXd accelbias_LFootImu=gtsamutils::Vector3VectorToEigenMatrix(m_lfootImuAccelBias);
    H5Easy::dump(file,"/Estimated/R_LFootImu_to_N",R_LFootImu_to_N,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/q_LFootImu_to_N",q_LFootImu_to_N,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/p_LFootImu",p_LFootImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/v_LFootImu",v_LFootImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/gyrobias_LFootImu",gyrobias_LFootImu,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/accelbias_LFootImu",accelbias_LFootImu,H5Easy::DumpMode::Create);
    // knee axes
    H5Easy::dump(file,"/Estimated/RKneeAxis_RThigh",(Eigen::Vector3d) m_rkneeAxisThigh.point3(),H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/RKneeAxis_RShank",(Eigen::Vector3d) m_rkneeAxisShank.point3(),H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/LKneeAxis_LThigh",(Eigen::Vector3d) m_lkneeAxisThigh.point3(),H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/LKneeAxis_LShank",(Eigen::Vector3d) m_lkneeAxisShank.point3(),H5Easy::DumpMode::Create);
    // static vectors to joint rotation centers
    H5Easy::dump(file,"/Estimated/SacrumImuToRHipCtr",(Eigen::Vector3d) m_sacrumImuToRHipCtr,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/RThighImuToRHipCtr",(Eigen::Vector3d) m_rthighImuToHipCtr,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/RThighImuToRKneeCtr",(Eigen::Vector3d) m_rthighImuToKneeCtr,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/RShankImuToRKneeCtr",(Eigen::Vector3d) m_rshankImuToKneeCtr,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/RShankImuToRAnkleCtr",(Eigen::Vector3d) m_rshankImuToAnkleCtr,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/RFootImuToRAnkleCtr",(Eigen::Vector3d) m_rfootImuToAnkleCtr,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/SacrumImuToLHipCtr",(Eigen::Vector3d) m_sacrumImuToLHipCtr,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/LThighImuToLHipCtr",(Eigen::Vector3d) m_lthighImuToHipCtr,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/LThighImuToLKneeCtr",(Eigen::Vector3d) m_lthighImuToKneeCtr,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/LShankImuToLKneeCtr",(Eigen::Vector3d) m_lshankImuToKneeCtr,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/LShankImuToLAnkleCtr",(Eigen::Vector3d) m_lshankImuToAnkleCtr,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Estimated/LFootImuToLAnkleCtr",(Eigen::Vector3d) m_lfootImuToAnkleCtr,H5Easy::DumpMode::Create);
    if(m_assumeHipHinge){ // add hip hinge vectors
        H5Easy::dump(file,"/Estimated/HipAxis_Sacrum",(Eigen::Vector3d) m_hipAxisSacrum.point3(),H5Easy::DumpMode::Create);
        H5Easy::dump(file,"/Estimated/RHipAxis_RThigh",(Eigen::Vector3d) m_rhipAxisThigh.point3(),H5Easy::DumpMode::Create);
        H5Easy::dump(file,"/Estimated/LHipAxis_LThigh",(Eigen::Vector3d) m_lhipAxisThigh.point3(),H5Easy::DumpMode::Create);
    }
    if(m_assumeAnkleHinge){ // add ankle hinge vectors
        H5Easy::dump(file,"/Estimated/RAnkleAxis_Foot",(Eigen::Vector3d) m_rankleAxisFoot.point3(),H5Easy::DumpMode::Create);
        H5Easy::dump(file,"/Estimated/LAnkleAxis_Foot",(Eigen::Vector3d) m_lankleAxisFoot.point3(),H5Easy::DumpMode::Create);
        H5Easy::dump(file,"/Estimated/RAnkleAxis_RThigh",(Eigen::Vector3d) m_rankleAxisShank.point3(),H5Easy::DumpMode::Create);
        H5Easy::dump(file,"/Estimated/LAnkleAxis_LThigh",(Eigen::Vector3d) m_lankleAxisShank.point3(),H5Easy::DumpMode::Create);
    }
    // derived quantities: segment coordinate system, joint angles, etc.
    file.createGroup("Derived");
    // joint angles of the hip and knee
    H5Easy::dump(file,"/Derived/RKneeAngles_FlexEx",m_rkneeFlexExAng,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Derived/RKneeAngles_IntExtRot",m_rkneeIntExtRotAng,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Derived/RKneeAngles_AbAd",m_rkneeAbAdAng,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Derived/LKneeAngles_FlexEx",m_lkneeFlexExAng,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Derived/LKneeAngles_IntExtRot",m_lkneeIntExtRotAng,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Derived/LKneeAngles_AbAd",m_lkneeAbAdAng,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Derived/RHipAngles_FlexEx",m_rhipFlexExAng,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Derived/RHipAngles_IntExtRot",m_rhipIntExtRotAng,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Derived/RHipAngles_AbAd",m_rhipAbAdAng,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Derived/LHipAngles_FlexEx",m_lhipFlexExAng,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Derived/LHipAngles_IntExtRot",m_lhipIntExtRotAng,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Derived/LHipAngles_AbAd",m_lhipAbAdAng,H5Easy::DumpMode::Create);
    // internal precession angles
    H5Easy::dump(file,"/Derived/sacrumImuInternalPrecessionAngle",m_sacrumImuInternalPrecessionAng,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Derived/rthighImuInternalPrecessionAngle",m_rthighImuInternalPrecessionAng,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Derived/rshankImuInternalPrecessionAngle",m_rshankImuInternalPrecessionAng,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Derived/rfootImuInternalPrecessionAngle",m_rfootImuInternalPrecessionAng,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Derived/lthighImuInternalPrecessionAngle",m_lthighImuInternalPrecessionAng,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Derived/lshankImuInternalPrecessionAngle",m_lshankImuInternalPrecessionAng,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/Derived/lfootImuInternalPrecessionAngle",m_lfootImuInternalPrecessionAng,H5Easy::DumpMode::Create);
    // model settings
    file.createGroup("ModelSettings");
    H5Easy::dump(file,"/ModelSettings/ModelType",(std::string)"LowerBodyPoseEstimator",H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/useRFemurLengthFactor",m_useRFemurLengthFactor,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/useRTibiaLengthFactor",m_useRTibiaLengthFactor,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/useLFemurLengthFactor",m_useLFemurLengthFactor,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/useLTibiaLengthFactor",m_useLTibiaLengthFactor,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/usePelvicWidthFactor",m_usePelvicWidthFactor,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/useFemurLengthDiscrepancyFactor",m_useFemurLengthDiscrepancyFactor,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/useTibiaLengthDiscrepancyFactor",m_useTibiaLengthDiscrepancyFactor,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/useKneeAxisTibiaOrthogonalityFactor",m_useKneeAxisTibiaOrthogonalityFactor,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/useKneeAxisFemurOrthogonalityFactor",m_useKneeAxisFemurOrthogonalityFactor,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/assumeHipHinge", m_assumeHipHinge, H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/assumeAnkleHinge", m_assumeAnkleHinge, H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/gender",m_gender,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/rfemurLength",m_rfemurLength,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/rtibiaLength",m_rtibiaLength,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/lfemurLength",m_lfemurLength,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/ltibiaLength",m_ltibiaLength,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/pelvicWidth",m_pelvicWidth,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/rfemurLengthSigma",m_rfemurLengthSigma,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/rtibiaLengthSigma",m_rtibiaLengthSigma,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/lfemurLengthSigma",m_lfemurLengthSigma,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/ltibiaLengthSigma",m_ltibiaLengthSigma,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/pelvicWidthSigma",m_pelvicWidthSigma,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/angVelNoise",m_angVelNoise,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/kneeJointCtrConnectionNoise",m_kneeJointCtrConnectionNoise,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/hipJointCtrConnectionNoise",m_hipJointCtrConnectionNoise,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/ankleJointCtrConnectionNoise",m_ankleJointCtrConnectionNoise,H5Easy::DumpMode::Create);
    H5Easy::dump(file, "/ModelSettings/kneeHingeAxisNoise", m_kneeHingeAxisNoise, H5Easy::DumpMode::Create);
    H5Easy::dump(file, "/ModelSettings/ankleHingeAxisNoise", m_ankleHingeAxisNoise, H5Easy::DumpMode::Create);
    H5Easy::dump(file, "/ModelSettings/hipHingeAxisNoise", m_hipHingeAxisNoise, H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/femurLengthDiscrepancyNoise",m_femurLengthDiscrepancyNoise,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/tibiaLengthDiscrepancyNoise",m_tibiaLengthDiscrepancyNoise,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/angBwKneeAxisAndSegmentMeanRFemur",m_angBwKneeAxisAndSegmentMeanRFemur,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/angBwKneeAxisAndSegmentStdRFemur",m_angBwKneeAxisAndSegmentStdFemur,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/angBwKneeAxisAndSegmentMeanLFemur",m_angBwKneeAxisAndSegmentMeanLFemur,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/angBwKneeAxisAndSegmentStdLFemur",m_angBwKneeAxisAndSegmentStdFemur,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/angBwKneeAxisAndSegmentMeanRTibia",m_angBwKneeAxisAndSegmentMeanRTibia,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/angBwKneeAxisAndSegmentStdRTibia",m_angBwKneeAxisAndSegmentStdTibia,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/angBwKneeAxisAndSegmentMeanLTibia",m_angBwKneeAxisAndSegmentMeanLTibia,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/angBwKneeAxisAndSegmentStdLTibia",m_angBwKneeAxisAndSegmentStdTibia,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/useMaxAnthroConstraint",m_useMaxAnthroConstraint,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/useMinAnthroConstraint",m_useMinAnthroConstraint,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/useJointVelFactor",m_useJointVelFactor,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/jointCtrVelConnectionNoise",m_jointCtrVelConnectionNoise,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/ModelSettings/numPreintegratedImuMeasurements",m_sacrumImuPoseProblem.m_numMeasToPreint,H5Easy::DumpMode::Create);
    // IMU angular velocities
    Eigen::MatrixXd sacrumImuAngVel=gtsamutils::Vector3VectorToEigenMatrix(m_sacrumImuAngVel);
    H5Easy::dump(file,"/Estimated/SacrumImuAngVel",sacrumImuAngVel,H5Easy::DumpMode::Create);
    Eigen::MatrixXd rthighImuAngVel=gtsamutils::Vector3VectorToEigenMatrix(m_rthighImuAngVel);
    H5Easy::dump(file,"/Estimated/RThighImuAngVel",rthighImuAngVel,H5Easy::DumpMode::Create);
    Eigen::MatrixXd rshankImuAngVel=gtsamutils::Vector3VectorToEigenMatrix(m_rshankImuAngVel);
    H5Easy::dump(file,"/Estimated/RShankImuAngVel",rshankImuAngVel,H5Easy::DumpMode::Create);
    Eigen::MatrixXd rfootImuAngVel=gtsamutils::Vector3VectorToEigenMatrix(m_rfootImuAngVel);
    H5Easy::dump(file,"/Estimated/RFootImuAngVel",rfootImuAngVel,H5Easy::DumpMode::Create);
    Eigen::MatrixXd lthighImuAngVel=gtsamutils::Vector3VectorToEigenMatrix(m_lthighImuAngVel);
    H5Easy::dump(file,"/Estimated/LThighImuAngVel",lthighImuAngVel,H5Easy::DumpMode::Create);
    Eigen::MatrixXd lshankImuAngVel=gtsamutils::Vector3VectorToEigenMatrix(m_lshankImuAngVel);
    H5Easy::dump(file,"/Estimated/LShankImuAngVel",lshankImuAngVel,H5Easy::DumpMode::Create);
    Eigen::MatrixXd lfootImuAngVel=gtsamutils::Vector3VectorToEigenMatrix(m_lfootImuAngVel);
    H5Easy::dump(file,"/Estimated/LFootImuAngVel",lfootImuAngVel,H5Easy::DumpMode::Create);
    // solver options
    file.createGroup("SolverSettings");
    H5Easy::dump(file,"/SolverSettings/optimizerType",m_optimizerType,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/SolverSettings/absErrDecreaseLimit",m_absErrDecreaseLimit,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/SolverSettings/relErrDecreaseLimit",m_relErrDecreaseLimit,H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/SolverSettings/maxIterations",m_maxIterations,H5Easy::DumpMode::Create);
    // general stuff
    file.createGroup("General");
    H5Easy::dump(file,"/General/GeneratingModelType",(std::string)"cpp.lowerBodyPoseEstimator",H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/General/nKeyframes",m_sacrumImuOrientation.size(),H5Easy::DumpMode::Create);
    H5Easy::dump(file,"/General/ID",m_strId,H5Easy::DumpMode::Create);
    std::cout<<" complete! ("<<(clock()-startTic)/CLOCKS_PER_SEC<<" sec)"<<std::endl;
}

void lowerBodyPoseEstimator::printStaticStatesSummary(){
    // prints to std::cout the general static states. similar to the MATLAB function.
    std::cout<<"rknee axis, thigh frame: ["<<m_rkneeAxisThigh.point3().transpose()<<"]"<<std::endl;
    std::cout<<"rknee axis, shank frame: ["<<m_rkneeAxisShank.point3().transpose()<<"]"<<std::endl;
    std::cout << "    mean angle b/w rknee axes in nav frame: " << mathutils::meanAngBwAxesNavFrame(m_rthighImuOrientation, m_rkneeAxisThigh, m_rshankImuOrientation, m_rkneeAxisShank) * 180.0 / M_PI << " deg" << std::endl;
    std::cout<<"lknee axis, thigh frame: ["<<m_lkneeAxisThigh.point3().transpose()<<"]"<<std::endl;
    std::cout<<"lknee axis, shank frame: ["<<m_lkneeAxisShank.point3().transpose()<<"]"<<std::endl;
    std::cout << "    mean angle b/w lknee axes in nav frame: " << mathutils::meanAngBwAxesNavFrame(m_lthighImuOrientation, m_lkneeAxisThigh, m_lshankImuOrientation, m_lkneeAxisShank) * 180.0 / M_PI << " deg" << std::endl;;
    if(m_assumeHipHinge){ // also print hip hinge axes
        std::cout<<"both hip axis, sacrum frame: ["<<m_hipAxisSacrum.point3().transpose()<<"]"<<std::endl;
        std::cout<<"rhip axis, thigh frame: ["<<m_rhipAxisThigh.point3().transpose()<<"]"<<std::endl;
        std::cout << "    (angle b/w rhip and rknee axes in thigh frame: " << mathutils::unsignedAngle(m_rkneeAxisThigh.unitVector(), m_rhipAxisThigh.unitVector()) * 180.0 / M_PI << " deg)" << std::endl;
        std::cout << "    (mean angle b/w rhip axes in nav frame: " << mathutils::meanAngBwAxesNavFrame(m_sacrumImuOrientation, m_hipAxisSacrum, m_rthighImuOrientation, m_rhipAxisThigh) * 180.0 / M_PI << " deg)" << std::endl;
        std::cout<<"lhip axis, thigh frame: ["<<m_lhipAxisThigh.point3().transpose()<<"]"<<std::endl;
        std::cout << "    (angle b/w lhip and lknee axes in thigh frame: " << mathutils::unsignedAngle(m_lkneeAxisThigh.unitVector(), m_lhipAxisThigh.unitVector()) * 180.0 / M_PI << " deg)" << std::endl;
        std::cout << "    (mean angle b/w lhip axes in nav frame: " << mathutils::meanAngBwAxesNavFrame(m_sacrumImuOrientation, m_hipAxisSacrum, m_lthighImuOrientation, m_lhipAxisThigh) * 180.0 / M_PI << " deg)" << std::endl;
    }
    if(m_assumeAnkleHinge){ // also print ankle hinge axes
        std::cout<<"rankle axis, foot frame: ["<<m_rankleAxisFoot.point3().transpose()<<"]"<<std::endl;
        std::cout<<"lankle axis, foot frame: ["<<m_lankleAxisFoot.point3().transpose()<<"]"<<std::endl;
        std::cout<<"rankle axis, shank frame: ["<<m_rankleAxisShank.point3().transpose()<<"]"<<std::endl;
        std::cout << "    (angle b/w rankle and rknee axes in shank frame: " << mathutils::unsignedAngle(m_rkneeAxisShank.unitVector(), m_rankleAxisShank.unitVector()) * 180.0 / M_PI << " deg)" << std::endl;
        std::cout << "    (mean angle b/w rankle axes in nav frame: " << mathutils::meanAngBwAxesNavFrame(m_rshankImuOrientation, m_rankleAxisShank, m_rfootImuOrientation, m_rankleAxisFoot) * 180.0 / M_PI << " deg)" << std::endl;
        std::cout<<"lankle axis, shank frame: ["<<m_lankleAxisShank.point3().transpose()<<"]"<<std::endl;
        std::cout << "    (angle b/w lankle and lknee axes in shank frame: " << mathutils::unsignedAngle(m_lkneeAxisShank.unitVector(), m_lankleAxisShank.unitVector()) * 180.0 / M_PI << " deg)" << std::endl;
        std::cout << "    (mean angle b/w lankle axes in nav frame: " << mathutils::meanAngBwAxesNavFrame(m_lshankImuOrientation, m_lankleAxisShank, m_lfootImuOrientation, m_lankleAxisFoot) * 180.0 / M_PI << " deg)" << std::endl;
    }
    std::cout<<"sacrum imu to rhip ctr: ["<<m_sacrumImuToRHipCtr.transpose()<<"] (norm="<<m_sacrumImuToRHipCtr.norm()<<")"<<std::endl;
    std::cout<<"sacrum imu to lhip ctr: ["<<m_sacrumImuToLHipCtr.transpose()<<"] (norm="<<m_sacrumImuToLHipCtr.norm()<<")"<<std::endl;
    std::cout<<"thigh imu to rhip ctr: ["<<m_rthighImuToHipCtr.transpose()<<"] (norm="<<m_rthighImuToHipCtr.norm()<<")"<<std::endl;
    std::cout<<"thigh imu to rknee ctr: ["<<m_rthighImuToKneeCtr.transpose()<<"] (norm="<<m_rthighImuToKneeCtr.norm()<<")"<<std::endl;
    std::cout<<"shank imu to rknee ctr: ["<<m_rshankImuToKneeCtr.transpose()<<"] (norm="<<m_rshankImuToKneeCtr.norm()<<")"<<std::endl;
    std::cout<<"shank imu to rankle ctr: ["<<m_rshankImuToAnkleCtr.transpose()<<"] (norm="<<m_rshankImuToAnkleCtr.norm()<<")"<<std::endl;
    std::cout<<"foot imu to rankle ctr: ["<<m_rfootImuToAnkleCtr.transpose()<<"] (norm="<<m_rfootImuToAnkleCtr.norm()<<")"<<std::endl;
    std::cout<<"thigh imu to lhip ctr: ["<<m_lthighImuToHipCtr.transpose()<<"] (norm="<<m_lthighImuToHipCtr.norm()<<")"<<std::endl;
    std::cout<<"thigh imu to lknee ctr: ["<<m_lthighImuToKneeCtr.transpose()<<"] (norm="<<m_lthighImuToKneeCtr.norm()<<")"<<std::endl;
    std::cout<<"shank imu to lknee ctr: ["<<m_lshankImuToKneeCtr.transpose()<<"] (norm="<<m_lshankImuToKneeCtr.norm()<<")"<<std::endl;
    std::cout<<"shank imu to lankle ctr: ["<<m_lshankImuToAnkleCtr.transpose()<<"] (norm="<<m_lshankImuToAnkleCtr.norm()<<")"<<std::endl;
    std::cout<<"foot imu to lankle ctr: ["<<m_lfootImuToAnkleCtr.transpose()<<"] (norm="<<m_lfootImuToAnkleCtr.norm()<<")"<<std::endl;
    // print the angle between knee axis and the segment vector
    std::cout<<"angle between knee axis and the femur/tibia segment proximal vector"<<std::endl;
    std::cout << "    right thigh imu: " << 180.0 / M_PI * mathutils::unsignedAngle(m_rkneeAxisThigh.unitVector(), (m_rthighImuToHipCtr - m_rthighImuToKneeCtr)) << " deg" << std::endl;
    std::cout << "    right shank imu: " << 180.0 / M_PI * mathutils::unsignedAngle(m_rkneeAxisShank.unitVector(), (m_rshankImuToKneeCtr - m_rshankImuToAnkleCtr)) << " deg" << std::endl;
    std::cout << "    left thigh imu: " << 180.0 / M_PI * mathutils::unsignedAngle(m_lkneeAxisThigh.unitVector(), (m_lthighImuToHipCtr - m_lthighImuToKneeCtr)) << " deg" << std::endl;
    std::cout << "    left shank imu: " << 180.0 / M_PI * mathutils::unsignedAngle(m_lkneeAxisShank.unitVector(), (m_lshankImuToKneeCtr - m_lshankImuToAnkleCtr)) << " deg" << std::endl;

    // geometry of each IMU's triangle to adjacent joint centers
    double estRThighLength=(m_rthighImuToHipCtr-m_rthighImuToKneeCtr).norm();
    double estRShankLength=(m_rshankImuToKneeCtr-m_rshankImuToAnkleCtr).norm();
    double estLThighLength=(m_lthighImuToHipCtr-m_lthighImuToKneeCtr).norm();
    double estLShankLength=(m_lshankImuToKneeCtr-m_lshankImuToAnkleCtr).norm();
    double estPelvicWidth=(m_sacrumImuToRHipCtr-m_sacrumImuToLHipCtr).norm();
    std::vector<double> hipSep=mathutils::ptSeparationNorm(m_rthighImuPose, m_rthighImuToHipCtr, m_lthighImuPose, m_lthighImuToHipCtr);
    std::cout<<"Estimated anthropometry: RFemur="<<estRThighLength<<", "<<"RTibia="<<estRShankLength<<", LFemur="<<estLThighLength<<", LTibia="<<estLShankLength<<", pelvic width="<<estPelvicWidth<<std::endl;
    std::cout << "    hip sep. from thigh IMUs: mean=" << mathutils::mean(hipSep) << ", median=" << mathutils::median(hipSep) << ", max=" << mathutils::max(hipSep) << ", min=" << mathutils::min(hipSep) << std::endl;
    std::cout<<"leg length discrepancies:"<<std::endl;
    std::cout<<"    femur length discrepancy = "<<estRThighLength-estLThighLength<<std::endl;
    std::cout<<"    tibia length discrepancy = "<<estRShankLength-estLShankLength<<std::endl;
    std::cout<<"    right leg length: "<<estRThighLength+estRShankLength<<" (estRThighLength="<<estRThighLength<<" + estRShankLength="<<estRShankLength<<")"<<std::endl;
    std::cout<<"    left leg length: "<<estLThighLength+estLShankLength<<" (estLThighLength="<<estLThighLength<<" + estLShankLength="<<estLShankLength<<")"<<std::endl;
    std::cout<<"        difference = "<<abs((estRThighLength+estRShankLength)-(estLThighLength+estLShankLength))*1000.0<<" mm"<<std::endl;
    // if static, print imu biases
    if(m_sacrumImuGyroBias.size()==1){std::cout<<"estimated static sacrum IMU bias: gyro=["<<m_sacrumImuGyroBias[0].transpose()<<"] rad/s, accel=["<<m_sacrumImuAccelBias[0].transpose()<<"] m/s^2"<<std::endl;}
    if(m_rthighImuGyroBias.size()==1){std::cout<<"estimated static rthigh IMU bias: gyro=["<<m_rthighImuGyroBias[0].transpose()<<"] rad/s, accel=["<<m_rthighImuAccelBias[0].transpose()<<"] m/s^2"<<std::endl;}
    if(m_rshankImuGyroBias.size()==1){std::cout<<"estimated static rshank IMU bias: gyro=["<<m_rshankImuGyroBias[0].transpose()<<"] rad/s, accel=["<<m_rshankImuAccelBias[0].transpose()<<"] m/s^2"<<std::endl;}
    if(m_rfootImuGyroBias.size()==1){std::cout<<"estimated static rfoot IMU bias: gyro=["<<m_rfootImuGyroBias[0].transpose()<<"] rad/s, accel=["<<m_rfootImuAccelBias[0].transpose()<<"] m/s^2"<<std::endl;}
    if(m_lthighImuGyroBias.size()==1){std::cout<<"estimated static lthigh IMU bias: gyro=["<<m_lthighImuGyroBias[0].transpose()<<"] rad/s, accel=["<<m_lthighImuAccelBias[0].transpose()<<"] m/s^2"<<std::endl;}
    if(m_lshankImuGyroBias.size()==1){std::cout<<"estimated static lshank IMU bias: gyro=["<<m_lshankImuGyroBias[0].transpose()<<"] rad/s, accel=["<<m_lshankImuAccelBias[0].transpose()<<"] m/s^2"<<std::endl;}
    if(m_lfootImuGyroBias.size()==1){std::cout<<"estimated static lfoot IMU bias: gyro=["<<m_lfootImuGyroBias[0].transpose()<<"] rad/s, accel=["<<m_lfootImuAccelBias[0].transpose()<<"] m/s^2"<<std::endl;}
}

void lowerBodyPoseEstimator::setMemberEstimatedStatesFromValues(const gtsam::Values& vals){
    // first, set all static variables quickly from their saved key
    m_rkneeAxisThigh = vals.at<gtsam::Unit3>(m_rkneeAxisThighKey);
    m_rkneeAxisShank = vals.at<gtsam::Unit3>(m_rkneeAxisShankKey);
    m_lkneeAxisThigh = vals.at<gtsam::Unit3>(m_lkneeAxisThighKey);
    m_lkneeAxisShank = vals.at<gtsam::Unit3>(m_lkneeAxisShankKey);
    if(m_assumeHipHinge){
        m_hipAxisSacrum=vals.at<gtsam::Unit3>(m_hipAxisSacrumKey);
        m_rhipAxisThigh=vals.at<gtsam::Unit3>(m_rhipAxisThighKey);
        m_lhipAxisThigh=vals.at<gtsam::Unit3>(m_lhipAxisThighKey);
    }
    if(m_assumeAnkleHinge){
        m_rankleAxisFoot=vals.at<gtsam::Unit3>(m_rankleAxisFootKey);
        m_lankleAxisFoot=vals.at<gtsam::Unit3>(m_lankleAxisFootKey);
        m_rankleAxisShank=vals.at<gtsam::Unit3>(m_rankleAxisShankKey);
        m_lankleAxisShank=vals.at<gtsam::Unit3>(m_lankleAxisShankKey);
    }
    m_sacrumImuToRHipCtr = vals.at<gtsam::Point3>(m_sacrumImuToRHipCtrKey);
    m_rthighImuToHipCtr = vals.at<gtsam::Point3>(m_rthighImuToHipCtrKey);
    m_rthighImuToKneeCtr = vals.at<gtsam::Point3>(m_rthighImuToKneeCtrKey);
    m_rshankImuToKneeCtr = vals.at<gtsam::Point3>(m_rshankImuToKneeCtrKey);
    m_rshankImuToAnkleCtr = vals.at<gtsam::Point3>(m_rshankImuToAnkleCtrKey);
    m_rfootImuToAnkleCtr = vals.at<gtsam::Point3>(m_rfootImuToAnkleCtrKey);
    m_sacrumImuToLHipCtr = vals.at<gtsam::Point3>(m_sacrumImuToLHipCtrKey);
    m_lthighImuToHipCtr = vals.at<gtsam::Point3>(m_lthighImuToHipCtrKey);
    m_lthighImuToKneeCtr = vals.at<gtsam::Point3>(m_lthighImuToKneeCtrKey);
    m_lshankImuToKneeCtr = vals.at<gtsam::Point3>(m_lshankImuToKneeCtrKey);
    m_lshankImuToAnkleCtr = vals.at<gtsam::Point3>(m_lshankImuToAnkleCtrKey);
    m_lfootImuToAnkleCtr = vals.at<gtsam::Point3>(m_lfootImuToAnkleCtrKey);
    // --- set angular veloicty values ---
    // note: imuPoseEstimator::vectorizeVelocities is really just a vectorization of a generic gtsam::Vector3, so you can use it below.
    m_sacrumImuAngVel=imuPoseEstimator::vectorizeVelocities(vals,m_sacrumImuAngVelKeys);
    m_rthighImuAngVel=imuPoseEstimator::vectorizeVelocities(vals,m_rthighImuAngVelKeys);
    m_rshankImuAngVel=imuPoseEstimator::vectorizeVelocities(vals,m_rshankImuAngVelKeys);
    m_rfootImuAngVel=imuPoseEstimator::vectorizeVelocities(vals,m_rfootImuAngVelKeys);
    m_lthighImuAngVel=imuPoseEstimator::vectorizeVelocities(vals,m_lthighImuAngVelKeys);
    m_lshankImuAngVel=imuPoseEstimator::vectorizeVelocities(vals,m_lshankImuAngVelKeys);
    m_lfootImuAngVel=imuPoseEstimator::vectorizeVelocities(vals,m_lfootImuAngVelKeys);
    // --- now get time series poses, velocities and biases of each IMU ---
    m_sacrumImuPose=imuPoseEstimator::vectorizePoses(vals,m_sacrumImuPoseProblem.m_poseKeys);
    m_sacrumImuOrientation=imuPoseEstimator::vectorizeOrientations(m_sacrumImuPose);
    m_sacrumImuPosition=imuPoseEstimator::vectorizePositions(m_sacrumImuPose);
    m_sacrumImuVelocity=imuPoseEstimator::vectorizeVelocities(vals,m_sacrumImuPoseProblem.m_velKeys);
    m_sacrumImuGyroBias=imuPoseEstimator::vectorizeGyroBiases(vals,m_sacrumImuPoseProblem.m_imuBiasKeys);
    m_sacrumImuAccelBias=imuPoseEstimator::vectorizeAccelBiases(vals,m_sacrumImuPoseProblem.m_imuBiasKeys);
    m_rthighImuPose=imuPoseEstimator::vectorizePoses(vals,m_rthighImuPoseProblem.m_poseKeys);
    m_rthighImuOrientation=imuPoseEstimator::vectorizeOrientations(m_rthighImuPose);
    m_rthighImuPosition=imuPoseEstimator::vectorizePositions(m_rthighImuPose);
    m_rthighImuVelocity=imuPoseEstimator::vectorizeVelocities(vals,m_rthighImuPoseProblem.m_velKeys);
    m_rthighImuGyroBias=imuPoseEstimator::vectorizeGyroBiases(vals,m_rthighImuPoseProblem.m_imuBiasKeys);
    m_rthighImuAccelBias=imuPoseEstimator::vectorizeAccelBiases(vals,m_rthighImuPoseProblem.m_imuBiasKeys);
    m_rshankImuPose=imuPoseEstimator::vectorizePoses(vals,m_rshankImuPoseProblem.m_poseKeys);
    m_rshankImuOrientation=imuPoseEstimator::vectorizeOrientations(m_rshankImuPose);
    m_rshankImuPosition=imuPoseEstimator::vectorizePositions(m_rshankImuPose);
    m_rshankImuVelocity=imuPoseEstimator::vectorizeVelocities(vals,m_rshankImuPoseProblem.m_velKeys);
    m_rshankImuGyroBias=imuPoseEstimator::vectorizeGyroBiases(vals,m_rshankImuPoseProblem.m_imuBiasKeys);
    m_rshankImuAccelBias=imuPoseEstimator::vectorizeAccelBiases(vals,m_rshankImuPoseProblem.m_imuBiasKeys);
    m_rfootImuPose=imuPoseEstimator::vectorizePoses(vals,m_rfootImuPoseProblem.m_poseKeys);
    m_rfootImuOrientation=imuPoseEstimator::vectorizeOrientations(m_rfootImuPose);
    m_rfootImuPosition=imuPoseEstimator::vectorizePositions(m_rfootImuPose);
    m_rfootImuVelocity=imuPoseEstimator::vectorizeVelocities(vals,m_rfootImuPoseProblem.m_velKeys);
    m_rfootImuGyroBias=imuPoseEstimator::vectorizeGyroBiases(vals,m_rfootImuPoseProblem.m_imuBiasKeys);
    m_rfootImuAccelBias=imuPoseEstimator::vectorizeAccelBiases(vals,m_rfootImuPoseProblem.m_imuBiasKeys);
    m_lthighImuPose=imuPoseEstimator::vectorizePoses(vals,m_lthighImuPoseProblem.m_poseKeys);
    m_lthighImuOrientation=imuPoseEstimator::vectorizeOrientations(m_lthighImuPose);
    m_lthighImuPosition=imuPoseEstimator::vectorizePositions(m_lthighImuPose);
    m_lthighImuVelocity=imuPoseEstimator::vectorizeVelocities(vals,m_lthighImuPoseProblem.m_velKeys);
    m_lthighImuGyroBias=imuPoseEstimator::vectorizeGyroBiases(vals,m_lthighImuPoseProblem.m_imuBiasKeys);
    m_lthighImuAccelBias=imuPoseEstimator::vectorizeAccelBiases(vals,m_lthighImuPoseProblem.m_imuBiasKeys);
    m_lshankImuPose=imuPoseEstimator::vectorizePoses(vals,m_lshankImuPoseProblem.m_poseKeys);
    m_lshankImuOrientation=imuPoseEstimator::vectorizeOrientations(m_lshankImuPose);
    m_lshankImuPosition=imuPoseEstimator::vectorizePositions(m_lshankImuPose);
    m_lshankImuVelocity=imuPoseEstimator::vectorizeVelocities(vals,m_lshankImuPoseProblem.m_velKeys);
    m_lshankImuGyroBias=imuPoseEstimator::vectorizeGyroBiases(vals,m_lshankImuPoseProblem.m_imuBiasKeys);
    m_lshankImuAccelBias=imuPoseEstimator::vectorizeAccelBiases(vals,m_lshankImuPoseProblem.m_imuBiasKeys);
    m_lfootImuPose=imuPoseEstimator::vectorizePoses(vals,m_lfootImuPoseProblem.m_poseKeys);
    m_lfootImuOrientation=imuPoseEstimator::vectorizeOrientations(m_lfootImuPose);
    m_lfootImuPosition=imuPoseEstimator::vectorizePositions(m_lfootImuPose);
    m_lfootImuVelocity=imuPoseEstimator::vectorizeVelocities(vals,m_lfootImuPoseProblem.m_velKeys);
    m_lfootImuGyroBias=imuPoseEstimator::vectorizeGyroBiases(vals,m_lfootImuPoseProblem.m_imuBiasKeys);
    m_lfootImuAccelBias=imuPoseEstimator::vectorizeAccelBiases(vals,m_lfootImuPoseProblem.m_imuBiasKeys);
}

void lowerBodyPoseEstimator::defaultOptimize(){
    fastOptimize();
}

void lowerBodyPoseEstimator::robustOptimize() {
    throw std::runtime_error("method not implemented");
}

int lowerBodyPoseEstimator::testJointCenterVelocityAsDiscretePositionDerivative(double errorTol, bool printMatricesToCsv){
    // this is a debug method only!
    // used to test that the velocity of a joint according to an imu is roughly equivalent to the time derivative of its position
    // define the format you want, you only need one instance of this...
    const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
    // compute time series positions and velocities as a matrix
    uint nKeyframes=m_rthighImuPoseProblem.m_nKeyframes;
    Eigen::MatrixXd rhipPosThighImu(nKeyframes,3);
    Eigen::MatrixXd rhipVelThighImu(nKeyframes,3);
    for(uint k=0; k<nKeyframes; k++){
        rhipPosThighImu.block<1,3>(k,0)=(m_rthighImuPosition[k]+m_rthighImuOrientation[k].matrix()*m_rthighImuToHipCtr).transpose(); // joint pos = imu pos + imuOrientation*(vec to joint)
        rhipVelThighImu.block<1,3>(k,0)=(m_rthighImuVelocity[k] +m_rthighImuOrientation[k].matrix()*(m_rthighImuAngVel[k].cross(m_rthighImuToHipCtr)) ).transpose(); // joint vel = imu vel + imuOrientation*( cross(angVel,vecToJoint) )
    }
    // now get discrete derivative for velocity
    Eigen::MatrixXd rhipVelThighImu_DiscreteDiff(nKeyframes-1,3); // remember that it is one less row than position/velocity. when comparing, you should compare this matrix to position/velocity(2:end,:)
    double dt=m_rthighImuPoseProblem.m_time[1]-m_rthighImuPoseProblem.m_time[0];
    for(uint k=0; k<nKeyframes-1; k++){
        rhipVelThighImu_DiscreteDiff.block<1,3>(k,0)=(rhipPosThighImu.block<1,3>(k+1,0) - rhipPosThighImu.block<1,3>(k,0))/dt;
    }
    // now compare that discrete velocity to the theoretical velocity
    Eigen::MatrixXd rhipVelErr(nKeyframes-1,3);
    std::vector<double> rhipVelErrNorm(nKeyframes-1), rhipVelErrNormSq(nKeyframes-1) ;
    for(uint k=0; k<nKeyframes-1; k++){
        // remember to compare index k of discrete vel to index k+1 of theoretical vel
        rhipVelErr.block<1,3>(k,0) = rhipVelThighImu.block<1,3>(k,0) - rhipVelThighImu_DiscreteDiff.block<1,3>(k,0);
        rhipVelErrNorm[k]=(rhipVelErr.block<1,3>(k,0)).norm();
        rhipVelErrNormSq[k]=pow(rhipVelErrNorm[k],2.0);
    }
    // now get RMSE
    double rhipVelThighImuRmse = sqrt(accumulate(rhipVelErrNormSq.begin(),rhipVelErrNormSq.end(),0.0)/rhipVelErrNormSq.size());
    // optionally, print these data for convenient plotting in matlab
    if(printMatricesToCsv){
        gtsamutils::writeEigenMatrixToCsvFile("rhipPosThighImu",rhipPosThighImu,CSVFormat);
        gtsamutils::writeEigenMatrixToCsvFile("rhipVelThighImu",rhipVelThighImu,CSVFormat);
        gtsamutils::writeEigenMatrixToCsvFile("rhipVelThighImu_DiscreteDiff",rhipVelThighImu_DiscreteDiff,CSVFormat);
    }
    // do test
    if(rhipVelThighImuRmse>errorTol){
        std::cerr<<"high velocity error"<<std::endl;
        return 1;
    }
    return 0;
}

std::vector<double> lowerBodyPoseEstimator::getRKneeFlexExAngle() const {
    // get method to return the right knee's flexion/extension angle in radians
    std::vector<double> angle(m_rthighImuOrientation.size());
    std::vector<gtsam::Rot3> R_RFemur_to_N=bioutils::get_R_Segment_to_N(m_rthighImuOrientation,m_rkneeAxisThigh,m_rthighImuToHipCtr,m_rthighImuToKneeCtr);
    std::vector<gtsam::Rot3> R_RTibia_to_N=bioutils::get_R_Segment_to_N(m_rshankImuOrientation,m_rkneeAxisShank,m_rshankImuToKneeCtr,m_rshankImuToAnkleCtr);
    Eigen::MatrixXd RKneeClinicalAngles=bioutils::clinical3DofJcsAngles(R_RFemur_to_N,R_RTibia_to_N,true,false);
    return mathutils::EigenVectorToStdVector(RKneeClinicalAngles.col(0));
}

std::vector<double> lowerBodyPoseEstimator::getLKneeFlexExAngle() const {
    // get method to return the left knee's flexion/extension angle in radians
    std::vector<double> angle(m_lthighImuOrientation.size());
    std::vector<gtsam::Rot3> R_LFemur_to_N=bioutils::get_R_Segment_to_N(m_lthighImuOrientation,m_lkneeAxisThigh,m_lthighImuToHipCtr,m_lthighImuToKneeCtr);
    std::vector<gtsam::Rot3> R_LTibia_to_N=bioutils::get_R_Segment_to_N(m_lshankImuOrientation,m_lkneeAxisShank,m_lshankImuToKneeCtr,m_lshankImuToAnkleCtr);
    Eigen::MatrixXd LKneeClinicalAngles=bioutils::clinical3DofJcsAngles(R_LFemur_to_N,R_LTibia_to_N,false,false);
    return mathutils::EigenVectorToStdVector(LKneeClinicalAngles.col(0));
}

void lowerBodyPoseEstimator::addNewPriorSacrumImuToRHipCtr(const gtsam::Point3& newPriorValue, const double& newPriorStd){
    // clear previous priors and add new prior
    // useful if you've previously calibrated for this quantity
    // () check for any existing PriorFactor<Point3>
    gtsamutils::clearGtsamFactorByKey<gtsam::PriorFactor<gtsam::Point3>>(m_graph,m_sacrumImuToRHipCtrKey);
    // () construct new prior factor from input value and std
    gtsam::SharedNoiseModel priorNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(newPriorStd,newPriorStd,newPriorStd),true);
    gtsam::PriorFactor<gtsam::Point3> newPriorFactor(m_sacrumImuToRHipCtrKey,newPriorValue,priorNoiseModel);
    // () add new prior to factorgraph
    m_graph.push_back(newPriorFactor);
    // () update initial values
    m_initialValues.update(m_sacrumImuToRHipCtrKey,newPriorValue);
}

void lowerBodyPoseEstimator::addNewPriorSacrumImuToLHipCtr(const gtsam::Point3& newPriorValue, const double& newPriorStd){
    // clear previous priors and add new prior
    // useful if you've previously calibrated for this quantity
    // () check for any existing PriorFactor<Point3>
    gtsamutils::clearGtsamFactorByKey<gtsam::PriorFactor<gtsam::Point3>>(m_graph,m_sacrumImuToLHipCtrKey);
    // () construct new prior factor from input value and std
    gtsam::SharedNoiseModel priorNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(newPriorStd,newPriorStd,newPriorStd),true);
    gtsam::PriorFactor<gtsam::Point3> newPriorFactor(m_sacrumImuToLHipCtrKey,newPriorValue,priorNoiseModel);
    // () add new prior to factorgraph
    m_graph.push_back(newPriorFactor);
    // () update initial values
    m_initialValues.update(m_sacrumImuToLHipCtrKey,newPriorValue);
}

void lowerBodyPoseEstimator::addNewPriorRThighImuToHipCtr(const gtsam::Point3& newPriorValue, const double& newPriorStd){
    // clear previous priors and add new prior
    // useful if you've previously calibrated for this quantity
    // () check for any existing PriorFactor<Point3>
    gtsamutils::clearGtsamFactorByKey<gtsam::PriorFactor<gtsam::Point3>>(m_graph,m_rthighImuToHipCtrKey);
    // () construct new prior factor from input value and std
    gtsam::SharedNoiseModel priorNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(newPriorStd,newPriorStd,newPriorStd),true);
    gtsam::PriorFactor<gtsam::Point3> newPriorFactor(m_rthighImuToHipCtrKey,newPriorValue,priorNoiseModel);
    // () add new prior to factorgraph
    m_graph.push_back(newPriorFactor);
    // () update initial values
    m_initialValues.update(m_rthighImuToHipCtrKey,newPriorValue);
}

void lowerBodyPoseEstimator::addNewPriorLThighImuToHipCtr(const gtsam::Point3& newPriorValue, const double& newPriorStd){
    // clear previous priors and add new prior
    // useful if you've previously calibrated for this quantity
    // () check for any existing PriorFactor<Point3>
    gtsamutils::clearGtsamFactorByKey<gtsam::PriorFactor<gtsam::Point3>>(m_graph,m_lthighImuToHipCtrKey);
    // () construct new prior factor from input value and std
    gtsam::SharedNoiseModel priorNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(newPriorStd,newPriorStd,newPriorStd),true);
    gtsam::PriorFactor<gtsam::Point3> newPriorFactor(m_lthighImuToHipCtrKey,newPriorValue,priorNoiseModel);
    // () add new prior to factorgraph
    m_graph.push_back(newPriorFactor);
    // () update initial values
    m_initialValues.update(m_lthighImuToHipCtrKey,newPriorValue);
}

void lowerBodyPoseEstimator::addNewPoint3PriorToKey(const gtsam::Key& key, const gtsam::Point3& newPriorValue, const double& newPriorStd){
// clear previous priors and add new prior
    // useful if you've previously calibrated for this quantity
    // () check for any existing PriorFactor<Point3> and clear them
    gtsamutils::clearGtsamFactorByKey<gtsam::PriorFactor<gtsam::Point3>>(m_graph, key);
    // () construct new prior factor from input value and std
    gtsam::SharedNoiseModel priorNoiseModel=gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(newPriorStd,newPriorStd,newPriorStd),true);
    gtsam::PriorFactor<gtsam::Point3> newPriorFactor(key,newPriorValue,priorNoiseModel);
    // () add new prior to factorgraph
    m_graph.push_back(newPriorFactor);
    // () update initial values
    m_initialValues.update(key,newPriorValue);
}

bool lowerBodyPoseEstimator::testImusCanBeSpunWithOnlyChangeToIntExtRotAngle(const gtsam::NonlinearFactorGraph& origgraph, const gtsam::Values& origvals, bool verbose) const{
    // NOTE: the premise of this test has been refuted. Previously I thought for some reason that if you rotated a distal IMU in yaw, that it wouldn't change the flexion angle of the joint. This is untrue. I'm leaving this test here for posterity though.
    // make sure that you can randomly yaw IMUs, and only the joint's internal/external rotation angle will change.
    gtsam::Values vals=origvals;
    bool doesTestPass=true;
    // for now, let's start with the right hip
    // () get approximate vertical vector in sacrum frame
    gtsam::Vector3 approxUp(-1.0,0.0,0.0);
    // () get segment systems -> N orientation
    std::vector<gtsam::Rot3> R_Pelvis_to_N=bioutils::get_R_Pelvis_to_N(imuPoseEstimator::vectorizeOrientations(vals,m_sacrumImuPoseProblem.m_poseKeys),approxUp,vals.at<gtsam::Point3>(m_sacrumImuToRHipCtrKey),vals.at<gtsam::Point3>(m_sacrumImuToLHipCtrKey));
    std::vector<gtsam::Rot3> R_RFemur_to_N=bioutils::get_R_Segment_to_N(imuPoseEstimator::vectorizeOrientations(vals,m_rthighImuPoseProblem.m_poseKeys),vals.at<gtsam::Unit3>(m_rkneeAxisThighKey),vals.at<gtsam::Point3>(m_rthighImuToHipCtrKey),vals.at<gtsam::Point3>(m_rthighImuToKneeCtrKey));
    // () compute RHip Angles
    Eigen::MatrixXd origRHipAngles=bioutils::consistent3DofJcsAngles(R_Pelvis_to_N,R_RFemur_to_N); // returned as [flexion,adduction,externalRot]
    // get max/min of each joint angle
    double maxFlex=origRHipAngles.col(0).maxCoeff()*180.0/M_PI;
    double minFlex=origRHipAngles.col(0).minCoeff()*180.0/M_PI;
    double maxAdd=origRHipAngles.col(1).maxCoeff()*180.0/M_PI;
    double minAdd=origRHipAngles.col(1).minCoeff()*180.0/M_PI;
    double maxExt=origRHipAngles.col(2).maxCoeff()*180.0/M_PI;
    double minExt=origRHipAngles.col(2).minCoeff()*180.0/M_PI;
    if(verbose){std::cout<<"     pre iteration: (graph err="<<m_graph.error(vals)<<"): [max/min] of each angle: flexion: ["<<maxFlex<<","<<minFlex<<"] adduction: ["<<maxAdd<<","<<minAdd<<"] ext rot: ["<<maxExt<<","<<minExt<<"]"<<std::endl;}
    // () now loop through the values, update them, and test new angles
    uint numTries=20;
    double headingAngleDeltaRad=(2*M_PI)/numTries;
    for(uint k=0; k<numTries; k++){
        // updates values
        //imuPoseEstimator::yawImuStatesByConstantValueInPlace(vals, headingAngleDeltaRad, m_sacrumImuPoseProblem.m_poseKeys, m_sacrumImuPoseProblem.m_velKeys); // we only want to rotate the thigh IMU! if you rotate them both, nothing will change.
        imuPoseEstimator::yawImuStatesByConstantValueInPlace(vals, headingAngleDeltaRad, m_rthighImuPoseProblem.m_poseKeys, m_rthighImuPoseProblem.m_velKeys);
        // now compute orientations and joint angles
        R_Pelvis_to_N=bioutils::get_R_Pelvis_to_N(imuPoseEstimator::vectorizeOrientations(vals,m_sacrumImuPoseProblem.m_poseKeys),approxUp,vals.at<gtsam::Point3>(m_sacrumImuToRHipCtrKey),vals.at<gtsam::Point3>(m_sacrumImuToLHipCtrKey));
        R_RFemur_to_N=bioutils::get_R_Segment_to_N(imuPoseEstimator::vectorizeOrientations(vals,m_rthighImuPoseProblem.m_poseKeys),vals.at<gtsam::Unit3>(m_rkneeAxisThighKey),vals.at<gtsam::Point3>(m_rthighImuToHipCtrKey),vals.at<gtsam::Point3>(m_rthighImuToKneeCtrKey));
        // () compute RHip Angles
        Eigen::MatrixXd newRHipAngles=bioutils::consistent3DofJcsAngles(R_Pelvis_to_N,R_RFemur_to_N); // returned as [flexion,adduction,externalRot]
        // test: are these angles the same as origRHipAngles?
        bool isFlexionAngleEqual=newRHipAngles.col(0).isApprox(origRHipAngles.col(0));
        bool isAdductionAngleEqual=newRHipAngles.col(1).isApprox(origRHipAngles.col(1));
        bool isExternalRotAngleEqual=newRHipAngles.col(2).isApprox(origRHipAngles.col(2));
        if(!isFlexionAngleEqual){doesTestPass=false;}
        if(!isAdductionAngleEqual){doesTestPass=false;}
        //if(verbose){std::cout<<"     iteration "<<k<<" (graph err="<<m_graph.error(vals)<<"): isFlexionAngleEqual? "<<isFlexionAngleEqual<<" isAdductionAngleEqual? "<<isAdductionAngleEqual<<" isExternalRotAngleEqual? "<<isExternalRotAngleEqual<<" does test pass? "<<doesTestPass<<std::endl;}
        // get max/min of each joint angle
        maxFlex=newRHipAngles.col(0).maxCoeff()*180.0/M_PI;
        minFlex=newRHipAngles.col(0).minCoeff()*180.0/M_PI;
        maxAdd=newRHipAngles.col(1).maxCoeff()*180.0/M_PI;
        minAdd=newRHipAngles.col(1).minCoeff()*180.0/M_PI;
        maxExt=newRHipAngles.col(2).maxCoeff()*180.0/M_PI;
        minExt=newRHipAngles.col(2).minCoeff()*180.0/M_PI;
        if(verbose){std::cout<<"     iteration "<<k<<" (graph err="<<m_graph.error(vals)<<"): [max/min] of each angle: flexion: ["<<maxFlex<<","<<minFlex<<"] adduction: ["<<maxAdd<<","<<minAdd<<"] ext rot: ["<<maxExt<<","<<minExt<<"]"<<std::endl;}
    }
    if(verbose){std::cout<<"does test pass = "<<doesTestPass<<std::endl;}
    if(!doesTestPass){std::cerr<<"test failed (but remember: this test is wrong, we don't expect it to pass!)"<<std::endl;}
    return doesTestPass;
}

void lowerBodyPoseEstimator::printYawSlopesAndInterceptsFromMemberOrientations() const {
    // take the yaw angles of each IMU and print its slopes, for debugging purposes
    // note: I think the below unwrap function works well, but I've noticed that sometimes the yaw angle as computed by GTSAM doesn't seem to match up with what animations show. Further debugging needed.
    std::cout<<"slopes of each estimated IMU's yaw angle:"<<std::endl;
    std::cout << "    sacrum IMU yaw slope: " << 180.0 / M_PI * mathutils::slope(m_time, mathutils::unwrappedYawAngle(m_sacrumImuOrientation)) << " deg/s" << std::endl;
    std::cout << "    rthigh IMU yaw slope: " << 180.0 / M_PI * mathutils::slope(m_time, mathutils::unwrappedYawAngle(m_rthighImuOrientation)) << " deg/s" << std::endl;
    std::cout << "    rshank IMU yaw slope: " << 180.0 / M_PI * mathutils::slope(m_time, mathutils::unwrappedYawAngle(m_rshankImuOrientation)) << " deg/s" << std::endl;
    std::cout << "    rfoot IMU yaw slope: "  << 180.0 / M_PI * mathutils::slope(m_time, mathutils::unwrappedYawAngle(m_rfootImuOrientation)) << " deg/s" << std::endl;
    std::cout << "    lthigh IMU yaw slope: " << 180.0 / M_PI * mathutils::slope(m_time, mathutils::unwrappedYawAngle(m_lthighImuOrientation)) << " deg/s" << std::endl;
    std::cout << "    lshank IMU yaw slope: " << 180.0 / M_PI * mathutils::slope(m_time, mathutils::unwrappedYawAngle(m_lshankImuOrientation)) << " deg/s" << std::endl;
    std::cout << "    lfoot IMU yaw slope: "  << 180.0 / M_PI * mathutils::slope(m_time, mathutils::unwrappedYawAngle(m_lfootImuOrientation)) << " deg/s" << std::endl;
}

void lowerBodyPoseEstimator::alignYawAnglesOfAllImusToSacrumImu(gtsam::Values& vals){
    // this method corrects all IMU poses in input gtsam::Values vals, editing the values in place
    // it loops through all of the present poses, pulling out the rpy values, and then resetting them s.t. the new yaw value is the sacrum IMU yaw value
    // this is useful for running on the initial values before optimization occurs
    // first implemented Mar 19 2020, as it was determined that starting the optimization with really drifted/separated yaw values between the 7 IMUs could stop convergence.
    std::cout<<"     aligning all yaws in initial values to sacrum IMU... ";
    double origGraphErr=m_graph.error(vals);
    double beginTic=clock();
    uint nKeyframes=m_rthighImuPoseProblem.m_poseKeys.size();
    Eigen::MatrixXd sacrumImuYpr(nKeyframes,3), rthighImuYpr(nKeyframes,3), rshankImuYpr(nKeyframes,3), rfootImuYpr(nKeyframes,3), lthighImuYpr(nKeyframes,3), lshankImuYpr(nKeyframes,3), lfootImuYpr(nKeyframes,3);
    for(uint k=0; k<nKeyframes;k++){
        // get blocks of ypr
        sacrumImuYpr.block<1,3>(k,0)=(vals.at<gtsam::Pose3>(m_sacrumImuPoseProblem.m_poseKeys[k])).rotation().ypr().transpose();
        rthighImuYpr.block<1,3>(k,0)=(vals.at<gtsam::Pose3>(m_rthighImuPoseProblem.m_poseKeys[k])).rotation().ypr().transpose();
        rshankImuYpr.block<1,3>(k,0)=(vals.at<gtsam::Pose3>(m_rshankImuPoseProblem.m_poseKeys[k])).rotation().ypr().transpose();
        rfootImuYpr.block<1,3>(k,0)=(vals.at<gtsam::Pose3>(m_rfootImuPoseProblem.m_poseKeys[k])).rotation().ypr().transpose();
        lthighImuYpr.block<1,3>(k,0)=(vals.at<gtsam::Pose3>(m_lthighImuPoseProblem.m_poseKeys[k])).rotation().ypr().transpose();
        lshankImuYpr.block<1,3>(k,0)=(vals.at<gtsam::Pose3>(m_lshankImuPoseProblem.m_poseKeys[k])).rotation().ypr().transpose();
        lfootImuYpr.block<1,3>(k,0)=(vals.at<gtsam::Pose3>(m_lfootImuPoseProblem.m_poseKeys[k])).rotation().ypr().transpose();
        //std::cout<<"sacrumImuYpr.block<1,3>(k,0)=["<<sacrumImuYpr.block<1,3>(k,0)<<"]"<<std::endl;
        // construct new Pose3 with the update yaw of the sacrum IMU
        gtsam::Pose3 rthighImuNewPose=gtsam::Pose3(gtsam::Rot3::Ypr(sacrumImuYpr(k,0),rthighImuYpr(k,1),rthighImuYpr(k,2)),(vals.at<gtsam::Pose3>(m_rthighImuPoseProblem.m_poseKeys[k])).translation());
        gtsam::Pose3 rshankImuNewPose=gtsam::Pose3(gtsam::Rot3::Ypr(sacrumImuYpr(k,0),rshankImuYpr(k,1),rshankImuYpr(k,2)),(vals.at<gtsam::Pose3>(m_rshankImuPoseProblem.m_poseKeys[k])).translation());
        gtsam::Pose3 rfootImuNewPose=gtsam::Pose3(gtsam::Rot3::Ypr(sacrumImuYpr(k,0),rfootImuYpr(k,1),rfootImuYpr(k,2)),(vals.at<gtsam::Pose3>(m_rfootImuPoseProblem.m_poseKeys[k])).translation());
        gtsam::Pose3 lthighImuNewPose=gtsam::Pose3(gtsam::Rot3::Ypr(sacrumImuYpr(k,0),lthighImuYpr(k,1),lthighImuYpr(k,2)),(vals.at<gtsam::Pose3>(m_lthighImuPoseProblem.m_poseKeys[k])).translation());
        gtsam::Pose3 lshankImuNewPose=gtsam::Pose3(gtsam::Rot3::Ypr(sacrumImuYpr(k,0),lshankImuYpr(k,1),lshankImuYpr(k,2)),(vals.at<gtsam::Pose3>(m_lshankImuPoseProblem.m_poseKeys[k])).translation());
        gtsam::Pose3 lfootImuNewPose=gtsam::Pose3(gtsam::Rot3::Ypr(sacrumImuYpr(k,0),lfootImuYpr(k,1),lfootImuYpr(k,2)),(vals.at<gtsam::Pose3>(m_lfootImuPoseProblem.m_poseKeys[k])).translation());
        // now take these new poses and replace the old ones in vals
        vals.update(m_rthighImuPoseProblem.m_poseKeys[k],rthighImuNewPose);
        vals.update(m_rshankImuPoseProblem.m_poseKeys[k],rshankImuNewPose);
        vals.update(m_rfootImuPoseProblem.m_poseKeys[k],rfootImuNewPose);
        vals.update(m_lthighImuPoseProblem.m_poseKeys[k],lthighImuNewPose);
        vals.update(m_lshankImuPoseProblem.m_poseKeys[k],lshankImuNewPose);
        vals.update(m_lfootImuPoseProblem.m_poseKeys[k],lfootImuNewPose);
    }
    double newGraphErr=m_graph.error(vals);
    std::cout<<" complete! ("<<(clock()-beginTic)/CLOCKS_PER_SEC<<" sec)   graph error change: "<<origGraphErr<<" --> "<<newGraphErr<<std::endl;
}

void lowerBodyPoseEstimator::setAllImuVelocitiesAsImuPositionDerivatives(gtsam::Values& vals){
    // simply reset all velocities as discrete position derivatives
    const double dt=m_sacrumImuPoseProblem.m_imu.getDeltaT();
    // () pull out position arrays
    std::vector<gtsam::Point3> sacrumImuPos=imuPoseEstimator::vectorizePositions(vals,m_sacrumImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Point3> rthighImuPos=imuPoseEstimator::vectorizePositions(vals,m_rthighImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Point3> rshankImuPos=imuPoseEstimator::vectorizePositions(vals,m_rshankImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Point3> rfootImuPos=imuPoseEstimator::vectorizePositions(vals,m_rfootImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Point3> lthighImuPos=imuPoseEstimator::vectorizePositions(vals,m_lthighImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Point3> lshankImuPos=imuPoseEstimator::vectorizePositions(vals,m_lshankImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Point3> lfootImuPos=imuPoseEstimator::vectorizePositions(vals,m_lfootImuPoseProblem.m_poseKeys);
    // () find associated velocity derivatives
    std::vector<gtsam::Vector3> sacrumImuVel=imuPoseEstimator::consistentVelocitySetFromPositions(sacrumImuPos, dt);
    std::vector<gtsam::Vector3> rthighImuVel=imuPoseEstimator::consistentVelocitySetFromPositions(rthighImuPos, dt);
    std::vector<gtsam::Vector3> rshankImuVel=imuPoseEstimator::consistentVelocitySetFromPositions(rshankImuPos, dt);
    std::vector<gtsam::Vector3> rfootImuVel=imuPoseEstimator::consistentVelocitySetFromPositions(rfootImuPos, dt);
    std::vector<gtsam::Vector3> lthighImuVel=imuPoseEstimator::consistentVelocitySetFromPositions(lthighImuPos, dt);
    std::vector<gtsam::Vector3> lshankImuVel=imuPoseEstimator::consistentVelocitySetFromPositions(lshankImuPos, dt);
    std::vector<gtsam::Vector3> lfootImuVel=imuPoseEstimator::consistentVelocitySetFromPositions(lfootImuPos, dt);
    // () replace these velocities in the values
    for(uint k=0; k<m_sacrumImuPoseProblem.m_poseKeys.size(); k++){
        vals.update(m_sacrumImuPoseProblem.m_velKeys[k],sacrumImuVel[k]);
        vals.update(m_rthighImuPoseProblem.m_velKeys[k],rthighImuVel[k]);
        vals.update(m_rshankImuPoseProblem.m_velKeys[k],rshankImuVel[k]);
        vals.update(m_rfootImuPoseProblem.m_velKeys[k],rfootImuVel[k]);
        vals.update(m_lthighImuPoseProblem.m_velKeys[k],lthighImuVel[k]);
        vals.update(m_lshankImuPoseProblem.m_velKeys[k],lshankImuVel[k]);
        vals.update(m_lfootImuPoseProblem.m_velKeys[k],lfootImuVel[k]);
    }
}

void lowerBodyPoseEstimator::computeHipIeRotAngleRegressionSlopesFromValues(const gtsam::Values& vals, double& rhipIntExtRotSlope, double& lhipIntExtRotSlope){
    // () for convenience, pull out static vectors into variables
    gtsam::Point3 sacrumImuToRHipCtr=vals.at<gtsam::Point3>(m_sacrumImuToRHipCtrKey);
    gtsam::Point3 rthighImuToHipCtr=vals.at<gtsam::Point3>(m_rthighImuToHipCtrKey);
    gtsam::Point3 rthighImuToKneeCtr=vals.at<gtsam::Point3>(m_rthighImuToKneeCtrKey);
    gtsam::Point3 rshankImuToKneeCtr=vals.at<gtsam::Point3>(m_rshankImuToKneeCtrKey);
    gtsam::Point3 rshankImuToAnkleCtr=vals.at<gtsam::Point3>(m_rshankImuToAnkleCtrKey);
    gtsam::Point3 sacrumImuToLHipCtr=vals.at<gtsam::Point3>(m_sacrumImuToLHipCtrKey);
    gtsam::Point3 lthighImuToHipCtr=vals.at<gtsam::Point3>(m_lthighImuToHipCtrKey);
    gtsam::Point3 lthighImuToKneeCtr=vals.at<gtsam::Point3>(m_lthighImuToKneeCtrKey);
    gtsam::Point3 lshankImuToKneeCtr=vals.at<gtsam::Point3>(m_lshankImuToKneeCtrKey);
    gtsam::Point3 lshankImuToAnkleCtr=vals.at<gtsam::Point3>(m_lshankImuToAnkleCtrKey);
    gtsam::Unit3 rkneeAxisThigh=vals.at<gtsam::Unit3>(m_rkneeAxisThighKey);
    gtsam::Unit3 rkneeAxisShank=vals.at<gtsam::Unit3>(m_rkneeAxisShankKey);
    gtsam::Unit3 lkneeAxisThigh=vals.at<gtsam::Unit3>(m_lkneeAxisThighKey);
    gtsam::Unit3 lkneeAxisShank=vals.at<gtsam::Unit3>(m_lkneeAxisShankKey);
    // get sacrum imu pose, it won't be changing. also get other poses and vals
    const std::vector<gtsam::Pose3> sacrumImuPose=imuPoseEstimator::vectorizePoses(vals,m_sacrumImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> rthighImuPose=imuPoseEstimator::vectorizePoses(vals,m_rthighImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> rshankImuPose=imuPoseEstimator::vectorizePoses(vals,m_rshankImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> rfootImuPose=imuPoseEstimator::vectorizePoses(vals,m_rfootImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> lthighImuPose=imuPoseEstimator::vectorizePoses(vals,m_lthighImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> lshankImuPose=imuPoseEstimator::vectorizePoses(vals,m_lshankImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> lfootImuPose=imuPoseEstimator::vectorizePoses(vals,m_lfootImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Vector3> rthighImuVel=imuPoseEstimator::vectorizeVelocities(vals,m_rthighImuPoseProblem.m_velKeys);
    std::vector<gtsam::Vector3> rshankImuVel=imuPoseEstimator::vectorizeVelocities(vals,m_rshankImuPoseProblem.m_velKeys);
    std::vector<gtsam::Vector3> rfootImuVel=imuPoseEstimator::vectorizeVelocities(vals,m_rfootImuPoseProblem.m_velKeys);
    std::vector<gtsam::Vector3> lthighImuVel=imuPoseEstimator::vectorizeVelocities(vals,m_lthighImuPoseProblem.m_velKeys);
    std::vector<gtsam::Vector3> lshankImuVel=imuPoseEstimator::vectorizeVelocities(vals,m_lshankImuPoseProblem.m_velKeys);
    std::vector<gtsam::Vector3> lfootImuVel=imuPoseEstimator::vectorizeVelocities(vals,m_lfootImuPoseProblem.m_velKeys);
    // () compute R[Pelvis->N], which will be const
    const std::vector<gtsam::Rot3> R_Pelvis_to_N=bioutils::get_R_Pelvis_to_N(gtsamutils::Pose3VectorToRot3Vector(sacrumImuPose),Eigen::Vector3d(-1.0,0.0,0.0),sacrumImuToRHipCtr,sacrumImuToLHipCtr);
    const gtsam::Unit3 pelvisRightAxis=gtsam::Unit3(sacrumImuToRHipCtr-sacrumImuToLHipCtr);
    // () compute both current R[Femur->N]
    std::vector<gtsam::Rot3> R_RFemur_to_N=bioutils::get_R_Segment_to_N(gtsamutils::Pose3VectorToRot3Vector(rthighImuPose), rkneeAxisThigh, rthighImuToHipCtr, rthighImuToKneeCtr,true);
    std::vector<gtsam::Rot3> R_LFemur_to_N=bioutils::get_R_Segment_to_N(gtsamutils::Pose3VectorToRot3Vector(lthighImuPose), lkneeAxisThigh, lthighImuToHipCtr, lthighImuToKneeCtr,true);
    // () compute both hip angles
    Eigen::MatrixXd RHipAngles=bioutils::clinical3DofJcsAngles(R_Pelvis_to_N, R_RFemur_to_N, true, true, false);
    Eigen::MatrixXd LHipAngles=bioutils::clinical3DofJcsAngles(R_Pelvis_to_N, R_LFemur_to_N, false, true, false);
    // () pull out just the internal/external rotation angle component
    Eigen::VectorXd RHipIeAngle=RHipAngles.col(2), LHipIeAngle=LHipAngles.col(2);
    // () handle any unwrapping of the I/E angle
    //angleUnwrapOnKnownDomain(const Eigen::VectorXd& angle, const double& domainMin=-1.0*M_PI, const double& domainMax=M_PI, const double& tol=2*M_PI*0.95);
    RHipIeAngle=mathutils::angleUnwrapOnKnownDomain(RHipIeAngle, -1.0*M_PI, M_PI,0.95 * 2.0 * M_PI); // int/ext rot is on [-pi,pi], so let's say jump tol is 0.95*2*M_PI
    LHipIeAngle=mathutils::angleUnwrapOnKnownDomain(LHipIeAngle, -1.0*M_PI, M_PI,0.95 * 2.0 * M_PI); // int/ext rot is on [-pi,pi], so let's say jump tol is 0.95*2*M_PI
    // () calculate slopes
    double rhipIeSlope, lhipIeSlope, rhipIeInt, lhipIeInt;
    mathutils::linearRegression(mathutils::StdVectorToEigenVector(m_time),RHipIeAngle,rhipIeSlope,rhipIeInt);
    mathutils::linearRegression(mathutils::StdVectorToEigenVector(m_time),LHipIeAngle,lhipIeSlope,lhipIeInt);
    // write those values
    rhipIntExtRotSlope=rhipIeSlope, lhipIntExtRotSlope=lhipIeSlope;
}

void lowerBodyPoseEstimator::adjustLegsForCorrectedHipIeAngles(gtsam::Values& vals, bool verbose){
    if(verbose){std::cout<<"\tcorrecting hip angles for zero hip i/e angle slope..."<<std::endl;}
    // () for convenience, pull out static vectors into variables
    gtsam::Point3 sacrumImuToRHipCtr=vals.at<gtsam::Point3>(m_sacrumImuToRHipCtrKey), sacrumImuToLHipCtr=vals.at<gtsam::Point3>(m_sacrumImuToLHipCtrKey);
    gtsam::Point3 rthighImuToHipCtr=vals.at<gtsam::Point3>(m_rthighImuToHipCtrKey), rthighImuToKneeCtr=vals.at<gtsam::Point3>(m_rthighImuToKneeCtrKey);
    gtsam::Point3 rshankImuToKneeCtr=vals.at<gtsam::Point3>(m_rshankImuToKneeCtrKey), rshankImuToAnkleCtr=vals.at<gtsam::Point3>(m_rshankImuToAnkleCtrKey);
    gtsam::Point3 lthighImuToHipCtr=vals.at<gtsam::Point3>(m_lthighImuToHipCtrKey), lthighImuToKneeCtr=vals.at<gtsam::Point3>(m_lthighImuToKneeCtrKey);
    gtsam::Point3 lshankImuToKneeCtr=vals.at<gtsam::Point3>(m_lshankImuToKneeCtrKey), lshankImuToAnkleCtr=vals.at<gtsam::Point3>(m_lshankImuToAnkleCtrKey);
    gtsam::Point3 rfootImuToAnkleCtr=vals.at<gtsam::Point3>(m_rfootImuToAnkleCtrKey), lfootImuToAnkleCtr=vals.at<gtsam::Point3>(m_lfootImuToAnkleCtrKey);
    gtsam::Unit3 rkneeAxisThigh=vals.at<gtsam::Unit3>(m_rkneeAxisThighKey), rkneeAxisShank=vals.at<gtsam::Unit3>(m_rkneeAxisShankKey);
    gtsam::Unit3 lkneeAxisThigh=vals.at<gtsam::Unit3>(m_lkneeAxisThighKey), lkneeAxisShank=vals.at<gtsam::Unit3>(m_lkneeAxisShankKey);
    // get sacrum imu pose, it won't be changing. also get other poses and vals
    const std::vector<gtsam::Pose3> sacrumImuPose=imuPoseEstimator::vectorizePoses(vals,m_sacrumImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> rthighImuPose=imuPoseEstimator::vectorizePoses(vals,m_rthighImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> rshankImuPose=imuPoseEstimator::vectorizePoses(vals,m_rshankImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> rfootImuPose=imuPoseEstimator::vectorizePoses(vals,m_rfootImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> lthighImuPose=imuPoseEstimator::vectorizePoses(vals,m_lthighImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> lshankImuPose=imuPoseEstimator::vectorizePoses(vals,m_lshankImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> lfootImuPose=imuPoseEstimator::vectorizePoses(vals,m_lfootImuPoseProblem.m_poseKeys);
    // () compute R[Pelvis->N], which will be const
    const std::vector<gtsam::Rot3> R_Pelvis_to_N=bioutils::get_R_Pelvis_to_N(gtsamutils::Pose3VectorToRot3Vector(sacrumImuPose),Eigen::Vector3d(-1.0,0.0,0.0),sacrumImuToRHipCtr,sacrumImuToLHipCtr);
    const gtsam::Unit3 pelvisRightAxis=gtsam::Unit3(sacrumImuToRHipCtr-sacrumImuToLHipCtr);
    // () compute both current R[Femur->N]
    std::vector<gtsam::Rot3> R_RFemur_to_N=bioutils::get_R_Segment_to_N(gtsamutils::Pose3VectorToRot3Vector(rthighImuPose), rkneeAxisThigh, rthighImuToHipCtr, rthighImuToKneeCtr,true);
    std::vector<gtsam::Rot3> R_LFemur_to_N=bioutils::get_R_Segment_to_N(gtsamutils::Pose3VectorToRot3Vector(lthighImuPose), lkneeAxisThigh, lthighImuToHipCtr, lthighImuToKneeCtr,true);
    // () compute both hip angles
    Eigen::MatrixXd RHipAngles=bioutils::clinical3DofJcsAngles(R_Pelvis_to_N, R_RFemur_to_N, true, true, false);
    Eigen::MatrixXd LHipAngles=bioutils::clinical3DofJcsAngles(R_Pelvis_to_N, R_LFemur_to_N, false, true, false);
    // () pull out just the internal/external rotation angle component
    Eigen::VectorXd RHipIeAngle=RHipAngles.col(2), LHipIeAngle=LHipAngles.col(2);
    // () handle any unwrapping of the I/E angle
    //angleUnwrapOnKnownDomain(const Eigen::VectorXd& angle, const double& domainMin=-1.0*M_PI, const double& domainMax=M_PI, const double& tol=2*M_PI*0.95);
    RHipIeAngle=mathutils::angleUnwrapOnKnownDomain(RHipIeAngle, -1.0*M_PI, M_PI,0.95 * 2.0 * M_PI); // int/ext rot is on [-pi,pi], so let's say jump tol is 0.95*2*M_PI
    LHipIeAngle=mathutils::angleUnwrapOnKnownDomain(LHipIeAngle, -1.0*M_PI, M_PI,0.95 * 2.0 * M_PI); // int/ext rot is on [-pi,pi], so let's say jump tol is 0.95*2*M_PI
    // () compute new angles which have been adjusted to have regression slope = 0
    Eigen::VectorXd newRHipIeAng=mathutils::adjustRegressionSlopeToTargetSlope(mathutils::StdVectorToEigenVector(m_time), RHipIeAngle, 0.0);
    Eigen::VectorXd newLHipIeAng=mathutils::adjustRegressionSlopeToTargetSlope(mathutils::StdVectorToEigenVector(m_time), LHipIeAngle, 0.0);
    // () adjust angles so intercept is zero (maybe I should figure out something better for this)
    newRHipIeAng=newRHipIeAng.array()-newRHipIeAng[0], newLHipIeAng=newLHipIeAng.array()-newLHipIeAng[0];

    // () find the rotation change R[B->B2] for the thigh IMU which would make this new angle realized
    // right leg
    for(uint k=0; k<m_time.size(); k++){
        // get correction orientation
        gtsam::Rot3 R_B_to_B2 = lowerBodyPoseEstimator::getDistalImuOrientationAdjustmentGivenDesiredIntExtRotAngle(R_Pelvis_to_N[k],rthighImuPose[k].rotation(),newRHipIeAng[k],rkneeAxisThigh,rthighImuToHipCtr,rthighImuToKneeCtr); // R[B->B2]
        // () determine difference in hip I/E angle--how much do we need to change hip I/E by to correct it?
        double rhipAdj=newRHipIeAng[k]-RHipIeAngle[k], lhipAdj=newLHipIeAng[k]-LHipIeAngle[k]; // change = new - old
        //double rhipAdj=60.0*M_PI/180.0, lhipAdj=60.0*M_PI/180.0;
        // () find vector to rotate leg about. Angle is positive by right hand rule. So adjust right hip by proximal femur vector, left hip by distal femur vector.
        gtsam::Vector3 rthighRotVecNav=-1.0*rthighImuPose[k].rotation().rotate(rthighImuToHipCtr - rthighImuToKneeCtr), lthighRotVecNav= lthighImuPose[k].rotation().rotate(lthighImuToHipCtr - lthighImuToKneeCtr);
        //gtsam::Vector3 rthighRotVecNav(0.0,0.0,1.0), lthighRotVecNav(0.0,0.0,-1.0);
        gtsam::Point3 rthighRotPtNav=rthighImuPose[k].translation()+rthighImuPose[k].rotation().rotate(rthighImuToHipCtr); // should be equivalent to: rthighImuPose[k].transformFrom(rthighImuToHipCtr);
        gtsam::Point3 lthighRotPtNav=lthighImuPose[k].translation()+lthighImuPose[k].rotation().rotate(lthighImuToHipCtr); // should be equivalent to: lthighImuPose[k].transformFrom(lthighImuToHipCtr);
        gtsam::Vector3 rshankRotVecNav=rshankImuPose[k].rotation().rotate(rshankImuToKneeCtr - rshankImuToAnkleCtr), lshankRotVecNav= -1.0 * lshankImuPose[k].rotation().rotate(lshankImuToKneeCtr - lshankImuToAnkleCtr);
        gtsam::Point3 rshankRotPtNav=rshankImuPose[k].translation()+rshankImuPose[k].rotation().rotate(rshankImuToKneeCtr);
        gtsam::Point3 lshankRotPtNav=lshankImuPose[k].translation()+lshankImuPose[k].rotation().rotate(lshankImuToKneeCtr);
        gtsam::Point3 rfootRotPtNav=rfootImuPose[k].translation()+rfootImuPose[k].rotation().rotate(rfootImuToAnkleCtr);
        gtsam::Point3 lfootRotPtNav=lfootImuPose[k].translation()+lfootImuPose[k].rotation().rotate(lfootImuToAnkleCtr);
        /*
        // apply to right leg IMUs
        mathutils::rotateImuPoseAboutPointAxisAngle(rthighImuPose[k], rthighRotPtNav, rthighRotVecNav, rhipAdj);
        mathutils::rotateImuPoseAboutPointAxisAngle(rshankImuPose[k], rthighRotPtNav, rthighRotVecNav, rhipAdj);
        mathutils::rotateImuPoseAboutPointAxisAngle(rfootImuPose[k], rthighRotPtNav, rthighRotVecNav, rhipAdj);
        // apply to left leg IMUs
        mathutils::rotateImuPoseAboutPointAxisAngle(lthighImuPose[k],lthighRotPtNav,lthighRotVecNav,lhipAdj);
        mathutils::rotateImuPoseAboutPointAxisAngle(lshankImuPose[k],lthighRotPtNav,lthighRotVecNav,lhipAdj);
        mathutils::rotateImuPoseAboutPointAxisAngle(lfootImuPose[k],lthighRotPtNav,lthighRotVecNav,lhipAdj);
         */

        // apply to right leg IMUs
        mathutils::rotateImuPoseAboutPointAxisAngle(rthighImuPose[k], rthighRotPtNav, rthighRotVecNav, rhipAdj);
        mathutils::rotateImuPoseAboutPointAxisAngle(rshankImuPose[k], rthighRotPtNav, rthighRotVecNav, rhipAdj);
        mathutils::rotateImuPoseAboutPointAxisAngle(rfootImuPose[k], rthighRotPtNav, rthighRotVecNav, rhipAdj);
        // apply to left leg IMUs
        mathutils::rotateImuPoseAboutPointAxisAngle(lthighImuPose[k],lthighRotPtNav,lthighRotVecNav,lhipAdj);
        mathutils::rotateImuPoseAboutPointAxisAngle(lshankImuPose[k],lthighRotPtNav,lthighRotVecNav,lhipAdj);
        mathutils::rotateImuPoseAboutPointAxisAngle(lfootImuPose[k],lthighRotPtNav,lthighRotVecNav,lhipAdj);
        // () now, adjust positions so that IMUs connect at joints again
        lowerBodyPoseEstimator::correctImuPosesForConnectedJointCenters(sacrumImuPose[k],rthighImuPose[k],rshankImuPose[k],rfootImuPose[k],sacrumImuToRHipCtr,rthighImuToHipCtr,rthighImuToKneeCtr,rshankImuToKneeCtr,rshankImuToAnkleCtr,rfootImuToAnkleCtr);
        lowerBodyPoseEstimator::correctImuPosesForConnectedJointCenters(sacrumImuPose[k],lthighImuPose[k],lshankImuPose[k],lfootImuPose[k],sacrumImuToLHipCtr,lthighImuToHipCtr,lthighImuToKneeCtr,lshankImuToKneeCtr,lshankImuToAnkleCtr,lfootImuToAnkleCtr);
    }

    // () put these new poses back into the values
    for(uint k=0; k<m_sacrumImuPoseProblem.m_poseKeys.size();k++){
        vals.update(m_rthighImuPoseProblem.m_poseKeys[k],rthighImuPose[k]);
        vals.update(m_rshankImuPoseProblem.m_poseKeys[k],rshankImuPose[k]);
        vals.update(m_rfootImuPoseProblem.m_poseKeys[k],rfootImuPose[k]);
        vals.update(m_lthighImuPoseProblem.m_poseKeys[k],lthighImuPose[k]);
        vals.update(m_lshankImuPoseProblem.m_poseKeys[k],lshankImuPose[k]);
        vals.update(m_lfootImuPoseProblem.m_poseKeys[k],lfootImuPose[k]);
    }
}

void lowerBodyPoseEstimator::correctImuPosesForConnectedJointCenters(const gtsam::Pose3& sacrumImuPose, gtsam::Pose3& thighImuPose, gtsam::Pose3& shankImuPose, gtsam::Pose3& footImuPose,
                                                    const gtsam::Point3& sacrumImuToHipCtr, const gtsam::Point3& thighImuToHipCtr, const gtsam::Point3& thighImuToKneeCtr,
                                                    const gtsam::Point3& shankImuToKneeCtr, const gtsam::Point3& shankImuToAnkleCtr, const gtsam::Point3& footImuToAnkleCtr){
    // edits poses in place. works down from sacrum imu to connect all joint centers.
    thighImuPose=gtsam::Pose3(thighImuPose.rotation(),thighImuPose.translation()+(sacrumImuPose.transformFrom(sacrumImuToHipCtr)-thighImuPose.transformFrom(thighImuToHipCtr))); // newp = oldp + d where d = rhip ctr according to sacrum - rhip ctr according to thigh IMU
    shankImuPose=gtsam::Pose3(shankImuPose.rotation(),shankImuPose.translation()+(thighImuPose.transformFrom(thighImuToKneeCtr)-shankImuPose.transformFrom(shankImuToKneeCtr))); // newp = oldp + d where d is (knee from thigh - knee from shank)
    footImuPose=gtsam::Pose3(footImuPose.rotation(),footImuPose.translation()+(shankImuPose.transformFrom(shankImuToAnkleCtr)-footImuPose.transformFrom(footImuToAnkleCtr)));
}

void lowerBodyPoseEstimator::correctImuOrientationsForZeroHipIeSlope(gtsam::Values& vals, bool verbose, uint iterNum){
    // intuition: in many tasks (e.g., walking), there should be no drift of hip internal/external rotation angle
    // this method corrects thigh (+ distal) IMU orientations' yaw angles so that slope of the hip I/E angles is zero
    if(verbose){std::cout<<"\tcorrecting hip angles for zero hip i/e angle slope..."<<std::endl;}
    // () for convenience, pull out static vectors into variables
    gtsam::Point3 sacrumImuToRHipCtr=vals.at<gtsam::Point3>(m_sacrumImuToRHipCtrKey), sacrumImuToLHipCtr=vals.at<gtsam::Point3>(m_sacrumImuToLHipCtrKey);
    gtsam::Point3 rthighImuToHipCtr=vals.at<gtsam::Point3>(m_rthighImuToHipCtrKey), rthighImuToKneeCtr=vals.at<gtsam::Point3>(m_rthighImuToKneeCtrKey);
    gtsam::Point3 rshankImuToKneeCtr=vals.at<gtsam::Point3>(m_rshankImuToKneeCtrKey), rshankImuToAnkleCtr=vals.at<gtsam::Point3>(m_rshankImuToAnkleCtrKey);
    gtsam::Point3 lthighImuToHipCtr=vals.at<gtsam::Point3>(m_lthighImuToHipCtrKey), lthighImuToKneeCtr=vals.at<gtsam::Point3>(m_lthighImuToKneeCtrKey);
    gtsam::Point3 lshankImuToKneeCtr=vals.at<gtsam::Point3>(m_lshankImuToKneeCtrKey), lshankImuToAnkleCtr=vals.at<gtsam::Point3>(m_lshankImuToAnkleCtrKey);
    gtsam::Unit3 rkneeAxisThigh=vals.at<gtsam::Unit3>(m_rkneeAxisThighKey), rkneeAxisShank=vals.at<gtsam::Unit3>(m_rkneeAxisShankKey);
    gtsam::Unit3 lkneeAxisThigh=vals.at<gtsam::Unit3>(m_lkneeAxisThighKey), lkneeAxisShank=vals.at<gtsam::Unit3>(m_lkneeAxisShankKey);
    // get sacrum imu pose, it won't be changing. also get other poses and vals
    const std::vector<gtsam::Pose3> sacrumImuPose=imuPoseEstimator::vectorizePoses(vals,m_sacrumImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> rthighImuPose=imuPoseEstimator::vectorizePoses(vals,m_rthighImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> rshankImuPose=imuPoseEstimator::vectorizePoses(vals,m_rshankImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> rfootImuPose=imuPoseEstimator::vectorizePoses(vals,m_rfootImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> lthighImuPose=imuPoseEstimator::vectorizePoses(vals,m_lthighImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> lshankImuPose=imuPoseEstimator::vectorizePoses(vals,m_lshankImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> lfootImuPose=imuPoseEstimator::vectorizePoses(vals,m_lfootImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Vector3> rthighImuVel=imuPoseEstimator::vectorizeVelocities(vals,m_rthighImuPoseProblem.m_velKeys);
    std::vector<gtsam::Vector3> rshankImuVel=imuPoseEstimator::vectorizeVelocities(vals,m_rshankImuPoseProblem.m_velKeys);
    std::vector<gtsam::Vector3> rfootImuVel=imuPoseEstimator::vectorizeVelocities(vals,m_rfootImuPoseProblem.m_velKeys);
    std::vector<gtsam::Vector3> lthighImuVel=imuPoseEstimator::vectorizeVelocities(vals,m_lthighImuPoseProblem.m_velKeys);
    std::vector<gtsam::Vector3> lshankImuVel=imuPoseEstimator::vectorizeVelocities(vals,m_lshankImuPoseProblem.m_velKeys);
    std::vector<gtsam::Vector3> lfootImuVel=imuPoseEstimator::vectorizeVelocities(vals,m_lfootImuPoseProblem.m_velKeys);
    // () compute R[Pelvis->N], which will be const
    const std::vector<gtsam::Rot3> R_Pelvis_to_N=bioutils::get_R_Pelvis_to_N(gtsamutils::Pose3VectorToRot3Vector(sacrumImuPose),Eigen::Vector3d(-1.0,0.0,0.0),sacrumImuToRHipCtr,sacrumImuToLHipCtr);
    const gtsam::Unit3 pelvisRightAxis=gtsam::Unit3(sacrumImuToRHipCtr-sacrumImuToLHipCtr);
    // () compute both current R[Femur->N]
    std::vector<gtsam::Rot3> R_RFemur_to_N=bioutils::get_R_Segment_to_N(gtsamutils::Pose3VectorToRot3Vector(rthighImuPose), rkneeAxisThigh, rthighImuToHipCtr, rthighImuToKneeCtr,true);
    std::vector<gtsam::Rot3> R_LFemur_to_N=bioutils::get_R_Segment_to_N(gtsamutils::Pose3VectorToRot3Vector(lthighImuPose), lkneeAxisThigh, lthighImuToHipCtr, lthighImuToKneeCtr,true);
    // () compute both hip angles
    Eigen::MatrixXd RHipAngles=bioutils::clinical3DofJcsAngles(R_Pelvis_to_N, R_RFemur_to_N, true, true, false);
    Eigen::MatrixXd LHipAngles=bioutils::clinical3DofJcsAngles(R_Pelvis_to_N, R_LFemur_to_N, false, true, false);
    // () pull out just the internal/external rotation angle component
    Eigen::VectorXd RHipIeAngle=RHipAngles.col(2), LHipIeAngle=LHipAngles.col(2);
    // --- DEBUG: PRINT TO FILE
    gtsamutils::writeEigenMatrixToCsvFile("precorrection_RHipIe_iter"+std::to_string(iterNum)+".csv", RHipIeAngle);
    gtsamutils::writeEigenMatrixToCsvFile("precorrection_LHipIe_iter"+std::to_string(iterNum)+".csv", LHipIeAngle);
    // () handle any unwrapping of the I/E angle
    //angleUnwrapOnKnownDomain(const Eigen::VectorXd& angle, const double& domainMin=-1.0*M_PI, const double& domainMax=M_PI, const double& tol=2*M_PI*0.95);
    RHipIeAngle=mathutils::angleUnwrapOnKnownDomain(RHipIeAngle, -1.0*M_PI, M_PI,0.95 * 2.0 * M_PI); // int/ext rot is on [-pi,pi], so let's say jump tol is 0.95*2*M_PI
    LHipIeAngle=mathutils::angleUnwrapOnKnownDomain(LHipIeAngle, -1.0*M_PI, M_PI,0.95 * 2.0 * M_PI); // int/ext rot is on [-pi,pi], so let's say jump tol is 0.95*2*M_PI
    // --- DEBUG PRINT POINT
    gtsamutils::writeEigenMatrixToCsvFile("precorrection_postunwrap_RHipIe_iter"+std::to_string(iterNum)+".csv", RHipIeAngle);
    gtsamutils::writeEigenMatrixToCsvFile("precorrection_postunwrap_LHipIe_iter"+std::to_string(iterNum)+".csv", LHipIeAngle);
    // () compute new angles which have been adjusted to have regression slope = 0
    Eigen::VectorXd newRHipIeAng=mathutils::adjustRegressionSlopeToTargetSlope(mathutils::StdVectorToEigenVector(m_time), RHipIeAngle, 0.0);
    Eigen::VectorXd newLHipIeAng=mathutils::adjustRegressionSlopeToTargetSlope(mathutils::StdVectorToEigenVector(m_time), LHipIeAngle, 0.0);
    // () adjust angles so intercept is zero (maybe I should figure out something better for this)
    newRHipIeAng=newRHipIeAng.array()-newRHipIeAng[0], newLHipIeAng=newLHipIeAng.array()-newLHipIeAng[0];
    // --- DEBUG PRINT POINT
    gtsamutils::writeEigenMatrixToCsvFile("postcorrection_RHipIe_iter"+std::to_string(iterNum)+".csv", newRHipIeAng);
    gtsamutils::writeEigenMatrixToCsvFile("postcorrection_LHipIe_iter"+std::to_string(iterNum)+".csv", newLHipIeAng);

    // () find the rotation change R[B->B2] for the thigh IMU which would make this new angle realized
    // right leg
    for(uint k=0; k<m_time.size(); k++){
        // get correction orientation
        gtsam::Rot3 R_B_to_B2 = lowerBodyPoseEstimator::getDistalImuOrientationAdjustmentGivenDesiredIntExtRotAngle(R_Pelvis_to_N[k],rthighImuPose[k].rotation(),newRHipIeAng[k],rkneeAxisThigh,rthighImuToHipCtr,rthighImuToKneeCtr); // R[B->B2]
        // apply to right leg IMUs


        rthighImuPose[k]=gtsam::Pose3((rthighImuPose[k].rotation()*R_B_to_B2.inverse()),rthighImuPose[k].translation()); // R[B2->N]=R[B->N]*R[B->B2]' % new orientation R[B2->N]
        rshankImuPose[k]=gtsam::Pose3((rshankImuPose[k].rotation()*R_B_to_B2.inverse()),rshankImuPose[k].translation());
        rfootImuPose[k]=gtsam::Pose3((rfootImuPose[k].rotation()*R_B_to_B2.inverse()),rfootImuPose[k].translation());
    }
    // left leg
    for(uint k=0; k<m_time.size(); k++){
        // get correction orientation
        gtsam::Rot3 R_B_to_B2 = lowerBodyPoseEstimator::getDistalImuOrientationAdjustmentGivenDesiredIntExtRotAngle(R_Pelvis_to_N[k],lthighImuPose[k].rotation(),newLHipIeAng[k],lkneeAxisThigh,lthighImuToHipCtr,lthighImuToKneeCtr); // R[B->B2]
        // apply to left leg IMUs
        lthighImuPose[k]=gtsam::Pose3((lthighImuPose[k].rotation()*R_B_to_B2.inverse()),lthighImuPose[k].translation()); // R[B2->N]=R[B->N]*R[B->B2]' % new orientation R[B2->N]
        lshankImuPose[k]=gtsam::Pose3((lshankImuPose[k].rotation()*R_B_to_B2.inverse()),lshankImuPose[k].translation());
        lfootImuPose[k]=gtsam::Pose3((lfootImuPose[k].rotation()*R_B_to_B2.inverse()),lfootImuPose[k].translation());
    }
    // () put these new poses back into the values
    for(uint k=0; k<m_sacrumImuPoseProblem.m_poseKeys.size();k++){
        vals.update(m_rthighImuPoseProblem.m_poseKeys[k],rthighImuPose[k]);
        vals.update(m_rshankImuPoseProblem.m_poseKeys[k],rshankImuPose[k]);
        vals.update(m_rfootImuPoseProblem.m_poseKeys[k],rfootImuPose[k]);
        vals.update(m_lthighImuPoseProblem.m_poseKeys[k],lthighImuPose[k]);
        vals.update(m_lshankImuPoseProblem.m_poseKeys[k],lshankImuPose[k]);
        vals.update(m_lfootImuPoseProblem.m_poseKeys[k],lfootImuPose[k]);
        vals.update(m_rthighImuPoseProblem.m_velKeys[k],rthighImuVel[k]);
        vals.update(m_rshankImuPoseProblem.m_velKeys[k],rshankImuVel[k]);
        vals.update(m_rfootImuPoseProblem.m_velKeys[k],rfootImuVel[k]);
        vals.update(m_lthighImuPoseProblem.m_velKeys[k],lthighImuVel[k]);
        vals.update(m_lshankImuPoseProblem.m_velKeys[k],lshankImuVel[k]);
        vals.update(m_lfootImuPoseProblem.m_velKeys[k],lfootImuVel[k]);
    }


}

void lowerBodyPoseEstimator::setImuOrientationsBasedOnJointAngleLimits(gtsam::Values& vals, bool zeroMedianCenterIntExtRot, bool zeroMedianCenterAbAd, bool verbose){
    // start at the sacrum and work down both legs.
    // based on initial values for imu poses and expected joint angle limits
    // algorithm: (repeat for each joint as you move down the leg)
    // for k=1:N
    //    calculate joint angles at k
    //    if all joint angles in bounds, do nothing
    //    else
    //        find what rotation deltaR, applied to distal imu, would bring all joint angles in bounds
    //        apply deltaR to all distal imu orientations k:N
    //    (optional) loop back through joint angles k:N and make sure all are in bounds now
    // () set boundaries on angles
    // positive angles for right knee are: extension, adduction, internal rot
    gtsam::Vector2 kneeFlexExMaxMin(12.1 * M_PI/180.0, -162.9 * M_PI/180.0);
    //gtsam::Vector2 kneeAbAdMaxMin(m_maxKneeAdductionRad, -m_maxKneeAbductionRad);
    //gtsam::Vector2 kneeIntExtRotMaxMin(m_maxKneeInternalRotRad, -m_maxKneeExternalRotRad);
    double lim=15.0*M_PI/180.0;
    gtsam::Vector2 kneeAbAdMaxMin(lim,-lim), kneeIntExtRotMaxMin(lim,-lim);
    gtsam::Vector2 hipFlexExMaxMin(2.4435,   -0.5236);
    gtsam::Vector2 rhipAbAdMaxMin(lim,  -lim);
    gtsam::Vector2 rhipIntExtRotMaxMin(lim,  -lim);
    gtsam::Vector2 lhipAbAdMaxMin(lim,  -lim);
    gtsam::Vector2 lhipIntExtRotMaxMin(lim,  -lim);
    // () for convenience, pull out static vectors into variables
    gtsam::Point3 sacrumImuToRHipCtr=vals.at<gtsam::Point3>(m_sacrumImuToRHipCtrKey);
    gtsam::Point3 rthighImuToHipCtr=vals.at<gtsam::Point3>(m_rthighImuToHipCtrKey);
    gtsam::Point3 rthighImuToKneeCtr=vals.at<gtsam::Point3>(m_rthighImuToKneeCtrKey);
    gtsam::Point3 rshankImuToKneeCtr=vals.at<gtsam::Point3>(m_rshankImuToKneeCtrKey);
    gtsam::Point3 rshankImuToAnkleCtr=vals.at<gtsam::Point3>(m_rshankImuToAnkleCtrKey);
    gtsam::Point3 sacrumImuToLHipCtr=vals.at<gtsam::Point3>(m_sacrumImuToLHipCtrKey);
    gtsam::Point3 lthighImuToHipCtr=vals.at<gtsam::Point3>(m_lthighImuToHipCtrKey);
    gtsam::Point3 lthighImuToKneeCtr=vals.at<gtsam::Point3>(m_lthighImuToKneeCtrKey);
    gtsam::Point3 lshankImuToKneeCtr=vals.at<gtsam::Point3>(m_lshankImuToKneeCtrKey);
    gtsam::Point3 lshankImuToAnkleCtr=vals.at<gtsam::Point3>(m_lshankImuToAnkleCtrKey);
    gtsam::Unit3 rkneeAxisThigh=vals.at<gtsam::Unit3>(m_rkneeAxisThighKey);
    gtsam::Unit3 rkneeAxisShank=vals.at<gtsam::Unit3>(m_rkneeAxisShankKey);
    gtsam::Unit3 lkneeAxisThigh=vals.at<gtsam::Unit3>(m_lkneeAxisThighKey);
    gtsam::Unit3 lkneeAxisShank=vals.at<gtsam::Unit3>(m_lkneeAxisShankKey);
    // get sacrum imu pose, it won't be changing. also get other poses and vals
    const std::vector<gtsam::Pose3> sacrumImuPose=imuPoseEstimator::vectorizePoses(vals,m_sacrumImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> rthighImuPose=imuPoseEstimator::vectorizePoses(vals,m_rthighImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> rshankImuPose=imuPoseEstimator::vectorizePoses(vals,m_rshankImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> lthighImuPose=imuPoseEstimator::vectorizePoses(vals,m_lthighImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Pose3> lshankImuPose=imuPoseEstimator::vectorizePoses(vals,m_lshankImuPoseProblem.m_poseKeys);
    std::vector<gtsam::Vector3> rthighImuVel=imuPoseEstimator::vectorizeVelocities(vals,m_rthighImuPoseProblem.m_velKeys);
    std::vector<gtsam::Vector3> rshankImuVel=imuPoseEstimator::vectorizeVelocities(vals,m_rshankImuPoseProblem.m_velKeys);
    std::vector<gtsam::Vector3> lthighImuVel=imuPoseEstimator::vectorizeVelocities(vals,m_lthighImuPoseProblem.m_velKeys);
    std::vector<gtsam::Vector3> lshankImuVel=imuPoseEstimator::vectorizeVelocities(vals,m_lshankImuPoseProblem.m_velKeys);
    // compute R_Pelvis_to_N, which will be const
    const std::vector<gtsam::Rot3> R_Pelvis_to_N=bioutils::get_R_Pelvis_to_N(gtsamutils::Pose3VectorToRot3Vector(sacrumImuPose),Eigen::Vector3d(-1.0,0.0,0.0),sacrumImuToRHipCtr,sacrumImuToLHipCtr);
    const gtsam::Unit3 pelvisRightAxis=gtsam::Unit3(sacrumImuToRHipCtr-sacrumImuToLHipCtr);
    // RHip
    adjustDistalImuTrajectoryForInboundJointAngles(R_Pelvis_to_N, sacrumImuPose, pelvisRightAxis, rthighImuPose, rthighImuVel, rthighImuToHipCtr, rthighImuToKneeCtr,rkneeAxisThigh,hipFlexExMaxMin,rhipAbAdMaxMin,rhipIntExtRotMaxMin,zeroMedianCenterIntExtRot,zeroMedianCenterAbAd);
    const std::vector<gtsam::Rot3> R_RFemur_to_N=bioutils::get_R_Segment_to_N(gtsamutils::Pose3VectorToRot3Vector(rthighImuPose),rkneeAxisThigh,rthighImuToHipCtr,rthighImuToKneeCtr,false);
    if(verbose){
        std::cout<<std::endl;
        Eigen::MatrixXd rhipAngles=bioutils::consistent3DofJcsAngles(R_Pelvis_to_N,R_RFemur_to_N);
        std::cout << "    rhip int/ext rot ang (deg): " << mathutils::distributionInfoString(180.0 / M_PI * rhipAngles.col(2)) << std::endl;
    }
    // RKnee
    lowerBodyPoseEstimator::adjustDistalImuTrajectoryForInboundJointAngles(R_RFemur_to_N,rthighImuPose,rkneeAxisThigh,rshankImuPose, rshankImuVel, rshankImuToKneeCtr, rshankImuToAnkleCtr,rkneeAxisShank,kneeFlexExMaxMin,kneeAbAdMaxMin,kneeIntExtRotMaxMin,zeroMedianCenterIntExtRot,zeroMedianCenterAbAd);
    if(verbose){
        const std::vector<gtsam::Rot3> R_RTibia_to_N=bioutils::get_R_Segment_to_N(gtsamutils::Pose3VectorToRot3Vector(rshankImuPose),rkneeAxisShank,rshankImuToKneeCtr,rshankImuToAnkleCtr);
        Eigen::MatrixXd rhipAngles=bioutils::consistent3DofJcsAngles(R_Pelvis_to_N,R_RFemur_to_N);
        Eigen::MatrixXd rkneeAngles=bioutils::consistent3DofJcsAngles(R_RFemur_to_N,R_RTibia_to_N);
        std::cout << "    rhip int/ext rot ang (deg): " << mathutils::distributionInfoString(180.0 / M_PI * rhipAngles.col(2)) << std::endl;
        std::cout << "    rknee int/ext rot ang (deg): " << mathutils::distributionInfoString(180.0 / M_PI * rkneeAngles.col(2)) << std::endl;
    }
    // LHip
    adjustDistalImuTrajectoryForInboundJointAngles(R_Pelvis_to_N, sacrumImuPose, pelvisRightAxis, lthighImuPose, lthighImuVel, lthighImuToHipCtr, lthighImuToKneeCtr,lkneeAxisThigh,hipFlexExMaxMin,lhipAbAdMaxMin,lhipIntExtRotMaxMin,zeroMedianCenterIntExtRot,zeroMedianCenterAbAd);
    const std::vector<gtsam::Rot3> R_LFemur_to_N=bioutils::get_R_Segment_to_N(gtsamutils::Pose3VectorToRot3Vector(lthighImuPose),lkneeAxisThigh,lthighImuToHipCtr,lthighImuToKneeCtr,false);
    if(verbose){
        const std::vector<gtsam::Rot3> R_RTibia_to_N=bioutils::get_R_Segment_to_N(gtsamutils::Pose3VectorToRot3Vector(rshankImuPose),rkneeAxisShank,rshankImuToKneeCtr,rshankImuToAnkleCtr);
        Eigen::MatrixXd rhipAngles=bioutils::consistent3DofJcsAngles(R_Pelvis_to_N,R_RFemur_to_N);
        Eigen::MatrixXd rkneeAngles=bioutils::consistent3DofJcsAngles(R_RFemur_to_N,R_RTibia_to_N);
        Eigen::MatrixXd lhipAngles=bioutils::consistent3DofJcsAngles(R_Pelvis_to_N,R_LFemur_to_N);
        std::cout << "    rhip int/ext rot ang (deg): " << mathutils::distributionInfoString(180.0 / M_PI * rhipAngles.col(2)) << std::endl;
        std::cout << "    rknee int/ext rot ang (deg): " << mathutils::distributionInfoString(180.0 / M_PI * rkneeAngles.col(2)) << std::endl;
        std::cout << "    lhip int/ext rot ang (deg): " << mathutils::distributionInfoString(180.0 / M_PI * lhipAngles.col(2)) << std::endl;
    }
    // LKnee
    lowerBodyPoseEstimator::adjustDistalImuTrajectoryForInboundJointAngles(R_LFemur_to_N,lthighImuPose,lkneeAxisThigh,lshankImuPose, lshankImuVel, lshankImuToKneeCtr, lshankImuToAnkleCtr,lkneeAxisShank,kneeFlexExMaxMin,kneeAbAdMaxMin,kneeIntExtRotMaxMin,zeroMedianCenterIntExtRot,zeroMedianCenterAbAd);
    if(verbose){ // print new joint angle limits
        // get tibia coordinate systems
        const std::vector<gtsam::Rot3> R_RTibia_to_N=bioutils::get_R_Segment_to_N(gtsamutils::Pose3VectorToRot3Vector(rshankImuPose),rkneeAxisShank,rshankImuToKneeCtr,rshankImuToAnkleCtr);
        const std::vector<gtsam::Rot3> R_LTibia_to_N=bioutils::get_R_Segment_to_N(gtsamutils::Pose3VectorToRot3Vector(lshankImuPose),lkneeAxisShank,lshankImuToKneeCtr,lshankImuToAnkleCtr);
        // get angles
        Eigen::MatrixXd rhipAngles=bioutils::consistent3DofJcsAngles(R_Pelvis_to_N,R_RFemur_to_N);
        Eigen::MatrixXd lhipAngles=bioutils::consistent3DofJcsAngles(R_Pelvis_to_N,R_LFemur_to_N);
        Eigen::MatrixXd rkneeAngles=bioutils::consistent3DofJcsAngles(R_RFemur_to_N,R_RTibia_to_N);
        Eigen::MatrixXd lkneeAngles=bioutils::consistent3DofJcsAngles(R_LFemur_to_N,R_LTibia_to_N);
        // print
        std::cout << "    rhip int/ext rot ang (deg): " << mathutils::distributionInfoString(180.0 / M_PI * rhipAngles.col(2)) << std::endl;
        std::cout << "    lhip int/ext rot ang (deg): " << mathutils::distributionInfoString(180.0 / M_PI * lhipAngles.col(2)) << std::endl;
        std::cout << "    rknee int/ext rot ang (deg): " << mathutils::distributionInfoString(180.0 / M_PI * rkneeAngles.col(2)) << std::endl;
        std::cout << "    lknee int/ext rot ang (deg): " << mathutils::distributionInfoString(180.0 / M_PI * lkneeAngles.col(2)) << std::endl;
    }
    // () put these new poses back into the values
    for(uint k=0; k<m_sacrumImuPoseProblem.m_poseKeys.size();k++){
        vals.update(m_rthighImuPoseProblem.m_poseKeys[k],rthighImuPose[k]);
        vals.update(m_rshankImuPoseProblem.m_poseKeys[k],rshankImuPose[k]);
        vals.update(m_lthighImuPoseProblem.m_poseKeys[k],lthighImuPose[k]);
        vals.update(m_lshankImuPoseProblem.m_poseKeys[k],lshankImuPose[k]);
        vals.update(m_rthighImuPoseProblem.m_velKeys[k],rthighImuVel[k]);
        vals.update(m_rshankImuPoseProblem.m_velKeys[k],rshankImuVel[k]);
        vals.update(m_lthighImuPoseProblem.m_velKeys[k],lthighImuVel[k]);
        vals.update(m_lshankImuPoseProblem.m_velKeys[k],lshankImuVel[k]);
    }
    if(verbose){ // print new joint angle limits
        // get tibia coordinate systems
        const std::vector<gtsam::Rot3> R_RTibia_to_N=bioutils::get_R_Segment_to_N(gtsamutils::Pose3VectorToRot3Vector(rshankImuPose),rkneeAxisShank,rshankImuToKneeCtr,rshankImuToAnkleCtr);
        const std::vector<gtsam::Rot3> R_LTibia_to_N=bioutils::get_R_Segment_to_N(gtsamutils::Pose3VectorToRot3Vector(lshankImuPose),lkneeAxisShank,lshankImuToKneeCtr,lshankImuToAnkleCtr);
        // get angles
        Eigen::MatrixXd rhipAngles=bioutils::consistent3DofJcsAngles(R_Pelvis_to_N,R_RFemur_to_N);
        Eigen::MatrixXd lhipAngles=bioutils::consistent3DofJcsAngles(R_Pelvis_to_N,R_LFemur_to_N);
        Eigen::MatrixXd rkneeAngles=bioutils::consistent3DofJcsAngles(R_RFemur_to_N,R_RTibia_to_N);
        Eigen::MatrixXd lkneeAngles=bioutils::consistent3DofJcsAngles(R_LFemur_to_N,R_LTibia_to_N);
        // print
        std::cout<<"--- angles after setImuOrientationsBasedOnJointAngleLimits() ---"<<std::endl;
        std::cout << "    rhip int/ext rot ang (deg): " << mathutils::distributionInfoString(180.0 / M_PI * rhipAngles.col(2)) << std::endl;
        std::cout << "    lhip int/ext rot ang (deg): " << mathutils::distributionInfoString(180.0 / M_PI * lhipAngles.col(2)) << std::endl;
        std::cout << "    rknee int/ext rot ang (deg): " << mathutils::distributionInfoString(180.0 / M_PI * rkneeAngles.col(2)) << std::endl;
        std::cout << "    lknee int/ext rot ang (deg): " << mathutils::distributionInfoString(180.0 / M_PI * lkneeAngles.col(2)) << std::endl;
        std::cout<<"----------------------------------------------------------------"<<std::endl;
    }
}

void lowerBodyPoseEstimator::adjustDistalImuTrajectoryForInboundJointAngles(const std::vector<gtsam::Rot3>& R_ProxSeg_to_N,const std::vector<gtsam::Pose3>& proxImuPose, const gtsam::Unit3& rightAxisProx, std::vector<gtsam::Pose3>& distalImuPose, std::vector<gtsam::Vector3>& distalImuVel, const gtsam::Point3& distalImuToProxJointCtr, const gtsam::Point3& distalImuToDistalJointCtr,const gtsam::Unit3& rightAxisDist,const gtsam::Vector2& flexExMaxMin,const gtsam::Vector2& abAdMaxMin,const gtsam::Vector2& intExtRotMaxMin,bool zeroMedianCenterIntExtRot, bool zeroMedianCenterAbAd){
    // note: since the proximal segment is assumed fixed here, just pass in the proximal segment to N orientation
    // --- settings ----
    double overcorrectionAngle=2.0*M_PI/180.0; // how much to "overcorrect" by when moving an anatomical angle inbounds
    bool doubleCheckAllAnglesInBounds=true; // at end, go back and make sure everything is in bounds?
    // -----------------
    uint N=proxImuPose.size();
    // () calculate original joint angles
    std::vector<gtsam::Rot3> R_DistalSeg_to_N_orig = bioutils::get_R_Segment_to_N(gtsamutils::Pose3VectorToRot3Vector(distalImuPose), rightAxisDist, distalImuToProxJointCtr,distalImuToDistalJointCtr);
    Eigen::MatrixXd origJointAngles=bioutils::consistent3DofJcsAngles(R_ProxSeg_to_N,R_DistalSeg_to_N_orig); // [flexion,adduction,externalRot]
    gtsam::Rot3 R_B_to_B2=gtsam::Rot3::Identity(),R_DistalSeg_to_N_k=gtsam::Rot3::Identity();
    // () apriori centering of median int/ext rotation angle to zero
    if(zeroMedianCenterIntExtRot){
        Eigen::VectorXd intExtRotAngs=origJointAngles.col(2);
        std::vector<double> vec(intExtRotAngs.data(), intExtRotAngs.data() + intExtRotAngs.rows() * intExtRotAngs.cols()); // cast as std::vector
        double myMedian=gtsamutils::median(vec);
        uint medIdx=gtsamutils::nearestIdxToVal(vec, myMedian);
        //std::cout<<std::endl<<"int/ext rot median is "<<myMedian<<" and occurs at index "<<medIdx<<" (true val in array = "<<vec[medIdx]<<")"<<std::endl;
        R_B_to_B2 = lowerBodyPoseEstimator::getDistalImuOrientationAdjustmentGivenDesiredIntExtRotAngle(R_ProxSeg_to_N[medIdx],distalImuPose[medIdx].rotation(),0.0,rightAxisDist,distalImuToProxJointCtr,distalImuToDistalJointCtr); // R[B->B2]
        for(uint j=0; j<N; j++){
            gtsam::Rot3 R_newDistalImu_to_N=(distalImuPose[j].rotation()*R_B_to_B2.inverse()); // R[B2->N]=R[B->N]*R[B->B2]' % new orientation R[B2->N]
            distalImuPose[j]=gtsam::Pose3(R_newDistalImu_to_N,R_B_to_B2*distalImuPose[j].translation());
            distalImuVel[j]=R_B_to_B2*distalImuVel[j];
        }
    }
    // () apriori centering of median ab/ad angle to zero
    if(zeroMedianCenterAbAd){
        Eigen::VectorXd abAdAngs=origJointAngles.col(1);
        std::vector<double> vec(abAdAngs.data(), abAdAngs.data() + abAdAngs.rows() * abAdAngs.cols()); // cast as std::vector
        double myMedian=gtsamutils::median(vec);
        uint medIdx=gtsamutils::nearestIdxToVal(vec, myMedian);
        //std::cout<<std::endl<<"ab/ad rot median is "<<myMedian<<" and occurs at index "<<medIdx<<" (true val in array = "<<vec[medIdx]<<")"<<std::endl;
        R_B_to_B2 = lowerBodyPoseEstimator::getDistalImuOrientationAdjustmentGivenDesiredAbAdAngle(R_ProxSeg_to_N[medIdx],distalImuPose[medIdx].rotation(),0.0,rightAxisDist,distalImuToProxJointCtr,distalImuToDistalJointCtr); // R[B->B2]
        for(uint j=0; j<N; j++){
            gtsam::Rot3 R_newDistalImu_to_N=(R_B_to_B2*distalImuPose[j].rotation().inverse()).inverse(); // R[B2->N]=(R[B->B2]*R[B->N]')' % new orientation R[B2->N]
            distalImuPose[j]=gtsam::Pose3(R_newDistalImu_to_N,R_B_to_B2*distalImuPose[j].translation());
            distalImuVel[j]=R_B_to_B2*distalImuVel[j];
        }
    }
    // () main loop:
    double desiredIntExtRotAngle=0.0, desiredAbAdAngle=0.0, desiredFlexExAngle=0.0;
    for(uint k=0; k<N;k++){
        // () calculate joint angles at k
        R_DistalSeg_to_N_k = bioutils::get_R_Segment_to_N(distalImuPose[k].rotation(), rightAxisDist, distalImuToProxJointCtr,distalImuToDistalJointCtr);
        Eigen::RowVector3d jointAnglesK=bioutils::consistent3DofJcsAngles(R_ProxSeg_to_N[k],R_DistalSeg_to_N_k);
        // () if int/ext rot out of bounds, correct entire IMU trajectory k:N
        if(jointAnglesK(2)>intExtRotMaxMin(0)){ // too toe to the left-rotated (int/ext)
            desiredIntExtRotAngle=intExtRotMaxMin(0)-overcorrectionAngle;
        }else if(jointAnglesK(2)<intExtRotMaxMin(1)){ // too toe to the right-rotated (int/ext)
            desiredIntExtRotAngle=intExtRotMaxMin(1)+overcorrectionAngle;
        }
        if(jointAnglesK(2)>intExtRotMaxMin(0) || jointAnglesK(2)<intExtRotMaxMin(1)) {
            R_B_to_B2 = lowerBodyPoseEstimator::getDistalImuOrientationAdjustmentGivenDesiredIntExtRotAngle(R_ProxSeg_to_N[k],distalImuPose[k].rotation(),desiredIntExtRotAngle,rightAxisDist,distalImuToProxJointCtr,distalImuToDistalJointCtr); // R[B->B2]
            for(uint j=k; j<N; j++){
                gtsam::Rot3 R_newDistalImu_to_N=(R_B_to_B2*distalImuPose[j].rotation().inverse()).inverse(); // R[B2->N]=(R[B->B2]*R[B->N]')' % new orientation R[B2->N]
                distalImuPose[j]=gtsam::Pose3(R_newDistalImu_to_N,R_B_to_B2*distalImuPose[j].translation());
                distalImuVel[j]=R_B_to_B2*distalImuVel[j];
            }
        }
        // () if flexion/extension out of bounds, correct entire IMU trajectory k:N
        if(jointAnglesK(0)>flexExMaxMin(0)){ // knee is too extended
            desiredFlexExAngle=flexExMaxMin(0)-overcorrectionAngle;
        }else if(jointAnglesK(0)<flexExMaxMin(1)){ // // knee is too flexed
            desiredFlexExAngle=flexExMaxMin(1)+overcorrectionAngle;
        }
        if(jointAnglesK(0)>flexExMaxMin(0) || jointAnglesK(0)<flexExMaxMin(1)){
            R_B_to_B2 = lowerBodyPoseEstimator::getDistalImuOrientationAdjustmentGivenDesiredFlexExAngle(R_ProxSeg_to_N[k],proxImuPose[k].rotation(),distalImuPose[k].rotation(),desiredFlexExAngle,rightAxisProx,rightAxisDist,distalImuToProxJointCtr,distalImuToDistalJointCtr); // R[B->B2]
            for(uint j=k; j<N; j++){
                gtsam::Rot3 R_newDistalImu_to_N=(R_B_to_B2*distalImuPose[j].rotation().inverse()).inverse(); // R[B2->N]=(R[B->B2]*R[B->N]')' % new orientation R[B2->N]
                distalImuPose[j]=gtsam::Pose3(R_newDistalImu_to_N,R_B_to_B2*distalImuPose[j].translation());
                distalImuVel[j]=R_B_to_B2*distalImuVel[j];
            }
        }
        // () if ab/ad out of bounds, correct entire IMU trajectory k:N
        if(jointAnglesK(1)>abAdMaxMin(0)) { // too toe to left (ab/ad sense)
            desiredAbAdAngle=abAdMaxMin(0)-overcorrectionAngle;
        }else if(jointAnglesK(1)<abAdMaxMin(1)){ // // too toe to right (ab/ad sense)
            desiredAbAdAngle=abAdMaxMin(1)+overcorrectionAngle;
        }
        if(jointAnglesK(1)>abAdMaxMin(0) || jointAnglesK(1)<abAdMaxMin(1)) {
            R_B_to_B2 = lowerBodyPoseEstimator::getDistalImuOrientationAdjustmentGivenDesiredAbAdAngle(R_ProxSeg_to_N[k],distalImuPose[k].rotation(),desiredAbAdAngle,rightAxisDist,distalImuToProxJointCtr,distalImuToDistalJointCtr); // R[B->B2]
            for(uint j=k; j<N; j++){
                gtsam::Rot3 R_newDistalImu_to_N=(R_B_to_B2*distalImuPose[j].rotation().inverse()).inverse(); // R[B2->N]=(R[B->B2]*R[B->N]')' % new orientation R[B2->N]
                distalImuPose[j]=gtsam::Pose3(R_newDistalImu_to_N,R_B_to_B2*distalImuPose[j].translation());
                distalImuVel[j]=R_B_to_B2*distalImuVel[j];
            }
        }
    }
    // () calculate new angles
    std::vector<gtsam::Rot3> R_DistalSeg_to_N_new = bioutils::get_R_Segment_to_N(gtsamutils::Pose3VectorToRot3Vector(distalImuPose), rightAxisDist, distalImuToProxJointCtr,distalImuToDistalJointCtr);
    Eigen::MatrixXd newJointAngles=bioutils::consistent3DofJcsAngles(R_ProxSeg_to_N,R_DistalSeg_to_N_new); // [flexion,adduction,externalRot]
    // () optional: loop back through and check that all angles are in bounds
    if(doubleCheckAllAnglesInBounds){
        for(uint k=0; k<newJointAngles.rows(); k++){
            if(newJointAngles(k,0)>flexExMaxMin(0)){
                std::string errorStr="angle is out of bounds at k="+std::to_string(k)+": knee is too extended (angle "+std::to_string(newJointAngles(k,0))+"> max "+std::to_string(flexExMaxMin(0))+")";
                throw std::runtime_error(errorStr);
            }
            if(newJointAngles(k,0)<flexExMaxMin(1)){
                std::string errorStr="angle is out of bounds at k="+std::to_string(k)+": knee is too flexed (angle "+std::to_string(newJointAngles(k,0))+"< min "+std::to_string(flexExMaxMin(1))+")";
                throw std::runtime_error(errorStr);
            }
            if(newJointAngles(k,1)>abAdMaxMin(0)){
                std::string errorStr="angle is out of bounds at k="+std::to_string(k)+": knee is too toe to left (ab/ad sense) (angle "+std::to_string(newJointAngles(k,1))+"> max "+std::to_string(abAdMaxMin(0))+")";
                throw std::runtime_error(errorStr);
            }
            if(newJointAngles(k,1)<abAdMaxMin(1)){
                std::string errorStr="angle is out of bounds at k="+std::to_string(k)+": knee is too toe to right (ab/ad sense) (angle "+std::to_string(newJointAngles(k,1))+"< min "+std::to_string(abAdMaxMin(1))+")";
                throw std::runtime_error(errorStr);
            }
            if(newJointAngles(k,2)>intExtRotMaxMin(0)){
                std::string errorStr="angle is out of bounds at k="+std::to_string(k)+": knee is too toe to the left-rotated (int/ext) (angle "+std::to_string(newJointAngles(k,2))+"> max "+std::to_string(intExtRotMaxMin(0))+")";
                throw std::runtime_error(errorStr);
            }
            if(newJointAngles(k,2)<intExtRotMaxMin(1)){
                std::string errorStr="angle is out of bounds at k="+std::to_string(k)+": knee is too toe to the right-rotated (int/ext) (angle "+std::to_string(newJointAngles(k,2))+"< min "+std::to_string(intExtRotMaxMin(1))+")";
                throw std::runtime_error(errorStr);
            }
        }
    }
}

void lowerBodyPoseEstimator::setImuPositionsBasedOnConsistentInitialStaticVecsToJointCtrs(gtsam::Values& vals){
    // start at the sacrum and work down both legs.
    // based on initial values for the static vectors to joint centers, adjust all positions to be consistent
    // for convenience, pull out static vectors into variables
    // note: if this method is applied, the error due to ConstrainedJointCenterPositionFactor should be zero!
    gtsam::Point3 sacrumImuToRHipCtr=m_initialValues.at<gtsam::Point3>(m_sacrumImuToRHipCtrKey);
    gtsam::Point3 rthighImuToHipCtr=m_initialValues.at<gtsam::Point3>(m_rthighImuToHipCtrKey);
    gtsam::Point3 rthighImuToKneeCtr=m_initialValues.at<gtsam::Point3>(m_rthighImuToKneeCtrKey);
    gtsam::Point3 rshankImuToKneeCtr=m_initialValues.at<gtsam::Point3>(m_rshankImuToKneeCtrKey);
    gtsam::Point3 rshankImuToAnkleCtr=m_initialValues.at<gtsam::Point3>(m_rshankImuToAnkleCtrKey);
    gtsam::Point3 rfootImuToAnkleCtr=m_initialValues.at<gtsam::Point3>(m_rfootImuToAnkleCtrKey);
    gtsam::Point3 sacrumImuToLHipCtr=m_initialValues.at<gtsam::Point3>(m_sacrumImuToLHipCtrKey);
    gtsam::Point3 lthighImuToHipCtr=m_initialValues.at<gtsam::Point3>(m_lthighImuToHipCtrKey);
    gtsam::Point3 lthighImuToKneeCtr=m_initialValues.at<gtsam::Point3>(m_lthighImuToKneeCtrKey);
    gtsam::Point3 lshankImuToKneeCtr=m_initialValues.at<gtsam::Point3>(m_lshankImuToKneeCtrKey);
    gtsam::Point3 lshankImuToAnkleCtr=m_initialValues.at<gtsam::Point3>(m_lshankImuToAnkleCtrKey);
    gtsam::Point3 lfootImuToAnkleCtr=m_initialValues.at<gtsam::Point3>(m_lfootImuToAnkleCtrKey);
    // get sacrum imu pose, it won't be changing
    std::vector<gtsam::Pose3> sacrumImuPose=imuPoseEstimator::vectorizePoses(vals,m_sacrumImuPoseProblem.m_poseKeys);
    // RHip
    std::vector<gtsam::Pose3> rthighImuPose=imuPoseEstimator::vectorizePoses(vals,m_rthighImuPoseProblem.m_poseKeys);
    adjustDistalImuPosBasedOnStaticVecsToJointCtr(sacrumImuPose,rthighImuPose,sacrumImuToRHipCtr,rthighImuToHipCtr);
    // RKnee
    std::vector<gtsam::Pose3> rshankImuPose=imuPoseEstimator::vectorizePoses(vals,m_rshankImuPoseProblem.m_poseKeys);
    adjustDistalImuPosBasedOnStaticVecsToJointCtr(rthighImuPose,rshankImuPose,rthighImuToKneeCtr,rshankImuToKneeCtr);
    // RAnkle
    std::vector<gtsam::Pose3> rfootImuPose=imuPoseEstimator::vectorizePoses(vals,m_rfootImuPoseProblem.m_poseKeys);
    adjustDistalImuPosBasedOnStaticVecsToJointCtr(rshankImuPose,rfootImuPose,rshankImuToAnkleCtr,rfootImuToAnkleCtr);
    // LHip
    std::vector<gtsam::Pose3> lthighImuPose=imuPoseEstimator::vectorizePoses(vals,m_lthighImuPoseProblem.m_poseKeys);
    adjustDistalImuPosBasedOnStaticVecsToJointCtr(sacrumImuPose,lthighImuPose,sacrumImuToLHipCtr,lthighImuToHipCtr);
    // LKnee
    std::vector<gtsam::Pose3> lshankImuPose=imuPoseEstimator::vectorizePoses(vals,m_lshankImuPoseProblem.m_poseKeys);
    adjustDistalImuPosBasedOnStaticVecsToJointCtr(lthighImuPose,lshankImuPose,lthighImuToKneeCtr,lshankImuToKneeCtr);
    // LAnkle
    std::vector<gtsam::Pose3> lfootImuPose=imuPoseEstimator::vectorizePoses(vals,m_lfootImuPoseProblem.m_poseKeys);
    adjustDistalImuPosBasedOnStaticVecsToJointCtr(lshankImuPose,lfootImuPose,lshankImuToAnkleCtr,lfootImuToAnkleCtr);
    // put all of these back in the values
    for(uint k=0; k<sacrumImuPose.size(); k++){
        vals.update(m_rthighImuPoseProblem.m_poseKeys[k],rthighImuPose[k]);
        vals.update(m_rshankImuPoseProblem.m_poseKeys[k],rshankImuPose[k]);
        vals.update(m_rfootImuPoseProblem.m_poseKeys[k],rfootImuPose[k]);
        vals.update(m_lthighImuPoseProblem.m_poseKeys[k],lthighImuPose[k]);
        vals.update(m_lshankImuPoseProblem.m_poseKeys[k],lshankImuPose[k]);
        vals.update(m_lfootImuPoseProblem.m_poseKeys[k],lfootImuPose[k]);
    }
}

void lowerBodyPoseEstimator::adjustDistalImuPosBasedOnStaticVecsToJointCtr(const gtsam::Pose3& proximalImuPose,gtsam::Pose3& distalImuPose,const gtsam::Point3& proximalImuToJointCtr,const gtsam::Point3& distalImuToJointCtr){
    // edits distal imu pose in place
    // () calculate joint center in world frame
    const gtsam::Vector3 jointCtrPerProximalImu=proximalImuPose.translation()+proximalImuPose.rotation().matrix()*proximalImuToJointCtr; // world frame
    // () now find what the distal imu position must be given its orientation and joint ctr
    const gtsam::Vector3 distalImuPosFixed=jointCtrPerProximalImu-distalImuPose.rotation().matrix()*distalImuToJointCtr;
    // now put this new position into the array
    distalImuPose=gtsam::Pose3(distalImuPose.rotation(),gtsam::Point3(distalImuPosFixed));
    // optional: double check your math
    const gtsam::Vector3 jointCtrPerDistalImu=distalImuPose.translation()+distalImuPose.rotation().matrix()*distalImuToJointCtr; // world frame
    if(!jointCtrPerProximalImu.isApprox(jointCtrPerDistalImu)){
        throw std::runtime_error("vectors from imus to joint centers could not be consistently created.");
    }
}

void lowerBodyPoseEstimator::setValuesFromFile(gtsam::Values& vals, const std::string& file){
    // read data from file and set into values
    // make sure file exists
    if(!boost::filesystem::exists(file)){ std::cerr<<"file "<<file<<" does not exist!"<<std::endl; }
    //
    HighFive::File calH5File(file,HighFive::File::ReadOnly);
    HighFive::Group rootGroup=calH5File.getGroup("/");
    // read time
    std::vector<double> time;
    rootGroup.getDataSet("Time").read(time);
    if(time.empty()){  throw std::runtime_error("could not read /Time dataset (size is zero)"); }
    // read positions and quaternions. remember that the measured Vicon positions were in mm!
    Eigen::MatrixXd p_SacrumImu = H5Easy::load<Eigen::MatrixXd>(calH5File, "/p_SacrumImu");
    Eigen::MatrixXd p_RThighImu = H5Easy::load<Eigen::MatrixXd>(calH5File, "/p_RThighImu");
    Eigen::MatrixXd p_RShankImu = H5Easy::load<Eigen::MatrixXd>(calH5File, "/p_RShankImu");
    Eigen::MatrixXd p_RFootImu = H5Easy::load<Eigen::MatrixXd>(calH5File, "/p_RFootImu");
    Eigen::MatrixXd p_LThighImu = H5Easy::load<Eigen::MatrixXd>(calH5File, "/p_LThighImu");
    Eigen::MatrixXd p_LShankImu = H5Easy::load<Eigen::MatrixXd>(calH5File, "/p_LShankImu");
    Eigen::MatrixXd p_LFootImu = H5Easy::load<Eigen::MatrixXd>(calH5File, "/p_LFootImu");
    Eigen::MatrixXd q_SacrumImu_to_N = H5Easy::load<Eigen::MatrixXd>(calH5File, "/q_SacrumImu_to_N");
    Eigen::MatrixXd q_RThighImu_to_N = H5Easy::load<Eigen::MatrixXd>(calH5File, "/q_RThighImu_to_N");
    Eigen::MatrixXd q_RShankImu_to_N = H5Easy::load<Eigen::MatrixXd>(calH5File, "/q_RShankImu_to_N");
    Eigen::MatrixXd q_RFootImu_to_N = H5Easy::load<Eigen::MatrixXd>(calH5File, "/q_RFootImu_to_N");
    Eigen::MatrixXd q_LThighImu_to_N = H5Easy::load<Eigen::MatrixXd>(calH5File, "/q_LThighImu_to_N");
    Eigen::MatrixXd q_LShankImu_to_N = H5Easy::load<Eigen::MatrixXd>(calH5File, "/q_LShankImu_to_N");
    Eigen::MatrixXd q_LFootImu_to_N = H5Easy::load<Eigen::MatrixXd>(calH5File, "/q_LFootImu_to_N");

    if(p_SacrumImu.rows()!=3){ throw std::runtime_error("was expecting input position data to be a 3xN matrix"); }
    if(q_SacrumImu_to_N.rows()!=4){ throw std::runtime_error("was expecting input quat data to be a 4xN matrix"); }
    if(q_SacrumImu_to_N.cols()!=p_SacrumImu.cols()){ throw std::runtime_error("pos and quat matrices don't have same length (number of columns)"); }
    // () divide all position matrices by 1000 to put into meters!
    p_SacrumImu=p_SacrumImu/1000;
    p_RThighImu=p_RThighImu/1000;
    p_RShankImu=p_RShankImu/1000;
    p_RFootImu=p_RFootImu/1000;
    p_LThighImu=p_LThighImu/1000;
    p_LShankImu=p_LShankImu/1000;
    p_LFootImu=p_LFootImu/1000;
    // --- now read data into values --- //
    //     remember: it read data into 3xN array
    std::cout<<"size of m_time="<<m_time.size()<<std::endl;
    double loopStart=clock();
    uint foundIdxs=0, j=0;
    for(uint k=0; k<m_time.size();k++){
        // m_time[k] is the current time you'd like to find a position and orientation for
        // inside while loop, look for index j in the h5 data which matches your time
        //      once found, break the while loop and then update so on the next iteration you aren't starting from index 0 in the h5 file
        for(uint i=j; i<time.size(); i++){
            //std::cout<<"k="<<k<<", j="<<j<<", i="<<i<<std::endl;
            if(abs(m_time[k]-time[j])<0.00001){ // you found data, populate it
                //std::cout<<"found match at "<<"k="<<k<<", j="<<j<<", i="<<i<<std::endl;
                // construct orientation from quaternion and position into Pose3
                gtsam::Pose3 poseSacrumImu=gtsam::Pose3(gtsam::Rot3(q_SacrumImu_to_N(j,0),q_SacrumImu_to_N(j,1),q_SacrumImu_to_N(j,2),q_SacrumImu_to_N(j,3)),gtsam::Point3(p_SacrumImu(j,0),p_SacrumImu(j,1),p_SacrumImu(j,2)));
                gtsam::Pose3 poseRThighImu=gtsam::Pose3(gtsam::Rot3(q_RThighImu_to_N(j,0),q_RThighImu_to_N(j,1),q_RThighImu_to_N(j,2),q_RThighImu_to_N(j,3)),gtsam::Point3(p_RThighImu(j,0),p_RThighImu(j,1),p_RThighImu(j,2)));
                gtsam::Pose3 poseRShankImu=gtsam::Pose3(gtsam::Rot3(q_RShankImu_to_N(j,0),q_RShankImu_to_N(j,1),q_RShankImu_to_N(j,2),q_RShankImu_to_N(j,3)),gtsam::Point3(p_RShankImu(j,0),p_RShankImu(j,1),p_RShankImu(j,2)));
                gtsam::Pose3 poseRFootImu=gtsam::Pose3(gtsam::Rot3(q_RFootImu_to_N(j,0),q_RFootImu_to_N(j,1),q_RFootImu_to_N(j,2),q_RFootImu_to_N(j,3)),gtsam::Point3(p_RFootImu(j,0),p_RFootImu(j,1),p_RFootImu(j,2)));
                gtsam::Pose3 poseLThighImu=gtsam::Pose3(gtsam::Rot3(q_LThighImu_to_N(j,0),q_LThighImu_to_N(j,1),q_LThighImu_to_N(j,2),q_LThighImu_to_N(j,3)),gtsam::Point3(p_LThighImu(j,0),p_LThighImu(j,1),p_LThighImu(j,2)));
                gtsam::Pose3 poseLShankImu=gtsam::Pose3(gtsam::Rot3(q_LShankImu_to_N(j,0),q_LShankImu_to_N(j,1),q_LShankImu_to_N(j,2),q_LShankImu_to_N(j,3)),gtsam::Point3(p_LShankImu(j,0),p_LShankImu(j,1),p_LShankImu(j,2)));
                gtsam::Pose3 poseLFootImu=gtsam::Pose3(gtsam::Rot3(q_LFootImu_to_N(j,0),q_LFootImu_to_N(j,1),q_LFootImu_to_N(j,2),q_LFootImu_to_N(j,3)),gtsam::Point3(p_LFootImu(j,0),p_LFootImu(j,1),p_LFootImu(j,2)));
                // now add these pose3s to the Values
                //std::cout<<"before vals update, sacrumImuPose = "<<vals.at<gtsam::Pose3>(m_sacrumImuPoseProblem.m_poseKeys[k])<<std::endl;
                vals.update(m_sacrumImuPoseProblem.m_poseKeys[k],poseSacrumImu);
                vals.update(m_rthighImuPoseProblem.m_poseKeys[k],poseRThighImu);
                vals.update(m_rshankImuPoseProblem.m_poseKeys[k],poseRShankImu);
                vals.update(m_rfootImuPoseProblem.m_poseKeys[k],poseRFootImu);
                vals.update(m_lthighImuPoseProblem.m_poseKeys[k],poseLThighImu);
                vals.update(m_lshankImuPoseProblem.m_poseKeys[k],poseLShankImu);
                vals.update(m_lfootImuPoseProblem.m_poseKeys[k],poseLFootImu);
                //std::cout<<"after vals update, sacrumImuPose = "<<vals.at<gtsam::Pose3>(m_sacrumImuPoseProblem.m_poseKeys[k])<<std::endl;
                foundIdxs++;
                break; // break the i=j for loop, don't want to keep searching i>j
            }
            j++;
        }
    }
    std::cout<<"    updated "<<foundIdxs<<"/"<<m_time.size()<<" indexes of poses in Values ("<<(clock()-loopStart)/CLOCKS_PER_SEC<<" sec)"<<std::endl;
}

void lowerBodyPoseEstimator::adjustDistalImuPosBasedOnStaticVecsToJointCtr(const std::vector<gtsam::Pose3>& proximalImuPose,std::vector<gtsam::Pose3>& distalImuPose,const gtsam::Point3& proximalImuToJointCtr,const gtsam::Point3& distalImuToJointCtr) {
    // vectorized version of function of same name, edits the distal imu pose in place
    for(uint k=0; k<proximalImuPose.size(); k++){
        adjustDistalImuPosBasedOnStaticVecsToJointCtr(proximalImuPose[k],distalImuPose[k],proximalImuToJointCtr,distalImuToJointCtr);
    }
}

void lowerBodyPoseEstimator::searchYawAnglesForInboundsJointAngles(gtsam::Values &vals, bool verbose){
    // this method corrects all IMU poses in input gtsam::Values vals, editing the values in place
    // it goes from proximal IMUs to distal IMUs (for both legs), rotating yaw of imu poses until joint angles seem reasonable.
    // this is useful for running on the initial values before optimization occurs
    // ---- settings ---- //
    uint numIterationsPerSearch=40; // you'll rotate a full 360 deg, this edits the granularity of the search
    double maxHipFlexLimit=40*M_PI/180.0;
    double minHipFlexLimit=-170*M_PI/180.0;
    double maxHipAddLimit=50*M_PI/180.0;
    double minHipAddLimit=-50*M_PI/180.0;
    double maxHipExtLimit=50*M_PI/180.0;
    double minHipExtLimit=-50*M_PI/180.0;
    double maxKneeFlexLimit=30*M_PI/180.0;
    double minKneeFlexLimit=-170*M_PI/180.0;
    double maxKneeAddLimit=30*M_PI/180.0;
    double minKneeAddLimit=-30*M_PI/180.0;
    double maxKneeExtLimit=30*M_PI/180.0;
    double minKneeExtLimit=-30*M_PI/180.0;
    gtsam::Vector3 approxUp(-1.0,0.0,0.0); // in sacrum imu frame
    // ------------------ //
    std::cout<<"     searching for best joint angles by editing yaw... "<<std::endl;
    double origGraphErr=m_graph.error(vals);
    double beginTic=clock();
    uint nKeyframes=m_rthighImuPoseProblem.m_poseKeys.size();
    double headingAngleDeltaRad=(2*M_PI)/numIterationsPerSearch;
    // we hold the sacrum IMU's pose constant, and then move down the leg
    // ----------- right hip -----------
    bool rhipPass=false;
    // () get segment systems -> N orientation
    std::vector<gtsam::Rot3> R_Pelvis_to_N=bioutils::get_R_Pelvis_to_N(imuPoseEstimator::vectorizeOrientations(vals,m_sacrumImuPoseProblem.m_poseKeys),approxUp,vals.at<gtsam::Point3>(m_sacrumImuToRHipCtrKey),vals.at<gtsam::Point3>(m_sacrumImuToLHipCtrKey));
    std::vector<gtsam::Rot3> R_RFemur_to_N=bioutils::get_R_Segment_to_N(imuPoseEstimator::vectorizeOrientations(vals,m_rthighImuPoseProblem.m_poseKeys),vals.at<gtsam::Unit3>(m_rkneeAxisThighKey),vals.at<gtsam::Point3>(m_rthighImuToHipCtrKey),vals.at<gtsam::Point3>(m_rthighImuToKneeCtrKey));
    // () compute RHip Angles
    Eigen::MatrixXd origRHipAngles=bioutils::consistent3DofJcsAngles(R_Pelvis_to_N,R_RFemur_to_N); // returned as [flexion,adduction,externalRot]
    double currentHeuristicVal=yawAngleSearchHeuristic(maxHipFlexLimit, minHipFlexLimit, maxHipAddLimit, minHipAddLimit, maxHipExtLimit, minHipExtLimit, origRHipAngles);
    if(currentHeuristicVal==0){ // is good! we don't to search
        rhipPass=true;
        std::cout<<"original RThighImu pose passed heuristic check, no changes made."<<std::endl;
    }
    double totalYawRotation=0;
    std::vector<double> rhipHeuristicSearchVals(numIterationsPerSearch);
    std::vector<gtsam::Values> rhipHeuristicSearchGtsamValues(numIterationsPerSearch);
    for(uint k=0; k<numIterationsPerSearch; k++){
        // updates values
        imuPoseEstimator::yawImuStatesByConstantValueInPlace(vals, headingAngleDeltaRad, m_rthighImuPoseProblem.m_poseKeys, m_rthighImuPoseProblem.m_velKeys); // we only want to rotation the right thigh imu
        // now compute orientations and joint angles
        R_Pelvis_to_N=bioutils::get_R_Pelvis_to_N(imuPoseEstimator::vectorizeOrientations(vals,m_sacrumImuPoseProblem.m_poseKeys),approxUp,vals.at<gtsam::Point3>(m_sacrumImuToRHipCtrKey),vals.at<gtsam::Point3>(m_sacrumImuToLHipCtrKey));
        R_RFemur_to_N=bioutils::get_R_Segment_to_N(imuPoseEstimator::vectorizeOrientations(vals,m_rthighImuPoseProblem.m_poseKeys),vals.at<gtsam::Unit3>(m_rkneeAxisThighKey),vals.at<gtsam::Point3>(m_rthighImuToHipCtrKey),vals.at<gtsam::Point3>(m_rthighImuToKneeCtrKey));
        // () compute RHip Angles
        Eigen::MatrixXd newRHipAngles=bioutils::consistent3DofJcsAngles(R_Pelvis_to_N,R_RFemur_to_N); // returned as [flexion,adduction,externalRot]
        // ()error metric: pray at least one of these iterations has all six angles in bounds
        currentHeuristicVal=yawAngleSearchHeuristic(maxHipFlexLimit, minHipFlexLimit, maxHipAddLimit, minHipAddLimit, maxHipExtLimit, minHipExtLimit, newRHipAngles);
        if(currentHeuristicVal==0){ // is good! we can stop searching
            std::cout<<"found! k="<<k<<std::endl;
            rhipPass=true;
        }
        //if(verbose){std::cout<<"     iteration "<<k<<" (graph err="<<m_graph.error(vals)<<"): [max/min] of each angle: flexion: ["<<maxFlex<<","<<minFlex<<"] adduction: ["<<maxAdd<<","<<minAdd<<"] ext rot: ["<<maxExt<<","<<minExt<<"]"<<std::endl;}
        totalYawRotation+=headingAngleDeltaRad;
    }
    if(!rhipPass){
        std::cout<<"could not find a set of rthigh imu yaw which produced in-bounds RHip angles. Values are unaffected (they've been rotated "<<totalYawRotation*180.0/M_PI<<" deg)"<<std::endl;
    }
    // ----------- right knee -----------
    std::cout<<"right knee..."<<std::endl;
    bool rkneePass=false;
    // () get segment systems -> N orientation
    R_RFemur_to_N=bioutils::get_R_Segment_to_N(imuPoseEstimator::vectorizeOrientations(vals,m_rthighImuPoseProblem.m_poseKeys),vals.at<gtsam::Unit3>(m_rkneeAxisThighKey),vals.at<gtsam::Point3>(m_rthighImuToHipCtrKey),vals.at<gtsam::Point3>(m_rthighImuToKneeCtrKey));
    std::vector<gtsam::Rot3> R_RTibia_to_N=bioutils::get_R_Segment_to_N(imuPoseEstimator::vectorizeOrientations(vals,m_rshankImuPoseProblem.m_poseKeys),vals.at<gtsam::Unit3>(m_rkneeAxisShankKey),vals.at<gtsam::Point3>(m_rshankImuToKneeCtrKey),vals.at<gtsam::Point3>(m_rshankImuToAnkleCtrKey));
    // () compute RKnee Angles
    Eigen::MatrixXd origRKneeAngles=bioutils::consistent3DofJcsAngles(R_RFemur_to_N,R_RTibia_to_N); // returned as [flexion,adduction,externalRot]
    currentHeuristicVal=yawAngleSearchHeuristic(maxKneeFlexLimit, minKneeFlexLimit, maxKneeAddLimit, minKneeAddLimit, maxKneeExtLimit, minKneeExtLimit, origRKneeAngles);
    if(currentHeuristicVal==0){ // is good! we can stop searching
        rkneePass=true;
        std::cout<<"original RShankImu pose passed heuristic check, no changes made."<<std::endl;
    }
    totalYawRotation=0;
    std::vector<double> rkneeHeuristicSearchVals(numIterationsPerSearch);
    std::vector<double> rkneeTotalYawRot(numIterationsPerSearch);
    for(uint k=0; k<numIterationsPerSearch; k++){
        // updates values
        imuPoseEstimator::yawImuStatesByConstantValueInPlace(vals, headingAngleDeltaRad, m_rshankImuPoseProblem.m_poseKeys, m_rshankImuPoseProblem.m_velKeys); // we only want to rotation the right shank imu
        // now compute orientations and joint angles
        R_RFemur_to_N=bioutils::get_R_Segment_to_N(imuPoseEstimator::vectorizeOrientations(vals,m_rthighImuPoseProblem.m_poseKeys),vals.at<gtsam::Unit3>(m_rkneeAxisThighKey),vals.at<gtsam::Point3>(m_rthighImuToHipCtrKey),vals.at<gtsam::Point3>(m_rthighImuToKneeCtrKey));
        R_RTibia_to_N=bioutils::get_R_Segment_to_N(imuPoseEstimator::vectorizeOrientations(vals,m_rshankImuPoseProblem.m_poseKeys),vals.at<gtsam::Unit3>(m_rkneeAxisShankKey),vals.at<gtsam::Point3>(m_rshankImuToKneeCtrKey),vals.at<gtsam::Point3>(m_rshankImuToAnkleCtrKey));
        // () compute RKnee Angles
        Eigen::MatrixXd newRKneeAngles=bioutils::consistent3DofJcsAngles(R_RFemur_to_N,R_RTibia_to_N); // returned as [flexion,adduction,externalRot]
        // ()error metric: pray at least one of these iterations has all six angles in bounds
        currentHeuristicVal=yawAngleSearchHeuristic(maxKneeFlexLimit, minKneeFlexLimit, maxKneeAddLimit, minKneeAddLimit, maxKneeExtLimit, minKneeExtLimit, newRKneeAngles);
        if(currentHeuristicVal==0){ // is good! we can stop searching
            std::cout<<"found! k="<<k<<std::endl;
            rkneePass=true;
        }
        //if(verbose){std::cout<<"     iteration "<<k<<" (graph err="<<m_graph.error(vals)<<"): [max/min] of each angle: flexion: ["<<maxFlex<<","<<minFlex<<"] adduction: ["<<maxAdd<<","<<minAdd<<"] ext rot: ["<<maxExt<<","<<minExt<<"]"<<std::endl;}
        totalYawRotation+=headingAngleDeltaRad;
        // put in vector for output
        rkneeHeuristicSearchVals[k]=currentHeuristicVal;
        rkneeTotalYawRot[k]=totalYawRotation;
    }
    if(!rkneePass){
        // revert to whatever the minimum value for heuristic was by going back to that total yaw rotation
        std::cout<<"could not find a set of rshank imu yaw which produced in-bounds RKnee angles. Values are unaffected (they've been rotated "<<totalYawRotation*180.0/M_PI<<" deg)"<<std::endl;
    }

    double newGraphErr=m_graph.error(vals);
    std::cout<<" complete! ("<<(clock()-beginTic)/CLOCKS_PER_SEC<<" sec)   graph error change: "<<origGraphErr<<" --> "<<newGraphErr<<std::endl;
}


double lowerBodyPoseEstimator::yawAngleSearchHeuristic(double maxFlexLimit, double minFlexLimit, double maxAddLimit, double minAddLimit, double maxExtLimit, double minExtLimit, const Eigen::MatrixXd& jointAngles){
    // assumes joint angles are ordered: [flex,add,ext rot]
    bool isGood=false;
    double maxFlex=jointAngles.col(0).maxCoeff();
    double minFlex=jointAngles.col(0).minCoeff();
    double maxAdd=jointAngles.col(1).maxCoeff();
    double minAdd=jointAngles.col(1).minCoeff();
    double maxExt=jointAngles.col(2).maxCoeff();
    double minExt=jointAngles.col(2).minCoeff();
    double maxFlexOver=0;
    double minFlexOver=0;
    double maxAddOver=0;
    double minAddOver=0;
    double maxExtOver=0;
    double minExtOver=0;
    if(maxFlex>maxFlexLimit){ maxFlexOver=maxFlex-maxFlexLimit; }
    if(minFlex<minFlexLimit){ minFlexOver=minFlexLimit-minFlex; }
    if(maxAdd>maxAddLimit){ maxAddOver=maxAdd-maxAddLimit; }
    if(minAdd<minAddLimit){ minAddOver=minAddLimit-minAdd; }
    if(maxExt>maxExtLimit){ maxExtOver=maxExt-maxExtLimit; }
    if(minExt<minExtLimit){ minExtOver=minExtLimit-minExt; }
    double heuristicValue=maxFlexOver+minFlexOver+maxAddOver+minAddOver+maxExtOver+minExtOver; // sum of the overs
    std::cout<<"maxFlex="<<maxFlex<<"(limit="<<maxFlexLimit<<"), minFlex="<<minFlex<<"(limit="<<minFlexLimit<<"), maxAdd="<<maxAdd<<"(limit="<<maxAddLimit<<"), minAdd="<<minAdd<<"(limit="<<minAddLimit<<"), maxExt="<<maxExt<<"(limit="<<maxExtLimit<<"), minExt="<<minExt<<"(limit="<<minExtLimit<<"): total heuristic value="<<heuristicValue<<std::endl;
    return heuristicValue;
}

gtsam::Rot3 lowerBodyPoseEstimator::getDistalImuOrientationAdjustmentGivenDesiredIntExtRotAngle(const gtsam::Rot3& R_ProxSeg_to_N, const gtsam::Rot3& R_DistalImu_to_N, const double& desiredIntExtRotAng, const gtsam::Unit3& rightAxisDistal, const gtsam::Point3& distalImuToProxJointCtr, const gtsam::Point3& distalImuToDistalJointCtr, const bool& checkAngleAtEnd){
    // given the proximal segment orientation, reorient the distal IMU orientation to produce the desired consistent JCS int/ext rot angle
    // INPUTS:
    //    (insert info here) assumes all orientations are [B->N]
    // OUTPUTS:
    //    R_newDistalImu_to_N: R[B2->N], the new distal imu orientation which produces the desired angle
    //    R_B_to_B2: R[B->B2], the incremental rotation applied to B in order to produce the new orientation, i.e., satisfies R[B2->N]=(R[B->B2]*R[B->N]')'
    if(abs(desiredIntExtRotAng)>M_PI){throw std::runtime_error("cannot request angle with absolute value greater than pi, but you requested angle = "+std::to_string(desiredIntExtRotAng));}
    // () find original R_DistalSeg_to_N
    gtsam::Rot3 R_DistalSeg_to_N=bioutils::get_R_Segment_to_N(R_DistalImu_to_N,rightAxisDistal,distalImuToProxJointCtr, distalImuToDistalJointCtr);
    // () find original angle
    Eigen::RowVector3d jointAnglesOrig=bioutils::consistent3DofJcsAngles(R_ProxSeg_to_N, R_DistalSeg_to_N);
    // () determine delta rotation necessary to produce desired angle
    // desiredAng = origAng + deltaAng => deltaAng=desiredAng-origAng
    double spinAngle=desiredIntExtRotAng-jointAnglesOrig(2);
    gtsam::Point3 proxVecDistalFrame=(distalImuToProxJointCtr-distalImuToDistalJointCtr).normalized(); // spin axis
    gtsam::Rot3 R_B_to_B2=gtsam::Rot3::AxisAngle(proxVecDistalFrame,-spinAngle);
    // () find new distal imu orientation, R[B2->N]
    gtsam::Rot3 R_newDistalImu_to_N=R_DistalImu_to_N*R_B_to_B2.inverse(); // apply adjustment: R[B2->N]=R[B->N]*R[B->B2]'
    // () optional: double check that when applied, this R_B_to_B2 produces desired angle
    if(checkAngleAtEnd){
        // () get new distal segment orientation
        gtsam::Rot3 R_newDistalSeg_to_N=bioutils::get_R_Segment_to_N(R_newDistalImu_to_N,rightAxisDistal,distalImuToProxJointCtr, distalImuToDistalJointCtr);
        // () get new angle
        Eigen::RowVector3d jointAnglesNew=bioutils::consistent3DofJcsAngles(R_ProxSeg_to_N, R_newDistalSeg_to_N);
        double intExtRotAngleRadNew=jointAnglesNew(2);
        // () assert it's what you asked for
        if(abs(desiredIntExtRotAng-intExtRotAngleRadNew)>1.0e-3){
            throw std::runtime_error("angle out of bounds! ang difference is "+std::to_string(abs(desiredIntExtRotAng-intExtRotAngleRadNew)));
        }
    }
    // return deltaRot in case you need it
    return R_B_to_B2;
}

gtsam::Rot3 lowerBodyPoseEstimator::getDistalImuOrientationAdjustmentGivenDesiredAbAdAngle(const gtsam::Rot3& R_ProxSeg_to_N, const gtsam::Rot3& R_DistalImu_to_N, const double& desiredAbAdAng, const gtsam::Unit3& rightAxisDistal, const gtsam::Point3& distalImuToProxJointCtr, const gtsam::Point3& distalImuToDistalJointCtr, const bool& checkAngleAtEnd){
    // given the proximal segment orientation, reorient the distal IMU orientation to produce the desired ab/ad rot angle (consistent JCS)
    // INPUTS:
    //    (insert info here) assumes all orientations are [B->N]
    // OUTPUTS:
    //    R_newDistalImu_to_N: R[B2->N], the new distal imu orientation which produces the desired angle
    //    R_B_to_B2: R[B->B2], the incremental rotation applied to B in order to produce the new orientation, i.e., satisfies R[B2->N]=(R[B->B2]*R[B->N]')'
    if(abs(desiredAbAdAng)>M_PI/2.0){throw std::runtime_error("cannot request ab/ad angle with absolute value greater than pi/2, but you requested angle = "+std::to_string(desiredAbAdAng));}
    // () find original R_DistalSeg_to_N
    gtsam::Rot3 R_DistalSeg_to_N=bioutils::get_R_Segment_to_N(R_DistalImu_to_N,rightAxisDistal,distalImuToProxJointCtr, distalImuToDistalJointCtr);
    // () find original angle
    Eigen::RowVector3d jointAnglesOrig=bioutils::consistent3DofJcsAngles(R_ProxSeg_to_N, R_DistalSeg_to_N);
    // () determine delta rotation necessary to produce desired angle
    // the actual rotation axis of ab/ad according to Grood is e2=cross(e3,e1); so we need to construct e2 in the distal frame and rotate about that.
    Eigen::Vector3d INav=R_ProxSeg_to_N*Eigen::Vector3d(1.0,0.0,0.0); // I in nav frame
    Eigen::Vector3d kNav=R_DistalSeg_to_N*Eigen::Vector3d(0.0,0.0,1.0); // k in nav frame
    Eigen::Vector3d e1Nav=INav.normalized(); // e1 = proximal segment x, assumed to the subject's right
    Eigen::Vector3d e3Nav=kNav.normalized(); // e3 = distal segment z, assumed to be proximal
    Eigen::Vector3d e2Nav=(e3Nav.cross(e1Nav)).normalized();// e2 in nav frame. e2 floats, defined from cross product. if subject were at zero angles, e2 would point anterior
    Eigen::Vector3d e2DistImuFrame=R_DistalImu_to_N.inverse()*e2Nav;
    // desiredAng = origAng + deltaAng => deltaAng=desiredAng-origAng
    double spinAngle=desiredAbAdAng-jointAnglesOrig(1);
    gtsam::Rot3 R_B_to_B2=gtsam::Rot3::AxisAngle(e2DistImuFrame,spinAngle); // NOTE: for other DOF, this is -spinAngle (because you're applying it to the distal IMU frame). This is an inconsistency with my angle definition; need to figure this out.
    // () find new distal imu orientation, R[B2->N]
    gtsam::Rot3 R_newDistalImu_to_N=R_DistalImu_to_N*R_B_to_B2.inverse(); // apply adjustment: R[B2->N]=R[B->N]*R[B->B2]'
    // () optional: double check that when applied, this R_B_to_B2 produces desired angle
    if(checkAngleAtEnd){
        // () get new distal segment orientation
        gtsam::Rot3 R_newDistalSeg_to_N=bioutils::get_R_Segment_to_N(R_newDistalImu_to_N,rightAxisDistal,distalImuToProxJointCtr, distalImuToDistalJointCtr);
        // () get new angle
        Eigen::RowVector3d jointAnglesNew=bioutils::consistent3DofJcsAngles(R_ProxSeg_to_N, R_newDistalSeg_to_N);
        double abAdAngleNew=jointAnglesNew(1);
        // () assert it's what you asked for
        if(abs(desiredAbAdAng-abAdAngleNew)>1.0e-3){
            throw std::runtime_error("angle out of bounds! ang difference is "+std::to_string(abs(desiredAbAdAng-abAdAngleNew)));
        }
    }
    // return deltaRot in case you need it
    return R_B_to_B2;
}

gtsam::Rot3 lowerBodyPoseEstimator::getDistalImuOrientationAdjustmentGivenDesiredFlexExAngle(const gtsam::Rot3& R_ProxSeg_to_N, const gtsam::Rot3& R_ProxImu_to_N, const gtsam::Rot3& R_DistalImu_to_N, const double& desiredFlexExAng, const gtsam::Unit3& rightAxisProximal,const gtsam::Unit3& rightAxisDistal, const gtsam::Point3& distalImuToProxJointCtr, const gtsam::Point3& distalImuToDistalJointCtr, const bool& checkAngleAtEnd){
    // given the proximal segment orientation, reorient the distal IMU orientation to produce the desired flex/ex angle (consistent JCS)
    // INPUTS:
    //    (insert info here) assumes all orientations are [B->N]
    // OUTPUTS:
    //    R_newDistalImu_to_N: R[B2->N], the new distal imu orientation which produces the desired angle
    //    R_B_to_B2: R[B->B2], the incremental rotation applied to B in order to produce the new orientation, i.e., satisfies R[B2->N]=(R[B->B2]*R[B->N]')'
    if(abs(desiredFlexExAng)>M_PI){throw std::runtime_error("cannot request angle with absolute value greater than pi, but you requested angle = "+std::to_string(desiredFlexExAng));}
    // () find original R_DistalSeg_to_N
    gtsam::Rot3 R_DistalSeg_to_N=bioutils::get_R_Segment_to_N(R_DistalImu_to_N,rightAxisDistal,distalImuToProxJointCtr, distalImuToDistalJointCtr);
    // () find original angle
    Eigen::RowVector3d jointAnglesOrig=bioutils::consistent3DofJcsAngles(R_ProxSeg_to_N, R_DistalSeg_to_N);
    // () determine delta rotation necessary to produce desired angle
    gtsam::Unit3 kneeAxisProxInDistalFrame=R_DistalImu_to_N.inverse()*R_ProxImu_to_N*rightAxisProximal; // vA[B]= R_B_to_N'*R_A_to_N*vA[A]
    // desiredAng = origAng + deltaAng => deltaAng=desiredAng-origAng
    double spinAngle=desiredFlexExAng-jointAnglesOrig(0);
    gtsam::Rot3 R_B_to_B2=gtsam::Rot3::AxisAngle(kneeAxisProxInDistalFrame,-spinAngle);
    // () find new distal imu orientation, R[B2->N]
    gtsam::Rot3 R_newDistalImu_to_N=R_DistalImu_to_N*R_B_to_B2.inverse(); // apply adjustment: R[B2->N]=R[B->N]*R[B->B2]'
    // () optional: double check that when applied, this R_B_to_B2 produces desired angle
    if(checkAngleAtEnd){
        // () get new distal segment orientation
        gtsam::Rot3 R_newDistalSeg_to_N=bioutils::get_R_Segment_to_N(R_newDistalImu_to_N,rightAxisDistal,distalImuToProxJointCtr, distalImuToDistalJointCtr);
        // () get new angle
        Eigen::RowVector3d jointAnglesNew=bioutils::consistent3DofJcsAngles(R_ProxSeg_to_N, R_newDistalSeg_to_N);
        double flexExAngleNew=jointAnglesNew(0);
        // () assert it's what you asked for
        if(abs(desiredFlexExAng-flexExAngleNew)>1.0e-3){
            throw std::runtime_error("angle out of bounds! ang difference is "+std::to_string(abs(desiredFlexExAng-flexExAngleNew)));
        }
    }
    // return deltaRot in case you need it
    return R_B_to_B2;
}
