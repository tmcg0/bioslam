// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

/* imuPoseEstimator provides estimation of IMU pose, velocity, and bias from input IMU masurements */

#pragma once

#include "imu/imu.h"
#include <string>
#include <vector>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/CombinedImuFactor.h>

class imuPoseEstimator {

public:
    // ++++++++++++++++ SETTINGS ++++++++++++++++ //
    // --- model settings --- //
    uint m_numMeasToPreint=10; // number of IMU measurements to preintegrate
    bool m_useMagnetometer=false; // option: use magnetometer in estimation?
    bool m_usePositionPrior=true; // option: set a prior on the first position keyframe? if true, it will use means and stds set below.
    bool m_useVelocityPrior=true; // option: set a prior on the first velocity keyframe? if true, it will use means and stds set below.
    bool m_useImuBiasPrior=true; // option: set a prior on the first imu bias keyframe? if true, it will use means and stds set below.
    bool m_useCompassPrior=false; // option: set a prior on compass heading? if true, it will use means and stds set below.
    Eigen::Vector3d m_globalB=Eigen::Vector3d(30.5, 14.91, -56.3); // default magnetic field in nav frame. {30.5, 14.91, -56.3} is a decent value for a NWU frame
    Eigen::Vector3d m_globalAcc=Eigen::Vector3d(0.0,0.0,-9.81); // default acceleration in nav frame
    Eigen::Vector3d m_priorOrientation=Eigen::Vector3d(1.0e-8,1.0e-8,1.0e-8); // mean, rad so(3) "rotations around Z, Y, then X axes as in http://en.wikipedia.org/wiki/Rotation_matrix, counterclockwise when looking from unchanging axis."
    Eigen::Vector3d m_priorPosition=Eigen::Vector3d(1.0e-8,1.0e-8,1.0e-8); // mean, m
    Eigen::Vector3d m_priorVelocity=Eigen::Vector3d(1.0e-8,1.0e-8,1.0e-8); // mean, m/s
    Eigen::Vector3d m_priorAccelBias=Eigen::Vector3d(1.0e-8,1.0e-8,1.0e-8); // mean, [m/s^2]
    Eigen::Vector3d m_priorGyroBias=Eigen::Vector3d(1.0e-8,1.0e-8,1.0e-8); // mean, [rad/s]
    double m_priorCompassAngle=0.0;
    Eigen::Vector3d m_priorAccelBiasConstantNoise=Eigen::Vector3d(1.0e-3,1.0e-3,1.0e-3); // std, [m/s^2]
    Eigen::Vector3d m_priorGyroBiasConstantNoise=Eigen::Vector3d(1.0e-3,1.0e-3,1.0e-3); // std, [rad/s]
    Eigen::Vector3d m_priorOrientationNoise=Eigen::Vector3d(1.0e3,1.0e3,1.0e3); // std, rad
    Eigen::Vector3d m_priorPositionNoise=Eigen::Vector3d(1.0e-3,1.0e-3,1.0e-3); // std, m
    Eigen::Vector3d m_priorVelocityNoise=Eigen::Vector3d(1.0e-3,1.0e-3,1.0e-3); // std, m/s
    double m_priorCompassAngleNoise=1.0e-3;
    Eigen::Vector3d m_magnetometerNoise=Eigen::Vector3d(3.0,3.0,3.0); // std, uT
    uint m_initializationScheme=0; // 0 for just starting at zero, 1 for forward/backward EKF on orientation (1 still starts positions, velocities, and biases at zero)
    // compass system settings
    Eigen::Vector3d m_refVecLocal=Eigen::Vector3d(1.0,0.0,0.0), m_refVecNav=Eigen::Vector3d(1.0,0.0,0.0);
    // ---------------------- //
    // --- solver settings --- //
    double m_relErrDecreaseLimit=1.0e-8; // convergence criteria for relative decrease in error
    double m_absErrDecreaseLimit=1.0e-10; // convergence criteria for absolute decrease in error
    double m_absErrLimit=1.0e-12; // total error limit
    uint m_maxIterations=5000; // maximum number of iterations
    uint m_optType=1; // 0 for Gauss-Newton, 1 for Levenberg-Marquardt
    // ----------------------- //
    gtsam::NonlinearFactorGraph m_graph;
    gtsam::Values m_estimate, m_initialValues;
    imu m_imu;
    std::string m_strId; // string identifier, makes unique factorgraph key sets
    // IMU states, typically set from an optimization routine
    std::vector<gtsam::Pose3> m_pose;
    std::vector<gtsam::Rot3> m_orientation;
    std::vector<gtsam::Point3> m_position;
    std::vector<gtsam::Vector3> m_velocity, m_accelBias, m_gyroBias;
    std::vector<double> m_time; // estimated time domain
    // marginals and variances
    Eigen::MatrixXd m_orientationPrincipalVariance3, m_positionPrincipalVariance3;
    // variable characters, strings, and keys
    unsigned char m_poseVarChar, m_velVarChar, m_imuBiasVarChar;
    std::string m_poseVarStr, m_velVarStr, m_imuBiasVarStr;
    gtsam::KeyVector m_poseKeys, m_velKeys, m_imuBiasKeys;
    // named noise models
    gtsam::SharedNoiseModel m_magnetometerNoiseModel;
    // variables to store debugging data for preintegration
    std::vector<uint> m_beginPreintIdxInImuData, m_endPreintIdxInImuData; // the indeces in the imu data corresponding to the beginning/end of each preintegration period
    std::vector<gtsam::Vector3> m_gyroCombinations;
    std::vector<gtsam::PreintegratedCombinedMeasurements> m_preintImus;
    std::vector<gtsam::Point3> m_downsampledMagMeas;
    // misc.
    uint m_nKeyframes; // number of keyframes in this imu graph
    std::vector<double> m_optimizationTotalError, m_optimizationTotalTime; // for tracking optimization performance
    // ++++++++++++++++++++++++++++++++++++++++++ //
    explicit imuPoseEstimator(const imu& imuIn, const std::string& id); //construct from a single IMU and its unique string id
    explicit imuPoseEstimator()=default; // default constructor
    // setup methods and values setting methods
    void setup();
    void setupImuFactorStaticBias();
    void setupCombinedImuFactor();
    void setOptimizedValuesToInitialValues();
    void setNoiseModelsFromMemberNoiseVariables();
    void setAnyRequestedPriors(bool verbose=true);
    uint getImuBiasModelMode(bool printVerbose);
    void setImuBiasModelMode(uint modelMode);
    void setMemberStatesFromValues(const gtsam::Values& vals);
    std::tuple<std::vector<gtsam::Pose3>,gtsam::Matrix,std::vector<gtsam::imuBias::ConstantBias>> getInitializationData() const; // get initialization data as a tuple of <pose,velocity,imubias>
    // solver methods
    void defaultOptimize();
    void fastOptimize();
    void robustOptimize(bool printIntermediateResultsToFile=false);
    // other estimation methods
    static gtsam::Rot3 singleSampleDavenportQMethod(const Eigen::Vector3d& accelBody, const Eigen::Vector3d& magBody, const Eigen::Vector3d& accelNWU, const Eigen::Vector3d& magNWU, double wa, double wm);
    // methods to get error states
    double getMagFactorErrorAtState(const gtsam::Values& vals);
    double getGenericImuFactorErrorAtState(const gtsam::Values& vals);
    double getImuFactorErrorAtState(const gtsam::Values& vals);
    double getCombinedImuFactorErrorAtState(const gtsam::Values& vals);
    // static methods to vectorize member values into their native types
    static std::vector<gtsam::Pose3> vectorizePoses(const gtsam::Values& vals, const gtsam::KeyVector& poseKeys);
    static std::vector<gtsam::Rot3> vectorizeOrientations(const gtsam::Values& vals, const gtsam::KeyVector& poseKeys);
    static std::vector<gtsam::Rot3> vectorizeOrientations(const std::vector<gtsam::Pose3>& poses);
    static std::vector<gtsam::Point3> vectorizePositions(const gtsam::Values& vals, const gtsam::KeyVector& poseKeys);
    static std::vector<gtsam::Point3> vectorizePositions(const std::vector<gtsam::Pose3>& poses);
    static std::vector<gtsam::Vector3> vectorizeVelocities(const gtsam::Values& vals, const gtsam::KeyVector& velKeys);
    static std::vector<gtsam::Vector3> vectorizeGyroBiases(const gtsam::Values& vals, const gtsam::KeyVector& biasKeys);
    static std::vector<gtsam::Vector3> vectorizeAccelBiases(const gtsam::Values& vals, const gtsam::KeyVector& biasKeys);
    // --- methods for marginals and variances --- //
    Eigen::MatrixXd getOrientationPrincipalVariance();
    Eigen::MatrixXd getPositionPrincipalVariance();
    // general utilities
    std::vector<uint> getImuIndecesCorrespondingToKeyframes() const;
    uint getExpectedNumberOfKeyframesFromImuData() const;
    void clear(); // clear any estimation results stored
    // methods for recalculating/debugging IMU states
    std::vector<gtsam::Vector3> getImuLinearAccel() const;
    static void adjustImuTrajectoryByDeltaRot(std::vector<gtsam::Pose3>& imuPoseTrajectory, std::vector<gtsam::Vector3>& imuVelTrajectory, const gtsam::Rot3& deltaRot);
    static void adjustImuTrajectoryByDeltaRot(std::vector<gtsam::Pose3>& imuPoseTrajectory, std::vector<gtsam::Vector3>& imuVelTrajectory, const std::vector<gtsam::Rot3>& deltaRot);
    static void adjustImuTrajectoryByDeltaRot(gtsam::Pose3& imuPoseTrajectory, gtsam::Vector3& imuVelTrajectory, const gtsam::Rot3& deltaRot);
    static std::vector<gtsam::Vector3> consistentVelocitySetFromPositions(const std::vector<gtsam::Pose3>& poses, const double& dt);
    static std::vector<gtsam::Vector3> consistentVelocitySetFromPositions(const std::vector<gtsam::Point3>& pos, const double& dt);
    // --- some print utilities --- //
    void print();
    void printYprToFile(std::string filename) const;
    void printQuatToFile(std::string filename) const;
    void printDcmToFile(std::string filename) const;
    void printPosToFile(std::string filename) const;
    void printAccelBiasToFile(std::string filename) const;
    void printGyroBiasToFile(std::string filename) const;
    void printResultsToConsole() const;
    static void robustOptimizePrintInitialState(const int& nIndentSpaces, double& initImuFactorErr);
    void robustOptimizePrintImuBias(const gtsam::Values& vals, int nSpacesToIndent); // printing helper function for robustOptimize()
    // test/debug methods
    void printStateAtKeyframe(const gtsam::Values& vals, uint k, const std::string& tag= "");
    bool testHeadingCanBeSpunWithNoChangeToError(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& vals, bool verbose=false) const;
    static void yawImuStatesByConstantValueInPlace(gtsam::Values& vals, const double& headingAngleDeltaRad, const gtsam::KeyVector& poseKeys, const gtsam::KeyVector& velKeys);
    static gtsam::Values yawImuStatesByConstantValueWithCopy(const gtsam::Values& vals, const double& headingAngleDeltaRad, const gtsam::KeyVector& poseKeys, const gtsam::KeyVector& velKeys);
protected:
    uint imuBiasModelMode=0; // 0 for imufactor with static bias. 1 for combined imu factor with standard (dynamic) bias.

}; // end classdef