// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

/*
 * lowerBodyPoseEstimator implements estimation routines for the full lower-body IMU-kinematics system
 *    for the following human-IMU experimental setup:
 *    7 IMUs total on the lower body: both feet, both lower legs, both thighs, and the lower back (aka sacrum)
 *    note: it is assumed we know which segments each IMU is attached to, but *not* the pose of the IMU on the segment
 */

#pragma once

#include "imuPoseEstimator.h"

class lowerBodyPoseEstimator {
public:
    // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // %% ------------ options for modeling and solving ------------- %%%
    // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // ----------------- biomechanical model options ----------------- //
    bool m_useRFemurLengthFactor=true, m_useRTibiaLengthFactor=true, m_useLFemurLengthFactor=true, m_useLTibiaLengthFactor=true, m_usePelvicWidthFactor=true;
    bool m_useMaxAnthroConstraint=true, m_useMinAnthroConstraint=true;
    bool m_useJointVelFactor=false;
    bool m_useFemurLengthDiscrepancyFactor=true, m_useTibiaLengthDiscrepancyFactor=true; // constrain length discrepancy of total leg? or just femur? just tibia?
    bool m_useKneeAxisTibiaOrthogonalityFactor=true; // constrain the knee axis in the shank IMU frame and the tibia segment vector to be orthogonal?
    bool m_useKneeAxisFemurOrthogonalityFactor=true; // constrain the knee axis in the thigh IMU frame and the femur segment vector to be quasi-orthogonal?
    bool m_assumeHipHinge=true, m_assumeAnkleHinge=true; // assume the hip and/or ankle is a perfect hinge and estimate its hinge axis?
    uint m_initializationScheme=0; // 0 for nothing, 1 to apply sacrum IMU's yaw angle to other IMUs, 2 for yaw search of reasonable joint angles, 3 is debug based on the IMU internal precession angles (doesn't change values)
    // ----------------- model anthropometry settings ---------------- //
    // Note: in the setup() method in MATLAB, these values are changed according to the gender option, looked up from anthro databases.
    //    here, they're hardcoded for all gender, since the cpp implementation doesn't have this anthro lookup functionality.
    char m_gender='a'; // all/any: 'a', or 'm' for male, 'f' for female. used for anthropometric modeling.
    double m_rfemurLength=0.3943, m_rtibiaLength=0.4110, m_lfemurLength=0.3943, m_ltibiaLength=0.4110,  m_pelvicWidth=0.1866; // meters
    double m_rfemurLengthSigma=0.0302, m_rtibiaLengthSigma=0.0264, m_lfemurLengthSigma=0.0302, m_ltibiaLengthSigma=0.0264, m_pelvicWidthSigma=0.0098; // std, meters
    // --- distributions for angle b/w knee axis and femur/tibia segments --- //
    // assumed that knee axis points right. Values come from Hollister et al. (1993), and are adjusted as detailed in McGrath & Stirling (2020).
    double m_angBwKneeAxisAndSegmentMeanRFemur=84.0*M_PI/180.0, m_angBwKneeAxisAndSegmentStdFemur=2.4*M_PI/180.0; // mean and std [radians]
    double m_angBwKneeAxisAndSegmentMeanLFemur=96.0*M_PI/180.0; // mean [radians] (since std is same for both R and L, we don't duplicate the variable)
    double m_angBwKneeAxisAndSegmentMeanRTibia=92.0*M_PI/180.0, m_angBwKneeAxisAndSegmentStdTibia=1.2*M_PI/180.0; // mean and std [radians]
    double m_angBwKneeAxisAndSegmentMeanLTibia=88.0*M_PI/180.0; // mean [radians] (since std is same for both R and L, we don't duplicate the variable)
    // ------------------ other model noise options ------------------- //
    uint m_jointPosFactorDim=3, m_hingeAxisFactorDim=3, m_jointVelFactorDim=3; // dimensionality of relevant factors. 3: vector error model, 1: norm error model
    gtsam::Vector3 m_hipJointCtrConnectionNoise=gtsam::Vector3(9.135, 16.517 ,29.378)*1.0e-2*1.2; // std, meters
    gtsam::Vector3 m_kneeJointCtrConnectionNoise=gtsam::Vector3(6.7348 ,12.3006 ,10.1090)*1.0e-2*1.2; // std, meters
    gtsam::Vector3 m_ankleJointCtrConnectionNoise=gtsam::Vector3(7.3741, 6.2212, 4.6993)*1.0e-2*1.2; // std, meters
    gtsam::Vector3 m_hipHingeAxisNoise= gtsam::Vector3(1.0, 1.0, 1.0) * 20.0; // std, rad/s (only relevant if m_assumeHipHinge=true)
    gtsam::Vector3 m_kneeHingeAxisNoise= gtsam::Vector3(1.0, 1.0, 1.0) * 10.0; // std, rad/s
    gtsam::Vector3 m_ankleHingeAxisNoise= gtsam::Vector3(1.0, 1.0, 1.0) * 20.0; // std, rad/s (only relevant if m_assumeAnkleHinge=true)
    double m_segmentLengthMaxNoise=1.0e-3, m_segmentLengthMinNoise=1.0e-3;
    gtsam::Vector3 m_angVelNoise=gtsam::Vector3(0.000157079633, 0.000157079633 ,0.000157079633)*0.5; // note: this is the default Opal v2 gyro noise density
    gtsam::Vector3 m_jointCtrVelConnectionNoise=gtsam::Vector3(1.0, 1.0, 1.0)*10.0; // m/s
    double m_femurLengthDiscrepancyNoise=0.002, m_tibiaLengthDiscrepancyNoise=0.002; // std, meters
    // -------------------- model prior settings --------------------- //
    bool m_usePriorsOnKneeHingeAxes=true;
    bool m_usePriorsOnHipHingeAxes=true; // (only relevant if m_assumeHipHinge=true)
    bool m_usePriorsOnAnkleHingeAxes=true; // (only relevant if m_assumeAnkleHinge=true)
    bool m_usePriorsOnImuToJointCtrVectors=true;
    gtsam::Unit3 priorKneeAxisRThigh=gtsam::Unit3(0.05, 0.05, 0.95), priorKneeAxisRShank=gtsam::Unit3(-0.05, 0.05, 0.95), priorKneeAxisLThigh=gtsam::Unit3(0.05, 0.05, -0.95), priorKneeAxisLShank=gtsam::Unit3(-0.05, 0.05, -0.95); // default values for knee axis prior means
    gtsam::Vector2 rkneeHingeAxisThighPriorStd=gtsam::Vector2(0.15,0.15), rkneeHingeAxisShankPriorStd=gtsam::Vector2(0.15,0.15), lkneeHingeAxisThighPriorStd=gtsam::Vector2(0.15,0.15), lkneeHingeAxisShankPriorStd=gtsam::Vector2(0.15,0.15); // if m_usePriorsOnImuToJointCtrVectors=true, this is the noise std on the prior Unit3, assume they all use the same std
    gtsam::Unit3 priorHipAxisSacrum=gtsam::Unit3(0.05,0.95,0.05), priorRHipAxisThigh=gtsam::Unit3(-0.05,0.05,0.95), priorLHipAxisThigh=gtsam::Unit3(-0.05,0.05,-0.95); // default values for hip axis prior means (only relevant if m_assumeHipHinge=true)
    gtsam::Vector2 hipHingeAxisSacrumPriorStd=gtsam::Vector2(0.5,0.5), rhipHingeAxisThighPriorStd=gtsam::Vector2(0.5,0.5), lhipHingeAxisThighPriorStd=gtsam::Vector2(0.5,0.5);
    gtsam::Unit3 priorAnkleAxisRFoot=gtsam::Unit3(0.05,-0.95,-0.05), priorAnkleAxisLFoot=gtsam::Unit3(-0.05,-0.95,0.05), priorAnkleAxisRShank=gtsam::Unit3(0.05,0.05,0.95), priorAnkleAxisLShank=gtsam::Unit3(0.05,0.05,-0.95);  // default values for ankle axis prior means (if m_assumeAnkleHinge=true)
    gtsam::Vector2 rankleHingeAxisFootPriorStd=gtsam::Vector2(0.15,0.15), lankleHingeAxisFootPriorStd=gtsam::Vector2(0.15,0.15), rankleHingeAxisShankPriorStd=gtsam::Vector2(0.15,0.15), lankleHingeAxisShankPriorStd=gtsam::Vector2(0.15,0.15); // default values for ankle axis prior stdss (m_assumeAnkleHinge=true)
    gtsam::Point3 priorSacrumImuToRHipCtr=gtsam::Point3(.026,.135,-.215);
    gtsam::Point3 priorSacrumImuToLHipCtr=gtsam::Point3(.024,-.132,-.213);
    gtsam::Point3 priorRThighImuToKneeCtr=gtsam::Point3(.26,-1.0e-5,-1.0e-5);
    gtsam::Point3 priorRThighImuToHipCtr=gtsam::Point3(-.24,1.0e-5,1.0e-5);
    gtsam::Point3 priorRShankImuToKneeCtr=gtsam::Point3(-.11,1.0e-5,1.0e-5);
    gtsam::Point3 priorRShankImuToAnkleCtr=gtsam::Point3(.31,.11,1.0e-5);
    gtsam::Point3 priorRFootImuToAnkleCtr=gtsam::Point3(-.051,-1.0e-5,-.05);
    gtsam::Point3 priorLThighImuToKneeCtr=gtsam::Point3(.26,-1.0e-5,-1.0e-5);
    gtsam::Point3 priorLThighImuToHipCtr=gtsam::Point3(-.24,1.0e-5,1.0e-5);
    gtsam::Point3 priorLShankImuToKneeCtr=gtsam::Point3(-.1,1.0e-5,1.0e-5);
    gtsam::Point3 priorLShankImuToAnkleCtr=gtsam::Point3(.29,.09,-1.0e-5);
    gtsam::Point3 priorLFootImuToAnkleCtr=gtsam::Point3(-.049,1.0e-5,-.055);
    gtsam::Vector3 imuToJointCtrPriorStd=gtsam::Vector3(0.5,0.5,0.5); // std, m
    // ---------------------- solver settings ------------------------ //
    uint m_optimizerType=1; // 0 for Gauss Newton, 1 for Levenberg-Marquardt
    double m_absErrDecreaseLimit=1.0e-6, m_relErrDecreaseLimit=1.0e-5, m_absErrLimit=1.0e-5;
    uint m_maxIterations=10000;
    double m_lambdaUpperBound=1.0e25, m_lambdaLowerBound=1.0e-10, m_lambdaInitial=1.0e-5, m_lambdaFactor=10.0; // LM settings
    uint m_convergeAtConsecSmallIter=6; // how many consecutive "small" iterations must occur for the solver to exit? (1 -> common optimizaton; first time you hit the convergence constraints the solver exits)
    // -- restart optimization threshold for hip I/E drift -- //
    double m_hipIeAngDriftRestartThreshold=100.0*M_PI/180.0/30.0; // [rad/s]: a post-optimization regression slope larger than this threshold will straighten the angle and restart optimization
    uint m_maxNumRestarts=100, m_nRestarts=0;
    std::string m_linearSolverType="MULTIFRONTAL_CHOLESKY"; // choose MULTIFRONTAL_CHOLESKY, MULTIFRONTAL_QR, SEQUENTIAL_CHOLESKY, SEQUENTIAL_QR
    // --------------------------------------------------------------- //
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //%% --------------- named variables, do not edit -------------- %%%
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // general variables for graphs, values, id, and individual pose problems
    std::string m_strId; // string identifier of this problem. used for printing convenience.
    gtsam::NonlinearFactorGraph m_graph; // NonlinearFactorGraph
    gtsam::Values m_initialValues, m_estimate;
    imuPoseEstimator m_rthighImuPoseProblem, m_rshankImuPoseProblem, m_rfootImuPoseProblem, m_lthighImuPoseProblem, m_lshankImuPoseProblem, m_lfootImuPoseProblem, m_sacrumImuPoseProblem;
    // variable strings and chars
    unsigned char m_rthighImuToKneeCtrVarChar, m_rshankImuToKneeCtrVarChar; std::string m_rthighImuToKneeCtrVarStr, m_rshankImuToKneeCtrVarStr; // static offset vector to right knee Ctr
    unsigned char m_rthighImuToHipCtrVarChar, m_sacrumImuToRHipCtrVarChar; std::string m_rthighImuToHipCtrVarStr, m_sacrumImuToRHipCtrVarStr; // static offset vector to right hip Ctr
    unsigned char m_rfootImuToAnkleCtrVarChar, m_rshankImuToAnkleCtrVarChar; std::string m_rfootImuToAnkleCtrVarStr, m_rshankImuToAnkleCtrVarStr; // static offset vector to right ankle Ctr
    unsigned char m_lthighImuToKneeCtrVarChar, m_lshankImuToKneeCtrVarChar; std::string m_lthighImuToKneeCtrVarStr, m_lshankImuToKneeCtrVarStr; // static offset vector to left knee Ctr
    unsigned char m_lthighImuToHipCtrVarChar, m_sacrumImuToLHipCtrVarChar; std::string m_lthighImuToHipCtrVarStr, m_sacrumImuToLHipCtrVarStr; // static offset vector to left hip Ctr
    unsigned char m_lfootImuToAnkleCtrVarChar, m_lshankImuToAnkleCtrVarChar; std::string m_lfootImuToAnkleCtrVarStr, m_lshankImuToAnkleCtrVarStr; // static offset vector to left ankle center
    unsigned char m_lkneeAxisThighVarChar, m_lkneeAxisShankVarChar; std::string m_lkneeAxisThighVarStr, m_lkneeAxisShankVarStr; // knee axis var char/strs
    unsigned char m_rkneeAxisThighVarChar, m_rkneeAxisShankVarChar; std::string m_rkneeAxisThighVarStr, m_rkneeAxisShankVarStr; // knee axis var char/strs
    unsigned char m_hipAxisSacrumVarChar, m_rhipAxisThighVarChar, m_lhipAxisThighVarChar; std::string m_hipAxisSacrumVarStr, m_rhipAxisThighVarStr, m_lhipAxisThighVarStr; // hip axis var char/strs (if m_assumeHipHinge=true)
    unsigned char m_lankleAxisShankVarChar, m_lankleAxisFootVarChar; std::string m_lankleAxisShankVarStr, m_lankleAxisFootVarStr; // ankle axis var char/strs (only relevant if m_assumeAnkleHinge=true)
    unsigned char m_rankleAxisShankVarChar, m_rankleAxisFootVarChar; std::string m_rankleAxisShankVarStr, m_rankleAxisFootVarStr; // ankle axis var char/strs (only relevant if m_assumeAnkleHinge=true)
    unsigned char m_sacrumImuAngVelVarChar, m_rthighImuAngVelVarChar, m_rshankImuAngVelVarChar, m_rfootImuAngVelVarChar, m_lthighImuAngVelVarChar, m_lshankImuAngVelVarChar, m_lfootImuAngVelVarChar;
    std::string m_sacrumImuAngVelVarStr, m_rthighImuAngVelVarStr, m_rshankImuAngVelVarStr, m_rfootImuAngVelVarStr, m_lthighImuAngVelVarStr, m_lshankImuAngVelVarStr, m_lfootImuAngVelVarStr;
    // output (estimated) states
    std::vector<gtsam::Pose3> m_sacrumImuPose, m_rthighImuPose, m_rshankImuPose, m_rfootImuPose, m_lthighImuPose, m_lshankImuPose, m_lfootImuPose;
    std::vector<gtsam::Rot3> m_sacrumImuOrientation, m_rthighImuOrientation, m_rshankImuOrientation, m_rfootImuOrientation, m_lthighImuOrientation, m_lshankImuOrientation, m_lfootImuOrientation;
    std::vector<gtsam::Point3> m_sacrumImuPosition, m_rthighImuPosition, m_rshankImuPosition, m_rfootImuPosition, m_lthighImuPosition, m_lshankImuPosition, m_lfootImuPosition;
    std::vector<gtsam::Vector3> m_sacrumImuVelocity, m_rthighImuVelocity, m_rshankImuVelocity, m_rfootImuVelocity, m_lthighImuVelocity, m_lshankImuVelocity, m_lfootImuVelocity;
    std::vector<gtsam::Vector3> m_sacrumImuGyroBias, m_rthighImuGyroBias, m_rshankImuGyroBias, m_rfootImuGyroBias, m_lthighImuGyroBias, m_lshankImuGyroBias, m_lfootImuGyroBias;
    std::vector<gtsam::Vector3> m_sacrumImuAccelBias, m_rthighImuAccelBias, m_rshankImuAccelBias, m_rfootImuAccelBias, m_lthighImuAccelBias, m_lshankImuAccelBias, m_lfootImuAccelBias;
    gtsam::Unit3 m_rkneeAxisThigh,m_rkneeAxisShank, m_lkneeAxisThigh,m_lkneeAxisShank; // knee axes, Unit3
    gtsam::Unit3 m_hipAxisSacrum,m_rhipAxisThigh, m_lhipAxisThigh; // hip axes, Unit3 (only relevant if m_assumeHipHinge=true)
    gtsam::Unit3 m_rankleAxisShank, m_rankleAxisFoot, m_lankleAxisShank, m_lankleAxisFoot; // ankle axes (if m_assumeAnkleHinge=true)
    gtsam::Point3 m_sacrumImuToRHipCtr, m_rthighImuToHipCtr, m_rthighImuToKneeCtr, m_rshankImuToKneeCtr, m_rshankImuToAnkleCtr, m_rfootImuToAnkleCtr; // static offset vectors, Point3
    gtsam::Point3 m_sacrumImuToLHipCtr, m_lthighImuToHipCtr, m_lthighImuToKneeCtr, m_lshankImuToKneeCtr, m_lshankImuToAnkleCtr, m_lfootImuToAnkleCtr; // static offset vectors, Point3
    std::vector<double> m_time;
    std::vector<gtsam::Vector3> m_sacrumImuAngVel,m_rthighImuAngVel,m_rshankImuAngVel,m_rfootImuAngVel,m_lthighImuAngVel,m_lshankImuAngVel,m_lfootImuAngVel;
    Eigen::VectorXd m_rkneeFlexExAng, m_rkneeIntExtRotAng, m_rkneeAbAdAng, m_lkneeFlexExAng, m_lkneeIntExtRotAng, m_lkneeAbAdAng; // knee angles, radians
    Eigen::VectorXd m_rhipFlexExAng, m_rhipIntExtRotAng, m_rhipAbAdAng, m_lhipFlexExAng, m_lhipIntExtRotAng, m_lhipAbAdAng; // hip angles, radians
    // named noise models for construction
    gtsam::SharedNoiseModel m_kneeJointCtrConnectionNoiseModel, m_hipJointCtrConnectionNoiseModel, m_ankleJointCtrConnectionNoiseModel;
    gtsam::SharedNoiseModel m_kneeHingeAxisNoiseModel, m_hipHingeAxisNoiseModel, m_ankleHingeAxisNoiseModel; // hinge kinematics noise models
    gtsam::SharedNoiseModel m_rfemurLengthNoiseModel, m_rtibiaLengthNoiseModel, m_lfemurLengthNoiseModel, m_ltibiaLengthNoiseModel, m_pelvicWidthNoiseModel;
    gtsam::SharedNoiseModel m_angVelNoiseModel;
    gtsam::SharedNoiseModel m_jointCtrVelConnectionNoiseModel;
    gtsam::SharedNoiseModel m_segmentLengthMaxNoiseModel, m_segmentLengthMinNoiseModel;
    gtsam::SharedNoiseModel m_femurLengthDiscrepancyNoiseModel, m_tibiaLengthDiscrepancyNoiseModel;
    gtsam::SharedNoiseModel m_angBwKneeAxisAndSegmentFemurNoiseModel, m_angBwKneeAxisAndSegmentTibiaNoiseModel;
    // keys
    gtsam::Key m_rkneeAxisThighKey, m_rkneeAxisShankKey, m_lkneeAxisThighKey, m_lkneeAxisShankKey; // knee axes
    gtsam::Key m_hipAxisSacrumKey, m_rhipAxisThighKey, m_lhipAxisThighKey; // hip axes (only relevant if m_assumeHipHinge=true)
    gtsam::Key m_rankleAxisShankKey, m_rankleAxisFootKey, m_lankleAxisShankKey, m_lankleAxisFootKey; // ankle axes (only relevant if m_assumeAnkleHinge=true)
    gtsam::Key m_sacrumImuToRHipCtrKey, m_rthighImuToHipCtrKey, m_rthighImuToKneeCtrKey, m_rshankImuToKneeCtrKey, m_rshankImuToAnkleCtrKey, m_rfootImuToAnkleCtrKey; // static offset vectors, Point3
    gtsam::Key m_sacrumImuToLHipCtrKey, m_lthighImuToHipCtrKey, m_lthighImuToKneeCtrKey, m_lshankImuToKneeCtrKey, m_lshankImuToAnkleCtrKey, m_lfootImuToAnkleCtrKey; // static offset vectors, Point3
    gtsam::KeyVector m_sacrumImuAngVelKeys, m_rthighImuAngVelKeys, m_rshankImuAngVelKeys, m_rfootImuAngVelKeys, m_lthighImuAngVelKeys, m_lshankImuAngVelKeys, m_lfootImuAngVelKeys;
    // internal procession angles. mostly for debug purposes
    Eigen::VectorXd m_sacrumImuInternalPrecessionAng,m_rthighImuInternalPrecessionAng, m_rshankImuInternalPrecessionAng, m_rfootImuInternalPrecessionAng, m_lthighImuInternalPrecessionAng, m_lshankImuInternalPrecessionAng, m_lfootImuInternalPrecessionAng;
    // misc
    std::vector<double> m_optimizationTotalError, m_optimizationTotalTime;
    std::string m_initializationFile;
    // -----

    explicit lowerBodyPoseEstimator(const imuPoseEstimator& sacrumImuPoseProblem, const imuPoseEstimator& rthighImuPoseProblem, const imuPoseEstimator& rshankImuPoseProblem, const imuPoseEstimator& rfootImuPoseProblem,
                                    const imuPoseEstimator& lthighImuPoseProblem, const imuPoseEstimator& lshankImuPoseProblem, const imuPoseEstimator& lfootImuPoseProblem, const std::string& id);
    lowerBodyPoseEstimator() = default; // trivial constructor
    // --- methods to setup factor graph and add factors to it --- //
    void setup();
    void addImuProblemGraphsToGraph();
    void addInstantaneousAngVelEstimationToImuPoses(uint initMode=2);
    void add6JointSharedPosConstraints();
    void add6JointSharedVelConstraints();
    void addHipHingeConstraints();
    void addKneeHingeConstraints();
    void addAnkleHingeConstraints();
    // ----------------------------------------------------------- //
    // --- solution methods --- //
    void defaultOptimize();
    void fastOptimize(double prevGlobalErr=9e9);
    void robustOptimize();
    // ------------------------ //
    void addImuProblemSolutionsToInitialValues();
    void printNoiseModelInfo(); // prints relevant information about constituent noise models
    void addHipHingeAxisVariableKeys();
    void addAnkleHingeAxisVariableKeys();
    void setValuesFromFile(gtsam::Values& vals, const std::string& file);

    void setMemberEstimatedStatesFromValues(const gtsam::Values& vals);
    void printStaticStatesSummary();
    void saveResultsToH5File(const std::string& filename) const;
    int testJointCenterVelocityAsDiscretePositionDerivative(double errorTol=5.0e3, bool printMatricesToCsv=false);
    void setNoiseModelsFromMemberNoiseVariables();
    void setDerivedJointAnglesFromEstimatedStates(bool verbose=true);
    bool postHocAxesCheck(bool verbose=false);
    void setAllInternalPrecessionAngles();
    // methods to adapt priors
    void addNewPriorSacrumImuToRHipCtr(const gtsam::Point3& newPriorValue, const double& newPriorStd);
    void addNewPriorSacrumImuToLHipCtr(const gtsam::Point3& newPriorValue, const double& newPriorStd);
    void addNewPriorRThighImuToHipCtr(const gtsam::Point3& newPriorValue, const double& newPriorStd);
    void addNewPriorLThighImuToHipCtr(const gtsam::Point3& newPriorValue, const double& newPriorStd);
    void addNewPoint3PriorToKey(const gtsam::Key& key, const gtsam::Point3& newPriorValue, const double& newPriorStd);
    // get methods
    std::vector<double> getRKneeFlexExAngle() const; // radians
    std::vector<double> getLKneeFlexExAngle() const; // radians
    // heading debug/correction methods
    void adjustLegsForCorrectedHipIeAngles(gtsam::Values& vals, bool verbose);
    void printYawSlopesAndInterceptsFromMemberOrientations() const;
    void alignYawAnglesOfAllImusToSacrumImu(gtsam::Values& vals); // edit values in place
    void setImuOrientationsBasedOnJointAngleLimits(gtsam::Values& vals,bool zeroMedianCenterIntExtRot=true, bool zeroMedianCenterAbAd=true, bool verbose=false);
    void setAllImuVelocitiesAsImuPositionDerivatives(gtsam::Values& vals);
    static void correctImuPosesForConnectedJointCenters(const gtsam::Pose3& sacrumImuPose, gtsam::Pose3& thighImuPose, gtsam::Pose3& shankImuPose, gtsam::Pose3& footImuPose, const gtsam::Point3& sacrumImuToHipCtr, const gtsam::Point3& thighImuToHipCtr, const gtsam::Point3& thighImuToKneeCtr, const gtsam::Point3& shankImuToKneeCtr, const gtsam::Point3& shankImuToAnkleCtr, const gtsam::Point3& footImuToAnkleCtr);
    static void adjustDistalImuTrajectoryForInboundJointAngles(const std::vector<gtsam::Rot3>& R_ProxSeg_to_N,const std::vector<gtsam::Pose3>& proxImuPose, const gtsam::Unit3& kneeAxisProx, std::vector<gtsam::Pose3>& distalImuPose, std::vector<gtsam::Vector3>& distalImuVel, const gtsam::Point3& distalImuToProxJointCtr, const gtsam::Point3& distalImuToDistalJointCtr,const gtsam::Unit3& rightAxisDistal,const gtsam::Vector2& flexExMaxMin,const gtsam::Vector2& abAdMaxMin,const gtsam::Vector2& intExtRotMaxMin,bool zeroMedianCenterIntExtRot=true, bool zeroMedianCenterAbAd=true);
    void correctImuOrientationsForZeroHipIeSlope(gtsam::Values& vals, bool verbose=true, uint iterNum=0);
    void computeHipIeRotAngleRegressionSlopesFromValues(const gtsam::Values& vals, double& rhipIeIntExtRotSlope, double& lhipIntExtRotSlope);
    void setImuPositionsBasedOnConsistentInitialStaticVecsToJointCtrs(gtsam::Values& vals); // edit values in place
    static void adjustDistalImuPosBasedOnStaticVecsToJointCtr(const gtsam::Pose3& proximalImuPose,gtsam::Pose3& distalImuPose,const gtsam::Point3& proximalImuToJointCtr,const gtsam::Point3& distalImuToJointCtr);
    static void adjustDistalImuPosBasedOnStaticVecsToJointCtr(const std::vector<gtsam::Pose3>& proximalImuPose,std::vector<gtsam::Pose3>& distalImuPose,const gtsam::Point3& proximalImuToJointCtr,const gtsam::Point3& distalImuToJointCtr);
    void searchYawAnglesForInboundsJointAngles(gtsam::Values& vals, bool verbose=true); // edit values in place
    bool testImusCanBeSpunWithOnlyChangeToIntExtRotAngle(const gtsam::NonlinearFactorGraph& origgraph, const gtsam::Values& origvals, bool verbose=false) const;
    static double yawAngleSearchHeuristic(double maxFlexLimit, double minFlexLimit, double maxAddLimit, double minAddLimit, double maxExtLimit, double minExtLimit, const Eigen::MatrixXd& jointAngles);
    static gtsam::Rot3 getDistalImuOrientationAdjustmentGivenDesiredIntExtRotAngle(const gtsam::Rot3& R_ProxSeg_to_N, const gtsam::Rot3& R_DistalImu_to_N, const double& desiredIntExtRotAng, const gtsam::Unit3& rightAxisDistal, const gtsam::Point3& distalImuToProxJointCtr, const gtsam::Point3& distalImuToDistalJointCtr, const bool& checkAngleAtEnd=true);
    static gtsam::Rot3 getDistalImuOrientationAdjustmentGivenDesiredFlexExAngle(const gtsam::Rot3& R_ProxSeg_to_N, const gtsam::Rot3& R_ProxImu_to_N, const gtsam::Rot3& R_DistalImu_to_N, const double& desiredFlexExAng, const gtsam::Unit3& rightAxisProximal,const gtsam::Unit3& rightAxisDistal, const gtsam::Point3& distalImuToProxJointCtr, const gtsam::Point3& distalImuToDistalJointCtr, const bool& checkAngleAtEnd=true);
    static gtsam::Rot3 getDistalImuOrientationAdjustmentGivenDesiredAbAdAngle(const gtsam::Rot3& R_ProxSeg_to_N, const gtsam::Rot3& R_DistalImu_to_N, const double& desiredAbAdAng, const gtsam::Unit3& rightAxisDistal, const gtsam::Point3& distalImuToProxJointCtr, const gtsam::Point3& distalImuToDistalJointCtr, const bool& checkAngleAtEnd=true);
};
