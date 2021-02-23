% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

classdef lowerBodyPoseEstimator < handle
    %LOWERBODYPOSEESTIMATOR Skeletal pose estimation for the 7-IMU lower body problem
    %   IMUs placed on the lower back, thighs, shanks, and feet
    %   Class computes pose trajectory of IMUs + skeletal pose parameters
    properties
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% ------------ options for modeling and solving ------------- %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % model options
        m_useRFemurLengthFactor=1, m_useRTibiaLengthFactor=1, m_useLFemurLengthFactor=1, m_useLTibiaLengthFactor=1, m_usePelvicWidthFactor=1
        m_useMaxAnthroConstraint=1, m_useMinAnthroConstraint=1
        m_useJointVelFactor=0
        m_useFemurLengthDiscrepancyFactor=1, m_useTibiaLengthDiscrepancyFactor=1 % constrain leg length discrepancy or individual segment length discrepancies?
        m_useKneeAxisTibiaOrthogonalityFactor=1; % constrain the knee axis in the shank IMU frame and the tibia segment vector to be quasi-orthogonal?
        m_useKneeAxisFemurOrthogonalityFactor=1; % constrain the knee axis in the thigh IMU frame and the femur segment vector to be quasi-orthogonal?
        m_assumeHipHinge=1; % assume the hip is a perfect hinge and estimate its hinge axis?
        m_assumeAnkleHinge=1; % assume the ankle is a perfect hinge and estimate its hinge axis?
        m_initializationScheme=0;
        % // ----------------- model anthropometry settings ---------------- //
        % Note: in the setup() method, these values are changed according to the gender option, looked up from anthro databases.
        m_gender='a'; % all/any: 'a', or 'm' for male, 'f' for female. used for anthropometric modeling.
        m_rfemurLength=0.3943, m_rtibiaLength=0.4110, m_lfemurLength=0.3943, m_ltibiaLength=0.4110, m_pelvicWidth=0.4; % m
        m_rfemurLengthSigma=0.01, m_rtibiaLengthSigma=0.01, m_lfemurLengthSigma=0.01, m_ltibiaLengthSigma=0.01, m_pelvicWidthSigma=0.01; % std, m
        % // --- distributions for angle b/w knee axis and femur/tibia segments --- //
        % assumed that knee axis points right. Values come from Hollister et al. (1993), and are adjusted as detailed in McGrath & Stirling (2020).
        m_angBwKneeAxisAndSegmentMeanRFemur=deg2rad(84.0); m_angBwKneeAxisAndSegmentStdFemur=deg2rad(2.4); % mean and std [radians]
        m_angBwKneeAxisAndSegmentMeanLFemur=deg2rad(96.0); % mean [radians] (since std is same for both R and L, we don't duplicate the variable)
        m_angBwKneeAxisAndSegmentMeanRTibia=deg2rad(92.0); m_angBwKneeAxisAndSegmentStdTibia=deg2rad(1.2); % mean and std [radians]
        m_angBwKneeAxisAndSegmentMeanLTibia=deg2rad(88.0); % mean [radians] (since std is same for both R and L, we don't duplicate the variable)
        % noise values
        m_kneeJointCtrConnectionNoise=[6.7348 12.3006 10.1090]*1e-2*1.2; % std, meters
        m_hipJointCtrConnectionNoise=[9.135 16.517 29.378]*1e-2*1.2; % std, meters
        m_ankleJointCtrConnectionNoise=[7.3741 6.2212 4.6993]*1e-2*1.2; % std, meters
        m_kneeHingeAxisNoise=[1 1 1]*10.0; % std, rad/s
        m_hipHingeAxisNoise=[1 1 1]*10.0; % std, rad/s (only relevant if m_assumeHipHinge=true)
        m_ankleHingeAxisNoise=[1 1 1]*10.0; % std, rad/s (only relevant if m_assumeAnkleHinge=true)
        m_segmentLengthMaxNoise=1e-3, m_segmentLengthMinNoise=1e-3;
        m_angVelNoise=[0.000157079633 0.000157079633 0.000157079633]; % note: this is the default Opal v2 gyro noise density
        m_jointCtrVelConnectionNoise=[1 1 1]*1e-2; % m/s
        m_femurLengthDiscrepancyNoise=0.008; % std, meters (source: https://libres.uncg.edu/ir/uncg/f/S_Shultz_Bilateral_2007.pdf)
        m_tibiaLengthDiscrepancyNoise=0.006; % std, meters (source: https://libres.uncg.edu/ir/uncg/f/S_Shultz_Bilateral_2007.pdf)
        %// -------------------- model prior settings --------------------- //
        m_setInitialStaticVectorsFromFile=''; % if not empty, overrides these initial values with file
        m_usePriorsOnKneeHingeAxes=1;
        m_usePriorsOnHipHingeAxes=1; % (only relevant if m_assumeHipHinge=true)
        m_usePriorsOnAnkleHingeAxes=1; % (only relevant if m_assumeAnkleHinge=true)
        m_usePriorsOnImuToJointCtrVectors=0; % if you intend to add calibration, recommended to turn this off (the cal will add the priors)
        priorAxisRThigh=gtsam.Unit3(gtsam.Point3(0.,0.,1.)), priorAxisRShank=gtsam.Unit3(gtsam.Point3(0.,0.,1.)), priorAxisLThigh=gtsam.Unit3(gtsam.Point3(0.,0.,-1.)), priorAxisLShank=gtsam.Unit3(gtsam.Point3(0.,0.,-1.)); % default values for knee axis prior means
        rkneeHingeAxisThighPriorStd=[0.5,0.5]', rkneeHingeAxisShankPriorStd=[0.5,0.5]', lkneeHingeAxisThighPriorStd=[0.5,0.5]', lkneeHingeAxisShankPriorStd=[0.5,0.5]'; % if m_usePriorsOnImuToJointCtrVectors=true, this is the noise std on the prior Unit3, assume they all use the same std
        priorHipAxisSacrum=gtsam.Unit3(gtsam.Point3(0.,1.,0.)); priorRHipAxisThigh=gtsam.Unit3(gtsam.Point3(0.,0.,1.)), priorLHipAxisThigh=gtsam.Unit3(gtsam.Point3(0.,0.,-1.)); % default values for hip axis prior means (only relevant if m_assumeHipHinge=true)
        hipHingeAxisSacrumPriorStd=[0.5,0.5]'; rhipHingeAxisThighPriorStd=[0.5,0.5]', lhipHingeAxisThighPriorStd=[0.5,0.5]';
        priorAnkleAxisRFoot=gtsam.Unit3(gtsam.Point3(0,-1,0)); priorAnkleAxisLFoot=gtsam.Unit3(gtsam.Point3(0,-1,0)); priorAnkleAxisRShank=gtsam.Unit3(gtsam.Point3(0.0,0.0,1.0)); priorAnkleAxisLShank=gtsam.Unit3(gtsam.Point3(0.0,0.0,-1.0));
        rankleHingeAxisFootPriorStd=[0.5,0.5]'; lankleHingeAxisFootPriorStd=[0.5,0.5]'; rankleHingeAxisShankPriorStd=[0.5,0.5]'; lankleHingeAxisShankPriorStd=[0.5,0.5]'; 
        priorSacrumImuToRHipCtr=gtsam.Point3(.16,.135,-.215);
        priorSacrumImuToLHipCtr=gtsam.Point3(.14,-.132,-.213);
        priorRThighImuToKneeCtr=gtsam.Point3(.26,-1.0e-5,-1.0e-5);
        priorRThighImuToHipCtr=gtsam.Point3(-.24,1.0e-5,1.0e-5);
        priorRShankImuToKneeCtr=gtsam.Point3(-.11,1.0e-5,1.0e-5);
        priorRShankImuToAnkleCtr=gtsam.Point3(.31,.11,1.0e-5);
        priorRFootImuToAnkleCtr=gtsam.Point3(-.051,-1.0e-5,-.05);
        priorLThighImuToKneeCtr=gtsam.Point3(.26,-1.0e-5,-1.0e-5);
        priorLThighImuToHipCtr=gtsam.Point3(-.24,1.0e-5,1.0e-5);
        priorLShankImuToKneeCtr=gtsam.Point3(-.1,1.0e-5,1.0e-5);
        priorLShankImuToAnkleCtr=gtsam.Point3(.29,.09,-1.0e-5);
        priorLFootImuToAnkleCtr=gtsam.Point3(-.049,1.0e-5,-.055);
        imuToJointCtrPriorStd=[0.5,0.5,0.5]'; % std, m
        %// ---------------------- solver settings ------------------------ //
        m_optimizerType=1 % 0 for Gauss Newton, 1 for Levenberg-Marquardt
        m_absErrDecreaseLimit=1e-6; m_relErrDecreaseLimit=1e-5; m_maxIterations=1000;
        m_lambdaUpperBound=1.0e10, m_lambdaLowerBound=1.0e-10, m_lambdaInitial=1.0e-5, m_lambdaFactor=10.0; % LM settings
        m_setMarginals=0; % should you set marginal covariances? Can be time consuming, off by default.
        m_convergeAtConsecSmallIter=6; % how many consecutive "small" iterations must occur for the solver to exit? (1 -> common optimizaton; first time you hit the convergence constraints the solver exits)
        % note: optional methods to change setup
        %     addSternumImu(); % adds an IMU to the sternum, assumed kinematics are that it is rigidly connected to the lumbar IMU
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% --------------- named variables, do not edit -------------- %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % general variables for graphs, values, id, and individual pose problems
        id % string identifier of this problem. used for printing convenience.
        m_graph % NonlinearFactorGraph
        m_initialValues % gtsam.Values
        m_estimate % gtsam.Values
        m_rthighImuPoseProblem, m_rshankImuPoseProblem, m_rfootImuPoseProblem, m_lthighImuPoseProblem, m_lshankImuPoseProblem, m_lfootImuPoseProblem, m_sacrumImuPoseProblem, m_sternumImuPoseProblem
        % variable strings and chars
        m_sacrumPoseVarChar, m_sacrumVelVarChar, m_sacrumImuBiasVarChar, m_sacrumMagBiasVarChar % sacrum IMU var char states
        m_rkneeAxisThighVarChar, m_rkneeAxisShankVarChar, m_rkneeAxisThighVarStr, m_rkneeAxisShankVarStr % knee axis var char/strs
        m_rthighPoseVarChar, m_rthighVelVarChar, m_rthighImuBiasVarChar, m_rthighMagBiasVarChar % thigh IMU var char states
        m_rshankPoseVarChar, m_rshankVelVarChar, m_rshankImuBiasVarChar, m_rshankMagBiasVarChar % shank IMU var char states
        m_rfootPoseVarChar, m_rfootVelVarChar, m_rfootImuBiasVarChar, m_rfootMagBiasVarChar % foot IMU var char states
        m_sternumPoseVarChar, m_sternumVelVarChar, m_sternumImuBiasVarChar, m_sternumMagBiasVarChar % (optional) sternum IMU var char states
        m_sternumImuToSacrumVarChar, m_sacrumImuToSternumVarChar, m_sternumImuToSacrumVarStr, m_sacrumImuToSternumVarStr
        m_rthighImuToKneeCenterVarChar, m_rshankImuToKneeCenterVarChar, m_rthighImuToKneeCenterVarStr, m_rshankImuToKneeCenterVarStr % static offset vector to right knee center
        m_rthighImuToHipCenterVarChar, m_sacrumImuToRHipCenterVarChar, m_rthighImuToHipCenterVarStr, m_sacrumImuToRHipCenterVarStr % static offset vector to right hip center
        m_rfootImuToAnkleCenterVarChar, m_rshankImuToAnkleCenterVarChar, m_rfootImuToAnkleCenterVarStr, m_rshankImuToAnkleCenterVarStr % static offset vector to right ankle center
        m_lthighImuToKneeCenterVarChar, m_lshankImuToKneeCenterVarChar, m_lthighImuToKneeCenterVarStr, m_lshankImuToKneeCenterVarStr % static offset vector to left knee center
        m_lthighImuToHipCenterVarChar, m_sacrumImuToLHipCenterVarChar, m_lthighImuToHipCenterVarStr, m_sacrumImuToLHipCenterVarStr % static offset vector to left hip center
        m_lfootImuToAnkleCenterVarChar, m_lshankImuToAnkleCenterVarChar, m_lfootImuToAnkleCenterVarStr, m_lshankImuToAnkleCenterVarStr % static offset vector to left ankle center
        m_lkneeAxisThighVarChar, m_lkneeAxisShankVarChar, m_lkneeAxisThighVarStr, m_lkneeAxisShankVarStr % knee axis var char/strs
        m_lthighPoseVarChar, m_lthighVelVarChar, m_lthighImuBiasVarChar, m_lthighMagBiasVarChar % thigh IMU var char states
        m_lshankPoseVarChar, m_lshankVelVarChar, m_lshankImuBiasVarChar, m_lshankMagBiasVarChar % shank IMU var char states
        m_lfootPoseVarChar, m_lfootVelVarChar, m_lfootImuBiasVarChar, m_lfootMagBiasVarChar % foot IMU var char states
        m_sacrumImuAngVelVarChar, m_rthighImuAngVelVarChar, m_rshankImuAngVelVarChar, m_rfootImuAngVelVarChar, m_lthighImuAngVelVarChar, m_lshankImuAngVelVarChar, m_lfootImuAngVelVarChar
        m_sacrumImuAngVelVarStr, m_rthighImuAngVelVarStr, m_rshankImuAngVelVarStr, m_rfootImuAngVelVarStr, m_lthighImuAngVelVarStr, m_lshankImuAngVelVarStr, m_lfootImuAngVelVarStr
        m_hipAxisSacrumVarChar, m_hipAxisSacrumVarStr, m_rhipAxisThighVarChar, m_rhipAxisThighVarStr, m_lhipAxisThighVarChar, m_lhipAxisThighVarStr % hip axis var char/strs (only relevant if m_assumeHipHinge=true)
        m_rankleAxisFootVarChar, m_rankleAxisFootVarStr, m_lankleAxisFootVarChar, m_lankleAxisFootVarStr, m_rankleAxisShankVarChar, m_rankleAxisShankVarStr, m_lankleAxisShankVarChar, m_lankleAxisShankVarStr % ankle axis var char/strs (only relevant if m_assumeHipHinge=true)
        % output (estimated) states
        m_sacrumImuPose,m_sacrumImuOrientation,m_sacrumImuPosition,m_sacrumImuVelocity,m_sacrumImuAccelBias,m_sacrumImuGyroBias % sacrum IMU
        m_rthighImuPose,m_rthighImuOrientation,m_rthighImuPosition,m_rthighImuVelocity,m_rthighImuAccelBias,m_rthighImuGyroBias % thigh IMU
        m_rshankImuPose,m_rshankImuOrientation,m_rshankImuPosition,m_rshankImuVelocity,m_rshankImuAccelBias,m_rshankImuGyroBias % shank IMU
        m_rfootImuPose,m_rfootImuOrientation,m_rfootImuPosition,m_rfootImuVelocity,m_rfootImuAccelBias,m_rfootImuGyroBias % foot IMU
        m_lthighImuPose,m_lthighImuOrientation,m_lthighImuPosition,m_lthighImuVelocity,m_lthighImuAccelBias,m_lthighImuGyroBias % thigh IMU
        m_lshankImuPose,m_lshankImuOrientation,m_lshankImuPosition,m_lshankImuVelocity,m_lshankImuAccelBias,m_lshankImuGyroBias % shank IMU
        m_lfootImuPose,m_lfootImuOrientation,m_lfootImuPosition,m_lfootImuVelocity,m_lfootImuAccelBias,m_lfootImuGyroBias % foot IMU
        m_sternumImuOrientation, m_sternumImuPosition, m_sternumImuVelocity, m_sternumImuAccelBias, m_sternumImuGyroBias % (optional) sternum IMU
        m_rkneeAxisThigh,m_rkneeAxisShank, m_lkneeAxisThigh,m_lkneeAxisShank % knee axes, Unit3
        m_sacrumImuToRHipCtr, m_rthighImuToHipCtr, m_rthighImuToKneeCtr, m_rshankImuToKneeCtr, m_rshankImuToAnkleCtr, m_rfootImuToAnkleCtr % static offset vectors, Point3
        m_sacrumImuToLHipCtr, m_lthighImuToHipCtr, m_lthighImuToKneeCtr, m_lshankImuToKneeCtr, m_lshankImuToAnkleCtr, m_lfootImuToAnkleCtr % static offset vectors, Point3
        m_sacrumImuToSternum, m_sternumImuToSacrum
        m_sacrumImuAngVel, m_rthighImuAngVel, m_rshankImuAngVel, m_rfootImuAngVel, m_lthighImuAngVel, m_lshankImuAngVel, m_lfootImuAngVel
        m_hipAxisSacrum, m_rhipAxisThigh, m_lhipAxisThigh
        m_rankleAxisFoot, m_lankleAxisFoot
        m_time % array to hold time, in seconds. Not actually estimated.
        % derived joint angles and segment coordinate systems
        m_rhipFlexExAng, m_rhipIntExtRotAng, m_rhipAbAdAng, m_rkneeFlexExAng, m_rkneeIntExtRotAng, m_rkneeAbAdAng
        m_lhipFlexExAng, m_lhipIntExtRotAng, m_lhipAbAdAng, m_lkneeFlexExAng, m_lkneeIntExtRotAng, m_lkneeAbAdAng
        m_R_Pelvis_to_N, m_R_RFemur_to_N, m_R_RTibia_to_N, m_R_LFemur_to_N, m_R_LTibia_to_N
        % named noise models for construction
        m_kneeJointCtrConnectionNoiseModel, m_hipJointCtrConnectionNoiseModel, m_ankleJointCtrConnectionNoiseModel
        m_kneeHingeAxisNoiseModel, m_hipHingeAxisNoiseModel, m_ankleHingeAxisNoiseModel
        m_rfemurLengthNoiseModel, m_rtibiaLengthNoiseModel, m_lfemurLengthNoiseModel, m_ltibiaLengthNoiseModel, m_pelvicWidthNoiseModel
        m_segmentLengthMaxNoiseModel, m_segmentLengthMinNoiseModel
        m_angVelNoiseModel
        m_jointCtrVelConnectionNoiseModel
        m_femurLengthDiscrepancyNoiseModel, m_tibiaLengthDiscrepancyNoiseModel
        m_angBwKneeAxisAndSegmentFemurNoiseModel, m_angBwKneeAxisAndSegmentTibiaNoiseModel
        % keys
        m_rkneeAxisThighKey, m_rkneeAxisShankKey, m_lkneeAxisThighKey, m_lkneeAxisShankKey % knee axes
        m_sacrumImuToRHipCtrKey, m_rthighImuToHipCtrKey, m_rthighImuToKneeCtrKey, m_rshankImuToKneeCtrKey, m_rshankImuToAnkleCtrKey, m_rfootImuToAnkleCtrKey % static offset vectors, Point3
        m_sacrumImuToLHipCtrKey, m_lthighImuToHipCtrKey, m_lthighImuToKneeCtrKey, m_lshankImuToKneeCtrKey, m_lshankImuToAnkleCtrKey, m_lfootImuToAnkleCtrKey % static offset vectors, Point3
        m_sternumImuToSacrumKey, m_sacrumImuToSternumKey
        m_sacrumImuAngVelKeys, m_rthighImuAngVelKeys, m_rshankImuAngVelKeys, m_lthighImuAngVelKeys, m_lshankImuAngVelKeys, m_rfootImuAngVelKeys, m_lfootImuAngVelKeys
        m_hipAxisSacrumKey, m_rhipAxisThighKey, m_lhipAxisThighKey
        m_rankleAxisFootKey, m_lankleAxisFootKey, m_rankleAxisShankKey, m_lankleAxisShankKey
        % marginals and (co)variances
        m_marginals
        m_sacrumImuOrientationPrincipalVariance, m_sacrumImuPositionPrincipalVariance, m_sacrumImuVelocityPrincipalVariance, m_sacrumImuGyroBiasPrincipalVariance, m_sacrumImuAccelBiasPrincipalVariance
        m_rthighImuOrientationPrincipalVariance, m_rthighImuPositionPrincipalVariance, m_rthighImuVelocityPrincipalVariance, m_rthighImuGyroBiasPrincipalVariance, m_rthighImuAccelBiasPrincipalVariance
        m_rshankImuOrientationPrincipalVariance, m_rshankImuPositionPrincipalVariance, m_rshankImuVelocityPrincipalVariance, m_rshankImuGyroBiasPrincipalVariance, m_rshankImuAccelBiasPrincipalVariance
        m_rfootImuOrientationPrincipalVariance, m_rfootImuPositionPrincipalVariance, m_rfootImuVelocityPrincipalVariance, m_rfootImuGyroBiasPrincipalVariance, m_rfootImuAccelBiasPrincipalVariance
        m_lthighImuOrientationPrincipalVariance, m_lthighImuPositionPrincipalVariance, m_lthighImuVelocityPrincipalVariance, m_lthighImuGyroBiasPrincipalVariance, m_lthighImuAccelBiasPrincipalVariance
        m_lshankImuOrientationPrincipalVariance, m_lshankImuPositionPrincipalVariance, m_lshankImuVelocityPrincipalVariance, m_lshankImuGyroBiasPrincipalVariance, m_lshankImuAccelBiasPrincipalVariance
        m_lfootImuOrientationPrincipalVariance, m_lfootImuPositionPrincipalVariance, m_lfootImuVelocityPrincipalVariance, m_lfootImuGyroBiasPrincipalVariance, m_lfootImuAccelBiasPrincipalVariance
        m_rkneeAxisThighCovariance, m_rkneeAxisShankCovariance, m_lkneeAxisThighCovariance, m_lkneeAxisShankCovariance
        m_sacrumImuToRHipCtrCovariance, m_rthighImuToHipCtrCovariance, m_rthighImuToKneeCtrCovariance, m_rshankImuToKneeCtrCovariance, m_rshankImuToAnkleCtrCovariance, m_rfootImuToAnkleCtrCovariance
        m_sacrumImuToLHipCtrCovariance, m_lthighImuToHipCtrCovariance, m_lthighImuToKneeCtrCovariance, m_lshankImuToKneeCtrCovariance, m_lshankImuToAnkleCtrCovariance, m_lfootImuToAnkleCtrCovariance
        m_sacrumImuAngVelPrincipalVariance, m_rthighImuAngVelPrincipalVariance, m_rshankImuAngVelPrincipalVariance, m_rfootImuAngVelPrincipalVariance, m_lthighImuAngVelPrincipalVariance, m_lshankImuAngVelPrincipalVariance, m_lfootImuAngVelPrincipalVariance
        % internal precession angles
        m_sacrumImuInternalPrecessionAng, m_rthighImuInternalPrecessionAng, m_rshankImuInternalPrecessionAng, m_rfootImuInternalPrecessionAng, m_lthighImuInternalPrecessionAng, m_lshankImuInternalPrecessionAng, m_lfootImuInternalPrecessionAng
        % optimization results data
        m_optimizationTotalError, m_optimizationTotalTime
        % check behavior variables
        m_isSetup=0, m_allMarginalsSuccessful=0;
    end
    
    methods
        function obj = lowerBodyPoseEstimator(varargin)
            %LOWERBODYPOSEESTIMATOR Construct an instance of this class
            if nargin==0
                % do nothing
            elseif nargin==1
                if lowerBodyPoseEstimator.isValidResultsH5File(varargin{1})
                    constructFromResultsFile(obj,varargin{1});
                else; error('unknown single argument input');
                end
            elseif nargin==8 % normal usage
                % lowerBodyPoseEstimator(sacrumIpe, rthighIpe, rshankIpe, rfootIpe, lthighIpe, lshankIpe, lfootIpe, id)
                if isa(varargin{1},'imuPoseEstimator') && isa(varargin{2},'imuPoseEstimator') && isa(varargin{3},'imuPoseEstimator') && isa(varargin{4},'imuPoseEstimator') && isa(varargin{5},'imuPoseEstimator') && isa(varargin{6},'imuPoseEstimator') && isa(varargin{7},'imuPoseEstimator') && isa(varargin{8},'char')
                    obj.m_sacrumImuPoseProblem=varargin{1}; obj.m_rthighImuPoseProblem = varargin{2}; obj.m_rshankImuPoseProblem=varargin{3}; obj.m_rfootImuPoseProblem=varargin{4}; obj.m_lthighImuPoseProblem = varargin{5}; obj.m_lshankImuPoseProblem=varargin{6}; obj.m_lfootImuPoseProblem=varargin{7}; obj.id=varargin{8};
                    % setup variable strings and characters
                    % sacrum IMU
                    obj.m_sacrumPoseVarChar=obj.m_sacrumImuPoseProblem.m_poseVarChar;
                    obj.m_sacrumVelVarChar=obj.m_sacrumImuPoseProblem.m_velVarChar;
                    obj.m_sacrumImuBiasVarChar=obj.m_sacrumImuPoseProblem.m_imuBiasVarChar;
                    obj.m_sacrumImuToRHipCenterVarStr = strcat(obj.id,'_sacrumImuToRHipCenter');
                    VarStrToCharMap.insert(obj.m_sacrumImuToRHipCenterVarStr);
                    obj.m_sacrumImuToRHipCenterVarChar = VarStrToCharMap.getChar(obj.m_sacrumImuToRHipCenterVarStr);
                    obj.m_sacrumImuToLHipCenterVarStr = strcat(obj.id,'_sacrumImuToLHipCenter');
                    VarStrToCharMap.insert(obj.m_sacrumImuToLHipCenterVarStr);
                    obj.m_sacrumImuToLHipCenterVarChar = VarStrToCharMap.getChar(obj.m_sacrumImuToLHipCenterVarStr);
                    % right thigh IMU
                    obj.m_rthighPoseVarChar=obj.m_rthighImuPoseProblem.m_poseVarChar;
                    obj.m_rthighVelVarChar=obj.m_rthighImuPoseProblem.m_velVarChar;
                    obj.m_rthighImuBiasVarChar=obj.m_rthighImuPoseProblem.m_imuBiasVarChar;
                    obj.m_rkneeAxisThighVarStr=strcat(obj.id,'_rkneeAxisThigh');
                    VarStrToCharMap.insert(obj.m_rkneeAxisThighVarStr);
                    obj.m_rkneeAxisThighVarChar=VarStrToCharMap.getChar(obj.m_rkneeAxisThighVarStr);
                    obj.m_rthighImuToKneeCenterVarStr = strcat(obj.id,'_rthighImuToKneeCenter');
                    VarStrToCharMap.insert(obj.m_rthighImuToKneeCenterVarStr);
                    obj.m_rthighImuToKneeCenterVarChar = VarStrToCharMap.getChar(obj.m_rthighImuToKneeCenterVarStr);
                    obj.m_rthighImuToHipCenterVarStr = strcat(obj.id,'_rthighImuToHipCenter');
                    VarStrToCharMap.insert(obj.m_rthighImuToHipCenterVarStr);
                    obj.m_rthighImuToHipCenterVarChar = VarStrToCharMap.getChar(obj.m_rthighImuToHipCenterVarStr);
                    % right shank IMU
                    obj.m_rshankPoseVarChar=obj.m_rshankImuPoseProblem.m_poseVarChar;
                    obj.m_rshankVelVarChar=obj.m_rshankImuPoseProblem.m_velVarChar;
                    obj.m_rshankImuBiasVarChar=obj.m_rshankImuPoseProblem.m_imuBiasVarChar;
                    obj.m_rkneeAxisShankVarStr=strcat(obj.id,'_rkneeAxisShank');
                    VarStrToCharMap.insert(obj.m_rkneeAxisShankVarStr);
                    obj.m_rkneeAxisShankVarChar=VarStrToCharMap.getChar(obj.m_rkneeAxisShankVarStr);
                    obj.m_rshankImuToKneeCenterVarStr = strcat(obj.id,'_rshankImuToKneeCenter');
                    VarStrToCharMap.insert(obj.m_rshankImuToKneeCenterVarStr);
                    obj.m_rshankImuToKneeCenterVarChar = VarStrToCharMap.getChar(obj.m_rshankImuToKneeCenterVarStr);
                    obj.m_rshankImuToAnkleCenterVarStr = strcat(obj.id,'_rshankImuToAnkleCenter');
                    VarStrToCharMap.insert(obj.m_rshankImuToAnkleCenterVarStr);
                    obj.m_rshankImuToAnkleCenterVarChar = VarStrToCharMap.getChar(obj.m_rshankImuToAnkleCenterVarStr);
                    % right foot IMU
                    obj.m_rfootPoseVarChar=obj.m_rfootImuPoseProblem.m_poseVarChar;
                    obj.m_rfootVelVarChar=obj.m_rfootImuPoseProblem.m_velVarChar;
                    obj.m_rfootImuBiasVarChar=obj.m_rfootImuPoseProblem.m_imuBiasVarChar;
                    obj.m_rfootImuToAnkleCenterVarStr = strcat(obj.id,'_rfootImuToAnkleCenter');
                    VarStrToCharMap.insert(obj.m_rfootImuToAnkleCenterVarStr);
                    obj.m_rfootImuToAnkleCenterVarChar = VarStrToCharMap.getChar(obj.m_rfootImuToAnkleCenterVarStr);
                    % left thigh IMU
                    obj.m_lthighPoseVarChar=obj.m_lthighImuPoseProblem.m_poseVarChar;
                    obj.m_lthighVelVarChar=obj.m_lthighImuPoseProblem.m_velVarChar;
                    obj.m_lthighImuBiasVarChar=obj.m_lthighImuPoseProblem.m_imuBiasVarChar;
                    obj.m_lkneeAxisThighVarStr=strcat(obj.id,'_lkneeAxisThigh');
                    VarStrToCharMap.insert(obj.m_lkneeAxisThighVarStr);
                    obj.m_lkneeAxisThighVarChar=VarStrToCharMap.getChar(obj.m_lkneeAxisThighVarStr);
                    obj.m_lthighImuToKneeCenterVarStr = strcat(obj.id,'_lthighImuToKneeCenter');
                    VarStrToCharMap.insert(obj.m_lthighImuToKneeCenterVarStr);
                    obj.m_lthighImuToKneeCenterVarChar = VarStrToCharMap.getChar(obj.m_lthighImuToKneeCenterVarStr);
                    obj.m_lthighImuToHipCenterVarStr = strcat(obj.id,'_lthighImuToHipCenter');
                    VarStrToCharMap.insert(obj.m_lthighImuToHipCenterVarStr);
                    obj.m_lthighImuToHipCenterVarChar = VarStrToCharMap.getChar(obj.m_lthighImuToHipCenterVarStr);
                    % left shank IMU
                    obj.m_lshankPoseVarChar=obj.m_lshankImuPoseProblem.m_poseVarChar;
                    obj.m_lshankVelVarChar=obj.m_lshankImuPoseProblem.m_velVarChar;
                    obj.m_lshankImuBiasVarChar=obj.m_lshankImuPoseProblem.m_imuBiasVarChar;
                    obj.m_lkneeAxisShankVarStr=strcat(obj.id,'_lkneeAxisShank');
                    VarStrToCharMap.insert(obj.m_lkneeAxisShankVarStr);
                    obj.m_lkneeAxisShankVarChar=VarStrToCharMap.getChar(obj.m_lkneeAxisShankVarStr);
                    obj.m_lshankImuToKneeCenterVarStr = strcat(obj.id,'_lshankImuToKneeCenter');
                    VarStrToCharMap.insert(obj.m_lshankImuToKneeCenterVarStr);
                    obj.m_lshankImuToKneeCenterVarChar = VarStrToCharMap.getChar(obj.m_lshankImuToKneeCenterVarStr);
                    obj.m_lshankImuToAnkleCenterVarStr = strcat(obj.id,'_lshankImuToAnkleCenter');
                    VarStrToCharMap.insert(obj.m_lshankImuToAnkleCenterVarStr);
                    obj.m_lshankImuToAnkleCenterVarChar = VarStrToCharMap.getChar(obj.m_lshankImuToAnkleCenterVarStr);
                    % left foot IMU
                    obj.m_lfootPoseVarChar=obj.m_lfootImuPoseProblem.m_poseVarChar;
                    obj.m_lfootVelVarChar=obj.m_lfootImuPoseProblem.m_velVarChar;
                    obj.m_lfootImuBiasVarChar=obj.m_lfootImuPoseProblem.m_imuBiasVarChar;
                    obj.m_lfootImuToAnkleCenterVarStr = strcat(obj.id,'_lfootImuToAnkleCenter');
                    VarStrToCharMap.insert(obj.m_lfootImuToAnkleCenterVarStr);
                    obj.m_lfootImuToAnkleCenterVarChar = VarStrToCharMap.getChar(obj.m_lfootImuToAnkleCenterVarStr);
                    % set anthro data
                    setAnthroNoiseProperties(obj);
                    % noise models to store as properties
                    setNoiseModelsFromMemberNoiseVariables(obj);
                end
            else
                error('no matching constructor for lowerBodyPoseEstimator');
            end
        end
        
        function setInitialStaticVectorsFromH5File(obj,file)
            % overrides the properties for the initial value of the lbpe
            % also tries to update() their value in the initialValues (in case you had already added them to values)
            obj.priorSacrumImuToRHipCtr=gtsam.Point3(h5read(file,'/SacrumImuToRHipCtr')/1000);
            obj.priorSacrumImuToLHipCtr=gtsam.Point3(h5read(file,'/SacrumImuToLHipCtr')/1000);
            obj.priorRThighImuToKneeCtr=gtsam.Point3(h5read(file,'/RThighImuToRKneeCtr')/1000);
            obj.priorRThighImuToHipCtr=gtsam.Point3(h5read(file,'/RThighImuToRHipCtr')/1000);
            obj.priorRShankImuToKneeCtr=gtsam.Point3(h5read(file,'/RShankImuToRKneeCtr')/1000);
            obj.priorRShankImuToAnkleCtr=gtsam.Point3(h5read(file,'/RShankImuToRAnkleCtr')/1000);
            obj.priorRFootImuToAnkleCtr=gtsam.Point3(h5read(file,'/RFootImuToRAnkleCtr')/1000);
            obj.priorLThighImuToKneeCtr=gtsam.Point3(h5read(file,'/LThighImuToLKneeCtr')/1000);
            obj.priorLThighImuToHipCtr=gtsam.Point3(h5read(file,'/LThighImuToLHipCtr')/1000);
            obj.priorLShankImuToKneeCtr=gtsam.Point3(h5read(file,'/LShankImuToLKneeCtr')/1000);
            obj.priorLShankImuToAnkleCtr=gtsam.Point3(h5read(file,'/LShankImuToLAnkleCtr')/1000);
            obj.priorLFootImuToAnkleCtr=gtsam.Point3(h5read(file,'/LFootImuToLAnkleCtr')/1000);
            fprintf('--- setting initial static vectors from true data file ---\n');
            fprintf('pelvic components (all are added, though):\n');
            fprintf('    sacrum IMU -> RHip: [%.3f %.3f %.3f] m\n',obj.priorSacrumImuToRHipCtr.vector());
            fprintf('    sacrum IMU -> LHip: [%.3f %.3f %.3f] m\n',obj.priorSacrumImuToLHipCtr.vector());
            fprintf('    rthigh IMU -> RHip: [%.3f %.3f %.3f] m\n',obj.priorRThighImuToHipCtr.vector());
            fprintf('    lthigh IMU -> LHip: [%.3f %.3f %.3f] m\n',obj.priorLThighImuToHipCtr.vector());
            try
                obj.m_initialValues.update(obj.m_sacrumImuToRHipCtrKey,obj.priorSacrumImuToRHipCtr);
                obj.m_initialValues.update(obj.m_sacrumImuToLHipCtrKey,obj.priorSacrumImuToLHipCtr);
                obj.m_initialValues.update(obj.m_rthighImuToHipCtrKey,obj.priorRThighImuToHipCtr);
                obj.m_initialValues.update(obj.m_lthighImuToHipCtrKey,obj.priorLThighImuToHipCtr);
                obj.m_initialValues.update(obj.m_rthighImuToKneeCtrKey,obj.priorRThighImuToKneeCtr);
                obj.m_initialValues.update(obj.m_rshankImuToKneeCtrKey,obj.priorRShankImuToKneeCtr);
                obj.m_initialValues.update(obj.m_rshankImuToAnkleCtrKey,obj.priorRShankImuToAnkleCtr);
                obj.m_initialValues.update(obj.m_rfootImuToAnkleCtrKey,obj.priorRFootImuToAnkleCtr);
                obj.m_initialValues.update(obj.m_lthighImuToKneeCtrKey,obj.priorLThighImuToKneeCtr);
                obj.m_initialValues.update(obj.m_lshankImuToKneeCtrKey,obj.priorLShankImuToKneeCtr);
                obj.m_initialValues.update(obj.m_lshankImuToAnkleCtrKey,obj.priorLShankImuToAnkleCtr);
                obj.m_initialValues.update(obj.m_lfootImuToAnkleCtrKey,obj.priorLFootImuToAnkleCtr);
            catch
            end
        end
        
        function combineResults(obj,b)
            % combines results from lbpe b after current lbpe obj
            assert(isa(b,'lowerBodyPoseEstimator'));
            % --- settings --- %
            overwriteStaticParams=1; % option: do you want to overwrite static parameters from b into obj?
            % ---------------- %
            obj.m_time=vertcat(obj.m_time,b.m_time);
            obj.m_sacrumImuPose=[obj.m_sacrumImuPose,b.m_sacrumImuPose];
            obj.m_sacrumImuOrientation=[obj.m_sacrumImuOrientation,b.m_sacrumImuOrientation];
            obj.m_sacrumImuPosition=[obj.m_sacrumImuPosition,b.m_sacrumImuPosition];
            obj.m_sacrumImuVelocity=vertcat(obj.m_sacrumImuVelocity,b.m_sacrumImuVelocity);
            obj.m_sacrumImuGyroBias=vertcat(obj.m_sacrumImuGyroBias,b.m_sacrumImuGyroBias);
            obj.m_sacrumImuAccelBias=vertcat(obj.m_sacrumImuAccelBias,b.m_sacrumImuAccelBias);
            obj.m_rthighImuPose=[obj.m_rthighImuPose,b.m_rthighImuPose];
            obj.m_rthighImuOrientation=[obj.m_rthighImuOrientation,b.m_rthighImuOrientation];
            obj.m_rthighImuPosition=[obj.m_rthighImuPosition,b.m_rthighImuPosition];
            obj.m_rthighImuVelocity=vertcat(obj.m_rthighImuVelocity,b.m_rthighImuVelocity);
            obj.m_rthighImuGyroBias=vertcat(obj.m_rthighImuGyroBias,b.m_rthighImuGyroBias);
            obj.m_rthighImuAccelBias=vertcat(obj.m_rthighImuAccelBias,b.m_rthighImuAccelBias);
            obj.m_rshankImuPose=[obj.m_rshankImuPose,b.m_rshankImuPose];
            obj.m_rshankImuOrientation=[obj.m_rshankImuOrientation,b.m_rshankImuOrientation];
            obj.m_rshankImuPosition=[obj.m_rshankImuPosition,b.m_rshankImuPosition];
            obj.m_rshankImuVelocity=vertcat(obj.m_rshankImuVelocity,b.m_rshankImuVelocity);
            obj.m_rshankImuGyroBias=vertcat(obj.m_rshankImuGyroBias,b.m_rshankImuGyroBias);
            obj.m_rshankImuAccelBias=vertcat(obj.m_rshankImuAccelBias,b.m_rshankImuAccelBias);
            obj.m_rfootImuPose=[obj.m_rfootImuPose,b.m_rfootImuPose];
            obj.m_rfootImuOrientation=[obj.m_rfootImuOrientation,b.m_rfootImuOrientation];
            obj.m_rfootImuPosition=[obj.m_rfootImuPosition,b.m_rfootImuPosition];
            obj.m_rfootImuVelocity=vertcat(obj.m_rfootImuVelocity,b.m_rfootImuVelocity);
            obj.m_rfootImuGyroBias=vertcat(obj.m_rfootImuGyroBias,b.m_rfootImuGyroBias);
            obj.m_rfootImuAccelBias=vertcat(obj.m_rfootImuAccelBias,b.m_rfootImuAccelBias);
            obj.m_lthighImuPose=[obj.m_lthighImuPose,b.m_lthighImuPose];
            obj.m_lthighImuOrientation=[obj.m_lthighImuOrientation,b.m_lthighImuOrientation];
            obj.m_lthighImuPosition=[obj.m_lthighImuPosition,b.m_lthighImuPosition];
            obj.m_lthighImuVelocity=vertcat(obj.m_lthighImuVelocity,b.m_lthighImuVelocity);
            obj.m_lthighImuGyroBias=vertcat(obj.m_lthighImuGyroBias,b.m_lthighImuGyroBias);
            obj.m_lthighImuAccelBias=vertcat(obj.m_lthighImuAccelBias,b.m_lthighImuAccelBias);
            obj.m_lshankImuPose=[obj.m_lshankImuPose,b.m_lshankImuPose];
            obj.m_lshankImuOrientation=[obj.m_lshankImuOrientation,b.m_lshankImuOrientation];
            obj.m_lshankImuPosition=[obj.m_lshankImuPosition,b.m_lshankImuPosition];
            obj.m_lshankImuVelocity=vertcat(obj.m_lshankImuVelocity,b.m_lshankImuVelocity);
            obj.m_lshankImuGyroBias=vertcat(obj.m_lshankImuGyroBias,b.m_lshankImuGyroBias);
            obj.m_lshankImuAccelBias=vertcat(obj.m_lshankImuAccelBias,b.m_lshankImuAccelBias);
            obj.m_lfootImuPose=[obj.m_lfootImuPose,b.m_lfootImuPose];
            obj.m_lfootImuOrientation=[obj.m_lfootImuOrientation,b.m_lfootImuOrientation];
            obj.m_lfootImuPosition=[obj.m_lfootImuPosition,b.m_lfootImuPosition];
            obj.m_lfootImuVelocity=vertcat(obj.m_lfootImuVelocity,b.m_lfootImuVelocity);
            obj.m_lfootImuGyroBias=vertcat(obj.m_lfootImuGyroBias,b.m_lfootImuGyroBias);
            obj.m_lfootImuAccelBias=vertcat(obj.m_lfootImuAccelBias,b.m_lfootImuAccelBias);
            try % angular velocities
                obj.m_sacrumImuAngVel=vertcat(obj.m_sacrumImuAngVel,b.m_sacrumImuAngVel);
                obj.m_rthighImuAngVel=vertcat(obj.m_rthighImuAngVel,b.m_rthighImuAngVel);
                obj.m_rshankImuAngVel=vertcat(obj.m_rshankImuAngVel,b.m_rshankImuAngVel);
                obj.m_rfootImuAngVel=vertcat(obj.m_rfootImuAngVel,b.m_rfootImuAngVel);
                obj.m_lthighImuAngVel=vertcat(obj.m_lthighImuAngVel,b.m_lthighImuAngVel);
                obj.m_lshankImuAngVel=vertcat(obj.m_lshankImuAngVel,b.m_lshankImuAngVel);
                obj.m_lfootImuAngVel=vertcat(obj.m_lfootImuAngVel,b.m_lfootImuAngVel);
            catch
            end
            % () add derived joint angles and segment coordinate systems
            obj.m_rhipFlexExAng=vertcat(obj.m_rhipFlexExAng,b.m_rhipFlexExAng);
            obj.m_rhipIntExtRotAng=vertcat(obj.m_rhipIntExtRotAng,b.m_rhipIntExtRotAng);
            obj.m_rhipAbAdAng=vertcat(obj.m_rhipAbAdAng,b.m_rhipAbAdAng);
            obj.m_rkneeFlexExAng=vertcat(obj.m_rkneeFlexExAng,b.m_rkneeFlexExAng);
            obj.m_rkneeIntExtRotAng=vertcat(obj.m_rkneeIntExtRotAng,b.m_rkneeIntExtRotAng);
            obj.m_rkneeAbAdAng=vertcat(obj.m_rkneeAbAdAng,b.m_rkneeAbAdAng);
            obj.m_lhipFlexExAng=vertcat(obj.m_lhipFlexExAng,b.m_lhipFlexExAng);
            obj.m_lhipIntExtRotAng=vertcat(obj.m_lhipIntExtRotAng,b.m_lhipIntExtRotAng);
            obj.m_lhipAbAdAng=vertcat(obj.m_lhipAbAdAng,b.m_lhipAbAdAng);
            obj.m_lkneeFlexExAng=vertcat(obj.m_lkneeFlexExAng,b.m_lkneeFlexExAng);
            obj.m_lkneeIntExtRotAng=vertcat(obj.m_lkneeIntExtRotAng,b.m_lkneeIntExtRotAng);
            obj.m_lkneeAbAdAng=vertcat(obj.m_lkneeAbAdAng,b.m_lkneeAbAdAng);
            obj.m_R_Pelvis_to_N=cat(3,obj.m_R_Pelvis_to_N,b.m_R_Pelvis_to_N);
            obj.m_R_RFemur_to_N=cat(3,obj.m_R_RFemur_to_N,b.m_R_RFemur_to_N);
            obj.m_R_RTibia_to_N=cat(3,obj.m_R_RTibia_to_N,b.m_R_RTibia_to_N);
            obj.m_R_LFemur_to_N=cat(3,obj.m_R_LFemur_to_N,b.m_R_LFemur_to_N);
            obj.m_R_LTibia_to_N=cat(3,obj.m_R_LTibia_to_N,b.m_R_LTibia_to_N);
            % () optionally, overwrite static params
            if overwriteStaticParams
                obj.m_rkneeAxisThigh=b.m_rkneeAxisThigh;
                obj.m_rkneeAxisShank=b.m_rkneeAxisShank;
                obj.m_lkneeAxisThigh=b.m_lkneeAxisThigh;
                obj.m_lkneeAxisShank=b.m_lkneeAxisShank;
                obj.m_sacrumImuToRHipCtr=b.m_sacrumImuToRHipCtr;
                obj.m_rthighImuToHipCtr=b.m_rthighImuToHipCtr;
                obj.m_rthighImuToKneeCtr=b.m_rthighImuToKneeCtr;
                obj.m_rshankImuToKneeCtr=b.m_rshankImuToKneeCtr;
                obj.m_rshankImuToAnkleCtr=b.m_rshankImuToAnkleCtr;
                obj.m_rfootImuToAnkleCtr=b.m_rfootImuToAnkleCtr;
                obj.m_sacrumImuToLHipCtr=b.m_sacrumImuToLHipCtr;
                obj.m_lthighImuToHipCtr=b.m_lthighImuToHipCtr;
                obj.m_lthighImuToKneeCtr=b.m_lthighImuToKneeCtr;
                obj.m_lshankImuToKneeCtr=b.m_lshankImuToKneeCtr;
                obj.m_lshankImuToAnkleCtr=b.m_lshankImuToAnkleCtr;
                obj.m_lfootImuToAnkleCtr=b.m_lfootImuToAnkleCtr;
                try obj.m_hipAxisSacrum=b.m_hipAxisSacrum; catch; end
                try obj.m_rhipAxisThigh=b.m_rhipAxisThigh; catch; end
                try obj.m_lhipAxisThigh=b.m_lhipAxisThigh; catch; end
                try obj.m_rankleAxisFoot=b.m_rankleAxisFoot; catch; end
                try obj.m_lankleAxisFoot=b.m_lankleAxisFoot; catch; end
            end
        end
        
        function constructFromResultsFile(obj,file)
            % this function takes in an .h5 file and populated the obj's results from it.
            % useful for using all of the plotting, debugging methods, etc. associated with this class.
            % populate IMU data
            % remember that the IMU data is stored as the following types:
            %   orientations: array of gtsam.Rot3, positions: array gtsam.Point3, velocities, accelbias, gyrobias: Nx3 matrix
            %   in the MATLAB version we aren't storing the Pose3's. maybe can add that in the future.
            [~,filename,ext]=fileparts(file);
            fprintf('\tconstructing lowerBodyPoseEstimator from results file %s ...',strcat(filename,ext));
            startTic=tic;
            % pull out numeric data for imu states
            [obj.m_sacrumImuOrientation,obj.m_sacrumImuPosition,obj.m_sacrumImuVelocity,obj.m_sacrumImuGyroBias,obj.m_sacrumImuAccelBias]=lowerBodyPoseEstimator.readImuStatesFromH5ResultsFileByString(file,'Sacrum');
            [obj.m_rthighImuOrientation,obj.m_rthighImuPosition,obj.m_rthighImuVelocity,obj.m_rthighImuGyroBias,obj.m_rthighImuAccelBias]=lowerBodyPoseEstimator.readImuStatesFromH5ResultsFileByString(file,'RThigh');
            [obj.m_rshankImuOrientation,obj.m_rshankImuPosition,obj.m_rshankImuVelocity,obj.m_rshankImuGyroBias,obj.m_rshankImuAccelBias]=lowerBodyPoseEstimator.readImuStatesFromH5ResultsFileByString(file,'RShank');
            [obj.m_rfootImuOrientation,obj.m_rfootImuPosition,obj.m_rfootImuVelocity,obj.m_rfootImuGyroBias,obj.m_rfootImuAccelBias]=lowerBodyPoseEstimator.readImuStatesFromH5ResultsFileByString(file,'RFoot');
            [obj.m_lthighImuOrientation,obj.m_lthighImuPosition,obj.m_lthighImuVelocity,obj.m_lthighImuGyroBias,obj.m_lthighImuAccelBias]=lowerBodyPoseEstimator.readImuStatesFromH5ResultsFileByString(file,'LThigh');
            [obj.m_lshankImuOrientation,obj.m_lshankImuPosition,obj.m_lshankImuVelocity,obj.m_lshankImuGyroBias,obj.m_lshankImuAccelBias]=lowerBodyPoseEstimator.readImuStatesFromH5ResultsFileByString(file,'LShank');
            [obj.m_lfootImuOrientation,obj.m_lfootImuPosition,obj.m_lfootImuVelocity,obj.m_lfootImuGyroBias,obj.m_lfootImuAccelBias]=lowerBodyPoseEstimator.readImuStatesFromH5ResultsFileByString(file,'LFoot');
            % knee hinge axes (gtsam.Unit3)
            obj.m_lkneeAxisThigh=gtsam.Unit3(gtsam.Point3(h5read(file,'/Estimated/LKneeAxis_LThigh')));
            obj.m_lkneeAxisShank=gtsam.Unit3(gtsam.Point3(h5read(file,'/Estimated/LKneeAxis_LShank')));
            obj.m_rkneeAxisThigh=gtsam.Unit3(gtsam.Point3(h5read(file,'/Estimated/RKneeAxis_RThigh')));
            obj.m_rkneeAxisShank=gtsam.Unit3(gtsam.Point3(h5read(file,'/Estimated/RKneeAxis_RShank')));
            % if exist, pull out thigh axes
            try
                obj.m_rhipAxisThigh=gtsam.Unit3(gtsam.Point3(h5read(file,'/Estimated/RHipAxis_RThigh')));
                obj.m_lhipAxisThigh=gtsam.Unit3(gtsam.Point3(h5read(file,'/Estimated/LHipAxis_LThigh')));
            catch
            end
            % static vectors to joint centers
            obj.m_sacrumImuToRHipCtr=gtsam.Point3(h5read(file,'/Estimated/SacrumImuToRHipCtr'));
            obj.m_rthighImuToHipCtr=gtsam.Point3(h5read(file,'/Estimated/RThighImuToRHipCtr'));
            obj.m_rthighImuToKneeCtr=gtsam.Point3(h5read(file,'/Estimated/RThighImuToRKneeCtr'));
            obj.m_rshankImuToKneeCtr=gtsam.Point3(h5read(file,'/Estimated/RShankImuToRKneeCtr'));
            obj.m_rshankImuToAnkleCtr=gtsam.Point3(h5read(file,'/Estimated/RShankImuToRAnkleCtr'));
            obj.m_rfootImuToAnkleCtr=gtsam.Point3(h5read(file,'/Estimated/RFootImuToRAnkleCtr'));
            obj.m_sacrumImuToLHipCtr=gtsam.Point3(h5read(file,'/Estimated/SacrumImuToLHipCtr'));
            obj.m_lthighImuToHipCtr=gtsam.Point3(h5read(file,'/Estimated/LThighImuToLHipCtr'));
            obj.m_lthighImuToKneeCtr=gtsam.Point3(h5read(file,'/Estimated/LThighImuToLKneeCtr'));
            obj.m_lshankImuToKneeCtr=gtsam.Point3(h5read(file,'/Estimated/LShankImuToLKneeCtr'));
            obj.m_lshankImuToAnkleCtr=gtsam.Point3(h5read(file,'/Estimated/LShankImuToLAnkleCtr'));
            obj.m_lfootImuToAnkleCtr=gtsam.Point3(h5read(file,'/Estimated/LFootImuToLAnkleCtr'));
            % if exist, angular velocities
            try
                obj.m_sacrumImuAngVel=h5read(file,'/Estimated/SacrumImuAngVel')';
                obj.m_rthighImuAngVel=h5read(file,'/Estimated/RThighImuAngVel')';
                obj.m_rshankImuAngVel=h5read(file,'/Estimated/RShankImuAngVel')';
                obj.m_rfootImuAngVel=h5read(file,'/Estimated/RFootImuAngVel')';
                obj.m_lthighImuAngVel=h5read(file,'/Estimated/LThighImuAngVel')';
                obj.m_lshankImuAngVel=h5read(file,'/Estimated/LShankImuAngVel')';
                obj.m_lfootImuAngVel=h5read(file,'/Estimated/LFootImuAngVel')';
                % make sure they're a Nx3 matrix, not the other way around
                if size(obj.m_sacrumImuAngVel,1)==3; obj.m_sacrumImuAngVel=obj.m_sacrumImuAngVel'; end
                if size(obj.m_rthighImuAngVel,1)==3; obj.m_rthighImuAngVel=obj.m_rthighImuAngVel'; end
                if size(obj.m_rshankImuAngVel,1)==3; obj.m_rshankImuAngVel=obj.m_rshankImuAngVel'; end
                if size(obj.m_rfootImuAngVel,1)==3; obj.m_rfootImuAngVel=obj.m_rfootImuAngVel'; end
                if size(obj.m_lthighImuAngVel,1)==3; obj.m_lthighImuAngVel=obj.m_lthighImuAngVel'; end
                if size(obj.m_lshankImuAngVel,1)==3; obj.m_lshankImuAngVel=obj.m_lshankImuAngVel'; end
                if size(obj.m_lfootImuAngVel,1)==3; obj.m_lfootImuAngVel=obj.m_lfootImuAngVel'; end
            catch
            end
            % if exit, pull our foot and sacrum axes
            try
                obj.m_hipAxisSacrum=gtsam.Unit3(gtsam.Point3(h5read(file,'/Estimated/HipAxis_Sacrum')));
            catch
            end
            try
                obj.m_rankleAxisFoot=gtsam.Unit3(gtsam.Point3(h5read(file,'/Estimated/RAnkleAxis_Foot')));
                obj.m_lankleAxisFoot=gtsam.Unit3(gtsam.Point3(h5read(file,'/Estimated/LAnkleAxis_Foot')));
            catch
            end
            % try to pull out time. if not, it's because the cpp version didn't have it. make assumption. DEPRECATE THIS ONCE YOU RERUN THE CPP RESULTS WITH TIME SET IN THE H5 FILE
            try
                obj.m_time=h5read(file,'/Time');
            catch
                % it probably didn't exist. make assumption.
                obj.m_time=0:.1:((length(obj.m_rthighImuOrientation)-1)*.1);
            end
            % try to pull out id too
            try
                obj.id=h5read('/General/ID');
            catch
                % it probably didn't exist. make assumption.
                obj.id='NO_ID';
            end
            try % if exist, try internal precession angles
                obj.m_sacrumImuInternalPrecessionAng=h5read(file,'/Derived/sacrumImuInternalPrecessionAngle');
                obj.m_rthighImuInternalPrecessionAng=h5read(file,'/Derived/rthighImuInternalPrecessionAngle');
                obj.m_rshankImuInternalPrecessionAng=h5read(file,'/Derived/rshankImuInternalPrecessionAngle');
                obj.m_rfootImuInternalPrecessionAng=h5read(file,'/Derived/rfootImuInternalPrecessionAngle');
                obj.m_lthighImuInternalPrecessionAng=h5read(file,'/Derived/lthighImuInternalPrecessionAngle');
                obj.m_lshankImuInternalPrecessionAng=h5read(file,'/Derived/lshankImuInternalPrecessionAngle');
                obj.m_lfootImuInternalPrecessionAng=h5read(file,'/Derived/lfootImuInternalPrecessionAngle');
            catch mException
                fprintf('error thrown: %s... computing internal precession angles instead\n',mException.message);
                %                 setAllInternalPrecessionAngles(obj);
            end
            setDerivedJointAnglesFromEstimatedStates(obj); % set derived joint angle and skeletal parameters parameters
            fprintf(' done! (%.3f seconds)\n',toc(startTic));
        end
        
        function printHipAxisFromOtherMethods(obj)
            % after estimation runs, check the hip axis that was estimated against:
            % (1) the hip axis from PCA of all the raw IMU data (McGrath 2018)
            % (2) using McGrath 2018 on the estimated angular velocities and orientations
            assert(~isempty(obj.m_hipAxisSacrum),'hip axis is not estimated!');
            fprintf('-------- debugging info: hip axes comparison to other methods -------- \n');
            % method (1) above
            fprintf('From raw gyro data and APDM-estimated quaternions:\n');
            try
                sacrumImuGyros=[obj.m_sacrumImuPoseProblem.m_imu.gx obj.m_sacrumImuPoseProblem.m_imu.gy obj.m_sacrumImuPoseProblem.m_imu.gz];
                rthighImuGyros=[obj.m_rthighImuPoseProblem.m_imu.gx obj.m_rthighImuPoseProblem.m_imu.gy obj.m_rthighImuPoseProblem.m_imu.gz];
                lthighImuGyros=[obj.m_lthighImuPoseProblem.m_imu.gx obj.m_lthighImuPoseProblem.m_imu.gy obj.m_lthighImuPoseProblem.m_imu.gz];
                [hipAxisRSacrum,hipAxisRThigh]=hingeAxesAccordingToMcGrath2018(obj.m_sacrumImuPoseProblem.m_imu.qAPDM,obj.m_rthighImuPoseProblem.m_imu.qAPDM,sacrumImuGyros,rthighImuGyros);
                [hipAxisLSacrum,hipAxisLThigh]=hingeAxesAccordingToMcGrath2018(obj.m_sacrumImuPoseProblem.m_imu.qAPDM,obj.m_lthighImuPoseProblem.m_imu.qAPDM,sacrumImuGyros,lthighImuGyros);
                printVectorOptionalVariance('    hipAxisRSacrum',hipAxisRSacrum,[]);
                printVectorOptionalVariance('    hipAxisRThigh',hipAxisRThigh,[]);
                printVectorOptionalVariance('    hipAxisLSacrum',hipAxisLSacrum,[]);
                printVectorOptionalVariance('    hipAxisLThigh',hipAxisLThigh,[]);
            catch mException
                fprintf('could not get hip axes according to raw APDM and gyro data (McGrath et al. 2018), error was: %s\n',mException.message);
            end
            % method (2) above
            fprintf('From post-hoc estimated angular velocities and IMU orientations:\n');
            try
                [hipAxisRSacrum2,hipAxisRThigh2]=hingeAxesAccordingToMcGrath2018(obj.m_sacrumImuOrientation,obj.m_rthighImuOrientation,obj.m_sacrumImuAngVel,obj.m_rthighImuAngVel);
                [hipAxisLSacrum2,hipAxisLThigh2]=hingeAxesAccordingToMcGrath2018(obj.m_sacrumImuOrientation,obj.m_lthighImuOrientation,obj.m_sacrumImuAngVel,obj.m_lthighImuAngVel);
                printVectorOptionalVariance('    hipAxisRSacrum',hipAxisRSacrum2,[]);
                printVectorOptionalVariance('    hipAxisRThigh',hipAxisRThigh2,[]);
                printVectorOptionalVariance('    hipAxisLSacrum',hipAxisLSacrum2,[]);
                printVectorOptionalVariance('    hipAxisLThigh',hipAxisLThigh2,[]);
                fprintf('    (ideally, hipAxisRSacrum = hipAxisLSacrum since they share the same sacrum frame orientation and angular velocity)\n');
            catch mException
                fprintf('could not get hip axes according to post-hoc estimated orientations and angular velocities (McGrath et al. 2018), error was: %s\n',mException.message);
            end
            % our estimated hip hinge axes
            fprintf('Estimated hip hinge axes:\n');
            printVectorOptionalVariance('    hip axis, sacrum frame',obj.m_hipAxisSacrum.point3().vector(),[]);
            fprintf('-----------------------------------------------------------------------\n');
        end
        
        function printKneeAxisFromOtherMethods(obj)
            % after estimation runs, check the knee axis that was estimated against:
            % (1) the knee axis from PCA of all the raw IMU data (McGrath 2018)
            % (2) using McGrath 2018 on the estimated angular velocities and orientations
            fprintf('-------- debugging info: knee axes comparison to other methods -------- \n');
            % method (1) above
            fprintf('From raw gyro data and APDM-estimated quaternions:\n');
            try
                rthighImuGyros=[obj.m_rthighImuPoseProblem.m_imu.gx obj.m_rthighImuPoseProblem.m_imu.gy obj.m_rthighImuPoseProblem.m_imu.gz];
                rshankImuGyros=[obj.m_rshankImuPoseProblem.m_imu.gx obj.m_rshankImuPoseProblem.m_imu.gy obj.m_rshankImuPoseProblem.m_imu.gz];
                lthighImuGyros=[obj.m_lthighImuPoseProblem.m_imu.gx obj.m_lthighImuPoseProblem.m_imu.gy obj.m_lthighImuPoseProblem.m_imu.gz];
                lshankImuGyros=[obj.m_lshankImuPoseProblem.m_imu.gx obj.m_lshankImuPoseProblem.m_imu.gy obj.m_lshankImuPoseProblem.m_imu.gz];
                [kneeAxisRThigh,kneeAxisRShank]=hingeAxesAccordingToMcGrath2018(obj.m_rthighImuPoseProblem.m_imu.qAPDM,obj.m_rshankImuPoseProblem.m_imu.qAPDM,rthighImuGyros,rshankImuGyros);
                [kneeAxisLThigh,kneeAxisLShank]=hingeAxesAccordingToMcGrath2018(obj.m_lthighImuPoseProblem.m_imu.qAPDM,obj.m_lshankImuPoseProblem.m_imu.qAPDM,lthighImuGyros,lshankImuGyros);
                printVectorOptionalVariance('    kneeAxisRThigh',kneeAxisRThigh,[]);
                printVectorOptionalVariance('    kneeAxisRShank',kneeAxisRShank,[]);
                printVectorOptionalVariance('    kneeAxisLThigh',kneeAxisLThigh,[]);
                printVectorOptionalVariance('    kneeAxisLShank',kneeAxisLShank,[]);
            catch mException
                fprintf('could not get knee axes according to raw APDM and gyro data (McGrath et al. 2018), error was: %s\n',mException.message);
            end
            % method (2) above
            fprintf('From post-hoc estimated angular velocities and IMU orientations:\n');
            try
                [kneeAxisRThigh2,kneeAxisRShank2]=hingeAxesAccordingToMcGrath2018(obj.m_rthighImuOrientation,obj.m_rshankImuOrientation,obj.m_rthighImuAngVel,obj.m_rshankImuAngVel);
                [kneeAxisLThigh2,kneeAxisLShank2]=hingeAxesAccordingToMcGrath2018(obj.m_lthighImuOrientation,obj.m_lshankImuOrientation,obj.m_lthighImuAngVel,obj.m_lshankImuAngVel);
                printVectorOptionalVariance('    kneeAxisRThigh',kneeAxisRThigh2,[]);
                printVectorOptionalVariance('    kneeAxisRShank',kneeAxisRShank2,[]);
                printVectorOptionalVariance('    kneeAxisLThigh',kneeAxisLThigh2,[]);
                printVectorOptionalVariance('    kneeAxisLShank',kneeAxisLShank2,[]);
            catch mException
                fprintf('could not get knee axes according to post-hoc estimated orientations and angular velocities (McGrath et al. 2018), error was: %s\n',mException.message);
            end
            % our estimated knee hinge axes
            fprintf('Estimated knee hinge axes:\n');
            printVectorOptionalVariance('    rknee axis, thigh frame',obj.m_rkneeAxisThigh.point3().vector(),diag(obj.m_rkneeAxisThighCovariance));
            printVectorOptionalVariance('    rknee axis, shank frame',obj.m_rkneeAxisShank.point3().vector(),diag(obj.m_rkneeAxisShankCovariance));
            printVectorOptionalVariance('    lknee axis, thigh frame',obj.m_lkneeAxisThigh.point3().vector(),diag(obj.m_lkneeAxisThighCovariance));
            printVectorOptionalVariance('    lknee axis, shank frame',obj.m_lkneeAxisShank.point3().vector(),diag(obj.m_lkneeAxisShankCovariance));
            fprintf('-----------------------------------------------------------------------\n');
        end
        
        function setAllInternalPrecessionAngles(obj)
            % calculates and sets all internal precession angles
            obj.m_sacrumImuInternalPrecessionAng=imuInternalPrecessionAboutStaticVector(obj.m_sacrumImuOrientation,averageVerticalVectorInSacrumImuFrame(obj),(obj.m_sacrumImuToRHipCtr.vector()-obj.m_sacrumImuToLHipCtr.vector()) );
            obj.m_rthighImuInternalPrecessionAng=imuInternalPrecessionAboutStaticVector(obj.m_rthighImuOrientation,obj.m_rthighImuToHipCtr.vector(),obj.m_rkneeAxisThigh.point3().vector());
            obj.m_rshankImuInternalPrecessionAng=imuInternalPrecessionAboutStaticVector(obj.m_rshankImuOrientation,obj.m_rshankImuToKneeCtr.vector(),obj.m_rkneeAxisShank.point3().vector());
            obj.m_rfootImuInternalPrecessionAng=imuInternalPrecessionAboutStaticVector(obj.m_rfootImuOrientation,obj.m_rfootImuToAnkleCtr.vector(),obj.m_rankleAxisFoot.point3().vector());
            obj.m_lthighImuInternalPrecessionAng=imuInternalPrecessionAboutStaticVector(obj.m_lthighImuOrientation,obj.m_lthighImuToHipCtr.vector(),obj.m_lkneeAxisThigh.point3().vector());
            obj.m_lshankImuInternalPrecessionAng=imuInternalPrecessionAboutStaticVector(obj.m_lshankImuOrientation,obj.m_lshankImuToKneeCtr.vector(),obj.m_lkneeAxisShank.point3().vector());
            obj.m_lfootImuInternalPrecessionAng=imuInternalPrecessionAboutStaticVector(obj.m_lfootImuOrientation,obj.m_lfootImuToAnkleCtr.vector(),obj.m_lankleAxisFoot.point3().vector());
        end
        
        function setNoiseModelsFromMemberNoiseVariables(obj)
            % sets the propoerties for noise models from the propoerties for noise vectors
            obj.m_kneeJointCtrConnectionNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_kneeJointCtrConnectionNoise(:));
            obj.m_hipJointCtrConnectionNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_hipJointCtrConnectionNoise(:));
            obj.m_ankleJointCtrConnectionNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_ankleJointCtrConnectionNoise(:));
            obj.m_kneeHingeAxisNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_kneeHingeAxisNoise(:));
            obj.m_hipHingeAxisNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_hipHingeAxisNoise(:));
            obj.m_ankleHingeAxisNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_ankleHingeAxisNoise(:));
            obj.m_rfemurLengthNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_rfemurLengthSigma);
            obj.m_lfemurLengthNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_lfemurLengthSigma);
            obj.m_rtibiaLengthNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_rtibiaLengthSigma);
            obj.m_ltibiaLengthNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_ltibiaLengthSigma);
            obj.m_pelvicWidthNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_pelvicWidthSigma);
            obj.m_segmentLengthMaxNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_segmentLengthMaxNoise);
            obj.m_segmentLengthMinNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_segmentLengthMinNoise);
            obj.m_angVelNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_angVelNoise(:));
            obj.m_jointCtrVelConnectionNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_jointCtrVelConnectionNoise(:));
            obj.m_femurLengthDiscrepancyNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_femurLengthDiscrepancyNoise);
            obj.m_tibiaLengthDiscrepancyNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_tibiaLengthDiscrepancyNoise);
            obj.m_angBwKneeAxisAndSegmentFemurNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_angBwKneeAxisAndSegmentStdFemur);
            obj.m_angBwKneeAxisAndSegmentTibiaNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_angBwKneeAxisAndSegmentStdTibia);
        end
        
        function setAnthroNoiseProperties(obj)
            % from perscribed gender, compute and set:
            % mean, std for each anthro measurement + set their properties
            obj.m_rfemurLength=getAnthroMeasure('femurlength',obj.m_gender,'mean');
            obj.m_rtibiaLength=getAnthroMeasure('tibialength',obj.m_gender,'mean');
            obj.m_rfemurLengthSigma=getAnthroMeasure('femurlength',obj.m_gender,'std');
            obj.m_rtibiaLengthSigma=getAnthroMeasure('tibialength',obj.m_gender,'std');
            obj.m_lfemurLength=getAnthroMeasure('femurlength',obj.m_gender,'mean');
            obj.m_ltibiaLength=getAnthroMeasure('tibialength',obj.m_gender,'mean');
            obj.m_lfemurLengthSigma=getAnthroMeasure('femurlength',obj.m_gender,'std');
            obj.m_ltibiaLengthSigma=getAnthroMeasure('tibialength',obj.m_gender,'std');
            obj.m_pelvicWidth=getAnthroMeasure('femoralheadseparation',obj.m_gender,'mean');
            obj.m_pelvicWidthSigma=getAnthroMeasure('femoralheadseparation',obj.m_gender,'std');
        end
        
        function setStaticVariablesNoiseHigh(obj,varargin)
            % debugging method: take the noise of the static variables constraints and set them high.
            % intuitively this should make the converged IMU solutions similar to running a single smoothing algo on each IMU
            % this should be run BEFORE setup()
            % () take each noise variable and multiply it by some factor
            fac=1.0e3;
            if nargin==2
                if isa(varargin{1},'double') % we assume this is the factor you want to multiply by
                    fac=varargin{1};
                end
            end
            obj.m_kneeJointCtrConnectionNoise=obj.m_kneeJointCtrConnectionNoise*fac; % std, meters
            obj.m_hipJointCtrConnectionNoise=obj.m_hipJointCtrConnectionNoise*fac; % std, meters
            obj.m_ankleJointCtrConnectionNoise=obj.m_ankleJointCtrConnectionNoise*fac; % std, meters
            obj.m_kneeJointCtrConnectionNoise=obj.m_kneeJointCtrConnectionNoise*fac; % std, meters
            obj.m_hipJointCtrConnectionNoise=obj.m_hipJointCtrConnectionNoise*fac; % std, meters
            obj.m_ankleJointCtrConnectionNoise=obj.m_ankleJointCtrConnectionNoise*fac; % std, meters
            obj.m_kneeHingeAxisNoise=obj.m_kneeHingeAxisNoise*fac; % std, rad/s
            obj.m_kneeHingeAxisNoise=obj.m_kneeHingeAxisNoise*fac; % std, rad/s
            obj.m_sacrumSternumRigidConnectionNoise=obj.m_sacrumSternumRigidConnectionNoise*fac; % std, meters
            obj.m_rfemurLengthSigma=obj.m_rfemurLengthSigma*fac;
            obj.m_lfemurLengthSigma=obj.m_lfemurLengthSigma*fac;
            obj.m_rtibiaLengthSigma=obj.m_rtibiaLengthSigma*fac;
            obj.m_ltibiaLengthSigma=obj.m_ltibiaLengthSigma*fac;
            obj.m_pelvicWidthSigma=obj.m_pelvicWidthSigma*fac;
            % refresh noise models
            setNoiseModelsFromMemberNoiseVariables(obj);
        end
        
        function resizeIpeBiasKeysIfNecessary(obj)
            % resizes each array of imu bias keys to make them full size if they were static
            if(obj.m_sacrumImuPoseProblem.m_imuBiasModelMode==0)
                obj.m_sacrumImuPoseProblem.m_imuBiasKeys=repmat(obj.m_sacrumImuPoseProblem.m_imuBiasKeys,[obj.getNumKeyframes 1]);
            end
            if(obj.m_rthighImuPoseProblem.m_imuBiasModelMode==0)
                obj.m_rthighImuPoseProblem.m_imuBiasKeys=repmat(obj.m_rthighImuPoseProblem.m_imuBiasKeys,[obj.getNumKeyframes 1]);
            end
            if(obj.m_rshankImuPoseProblem.m_imuBiasModelMode==0)
                obj.m_rshankImuPoseProblem.m_imuBiasKeys=repmat(obj.m_rshankImuPoseProblem.m_imuBiasKeys,[obj.getNumKeyframes 1]);
            end
            if(obj.m_rfootImuPoseProblem.m_imuBiasModelMode==0)
                obj.m_rfootImuPoseProblem.m_imuBiasKeys=repmat(obj.m_rfootImuPoseProblem.m_imuBiasKeys,[obj.getNumKeyframes 1]);
            end
            if(obj.m_lthighImuPoseProblem.m_imuBiasModelMode==0)
                obj.m_lthighImuPoseProblem.m_imuBiasKeys=repmat(obj.m_lthighImuPoseProblem.m_imuBiasKeys,[obj.getNumKeyframes 1]);
            end
            if(obj.m_lshankImuPoseProblem.m_imuBiasModelMode==0)
                obj.m_lshankImuPoseProblem.m_imuBiasKeys=repmat(obj.m_lshankImuPoseProblem.m_imuBiasKeys,[obj.getNumKeyframes 1]);
            end
            if(obj.m_lfootImuPoseProblem.m_imuBiasModelMode==0)
                obj.m_lfootImuPoseProblem.m_imuBiasKeys=repmat(obj.m_lfootImuPoseProblem.m_imuBiasKeys,[obj.getNumKeyframes 1]);
            end
        end
        
        function addInstantaneousAngVelEstimationToImuPoses(obj)
            % you have 7 IMUs. Each has a trajectory of poses, related through preintegration of the gyroscope measurements.
            % this method adds both the new variables and the constraints to estimate angular velocity at keyframe k
            % this is closely related to the measured angular velocity, but with bias removed and a noise distribution
            % this is only in the bioestimator functions -- not imuPoseEstimator, because it's useless for a single IMU.
            % () get var chars and strs for the new estimated angular velocity variable
            obj.m_sacrumImuAngVelVarStr=strcat(obj.id,'_sacrumImuAngVel');
            VarStrToCharMap.insert(obj.m_sacrumImuAngVelVarStr);
            obj.m_sacrumImuAngVelVarChar=VarStrToCharMap.getChar(obj.m_sacrumImuAngVelVarStr);
            obj.m_rthighImuAngVelVarStr=strcat(obj.id,'_rthighImuAngVel');
            VarStrToCharMap.insert(obj.m_rthighImuAngVelVarStr);
            obj.m_rthighImuAngVelVarChar=VarStrToCharMap.getChar(obj.m_rthighImuAngVelVarStr);
            obj.m_rshankImuAngVelVarStr=strcat(obj.id,'_rshankImuAngVel');
            VarStrToCharMap.insert(obj.m_rshankImuAngVelVarStr);
            obj.m_rshankImuAngVelVarChar=VarStrToCharMap.getChar(obj.m_rshankImuAngVelVarStr);
            obj.m_rfootImuAngVelVarStr=strcat(obj.id,'_rfootImuAngVel');
            VarStrToCharMap.insert(obj.m_rfootImuAngVelVarStr);
            obj.m_rfootImuAngVelVarChar=VarStrToCharMap.getChar(obj.m_rfootImuAngVelVarStr);
            obj.m_lthighImuAngVelVarStr=strcat(obj.id,'_lthighImuAngVel');
            VarStrToCharMap.insert(obj.m_lthighImuAngVelVarStr);
            obj.m_lthighImuAngVelVarChar=VarStrToCharMap.getChar(obj.m_lthighImuAngVelVarStr);
            obj.m_lshankImuAngVelVarStr=strcat(obj.id,'_lshankImuAngVel');
            VarStrToCharMap.insert(obj.m_lshankImuAngVelVarStr);
            obj.m_lshankImuAngVelVarChar=VarStrToCharMap.getChar(obj.m_lshankImuAngVelVarStr);
            obj.m_lfootImuAngVelVarStr=strcat(obj.id,'_lfootImuAngVel');
            VarStrToCharMap.insert(obj.m_lfootImuAngVelVarStr);
            obj.m_lfootImuAngVelVarChar=VarStrToCharMap.getChar(obj.m_lfootImuAngVelVarStr);
            % () now construct the arrays to hold the keys
            obj.m_sacrumImuAngVelKeys=zeros(length(obj.m_sacrumImuPoseProblem.m_poseKeys),1,'uint64');
            obj.m_rthighImuAngVelKeys=zeros(length(obj.m_rthighImuPoseProblem.m_poseKeys),1,'uint64');
            obj.m_rshankImuAngVelKeys=zeros(length(obj.m_rshankImuPoseProblem.m_poseKeys),1,'uint64');
            obj.m_rfootImuAngVelKeys=zeros(length(obj.m_rfootImuPoseProblem.m_poseKeys),1,'uint64');
            obj.m_lthighImuAngVelKeys=zeros(length(obj.m_lthighImuPoseProblem.m_poseKeys),1,'uint64');
            obj.m_lshankImuAngVelKeys=zeros(length(obj.m_lshankImuPoseProblem.m_poseKeys),1,'uint64');
            obj.m_lfootImuAngVelKeys=zeros(length(obj.m_lfootImuPoseProblem.m_poseKeys),1,'uint64');
            for k=1:(length(obj.m_sacrumImuAngVelKeys))
                obj.m_sacrumImuAngVelKeys(k)=gtsam.symbol(obj.m_sacrumImuAngVelVarChar,k-1);
                obj.m_rthighImuAngVelKeys(k)=gtsam.symbol(obj.m_rthighImuAngVelVarChar,k-1);
                obj.m_rshankImuAngVelKeys(k)=gtsam.symbol(obj.m_rshankImuAngVelVarChar,k-1);
                obj.m_rfootImuAngVelKeys(k)=gtsam.symbol(obj.m_rfootImuAngVelVarChar,k-1);
                obj.m_lthighImuAngVelKeys(k)=gtsam.symbol(obj.m_lthighImuAngVelVarChar,k-1);
                obj.m_lshankImuAngVelKeys(k)=gtsam.symbol(obj.m_lshankImuAngVelVarChar,k-1);
                obj.m_lfootImuAngVelKeys(k)=gtsam.symbol(obj.m_lfootImuAngVelVarChar,k-1);
            end
            % () check: if imu problems are static bias, expand the bias key array to make it the same size as everything else
            resizeIpeBiasKeysIfNecessary(obj);
            % () now loop and setup factors
            imuIdxArray=getImuIndecesCorrespondingToKeyframes(obj.m_sacrumImuPoseProblem); % assume these are all the same
            sacrumImuGyros=[obj.m_sacrumImuPoseProblem.m_imu.gx obj.m_sacrumImuPoseProblem.m_imu.gy obj.m_sacrumImuPoseProblem.m_imu.gz];
            rthighImuGyros=[obj.m_rthighImuPoseProblem.m_imu.gx obj.m_rthighImuPoseProblem.m_imu.gy obj.m_rthighImuPoseProblem.m_imu.gz];
            rshankImuGyros=[obj.m_rshankImuPoseProblem.m_imu.gx obj.m_rshankImuPoseProblem.m_imu.gy obj.m_rshankImuPoseProblem.m_imu.gz];
            rfootImuGyros=[obj.m_rfootImuPoseProblem.m_imu.gx obj.m_rfootImuPoseProblem.m_imu.gy obj.m_rfootImuPoseProblem.m_imu.gz];
            lthighImuGyros=[obj.m_lthighImuPoseProblem.m_imu.gx obj.m_lthighImuPoseProblem.m_imu.gy obj.m_lthighImuPoseProblem.m_imu.gz];
            lshankImuGyros=[obj.m_lshankImuPoseProblem.m_imu.gx obj.m_lshankImuPoseProblem.m_imu.gy obj.m_lshankImuPoseProblem.m_imu.gz];
            lfootImuGyros=[obj.m_lfootImuPoseProblem.m_imu.gx obj.m_lfootImuPoseProblem.m_imu.gy obj.m_lfootImuPoseProblem.m_imu.gz];
            for k=1:length(obj.m_sacrumImuPoseProblem.m_poseKeys)
                % add factors to graph
                obj.m_graph.push_back(bioslam.AngularVelocityFactor(obj.m_sacrumImuAngVelKeys(k),obj.m_sacrumImuPoseProblem.m_imuBiasKeys(k),sacrumImuGyros(imuIdxArray(k),:)',obj.m_angVelNoiseModel));
                obj.m_graph.push_back(bioslam.AngularVelocityFactor(obj.m_rthighImuAngVelKeys(k),obj.m_rthighImuPoseProblem.m_imuBiasKeys(k),rthighImuGyros(imuIdxArray(k),:)',obj.m_angVelNoiseModel));
                obj.m_graph.push_back(bioslam.AngularVelocityFactor(obj.m_rshankImuAngVelKeys(k),obj.m_rshankImuPoseProblem.m_imuBiasKeys(k),rshankImuGyros(imuIdxArray(k),:)',obj.m_angVelNoiseModel));
                obj.m_graph.push_back(bioslam.AngularVelocityFactor(obj.m_rfootImuAngVelKeys(k),obj.m_rfootImuPoseProblem.m_imuBiasKeys(k),rfootImuGyros(imuIdxArray(k),:)',obj.m_angVelNoiseModel));
                obj.m_graph.push_back(bioslam.AngularVelocityFactor(obj.m_lthighImuAngVelKeys(k),obj.m_lthighImuPoseProblem.m_imuBiasKeys(k),lthighImuGyros(imuIdxArray(k),:)',obj.m_angVelNoiseModel));
                obj.m_graph.push_back(bioslam.AngularVelocityFactor(obj.m_lshankImuAngVelKeys(k),obj.m_lshankImuPoseProblem.m_imuBiasKeys(k),lshankImuGyros(imuIdxArray(k),:)',obj.m_angVelNoiseModel));
                obj.m_graph.push_back(bioslam.AngularVelocityFactor(obj.m_lfootImuAngVelKeys(k),obj.m_lfootImuPoseProblem.m_imuBiasKeys(k),lfootImuGyros(imuIdxArray(k),:)',obj.m_angVelNoiseModel));
                % add initial values to graph as whatever the measured gyro rate is at that index
                %     i.e., initial error due to these factors should be zero
                %     you could check this by using printErrorsInGraphByFactorType(obj.m_graph,obj.m_initialValues);
                obj.m_initialValues.insert(obj.m_sacrumImuAngVelKeys(k),sacrumImuGyros(imuIdxArray(k),:)');
                obj.m_initialValues.insert(obj.m_rthighImuAngVelKeys(k),rthighImuGyros(imuIdxArray(k),:)');
                obj.m_initialValues.insert(obj.m_rshankImuAngVelKeys(k),rshankImuGyros(imuIdxArray(k),:)');
                obj.m_initialValues.insert(obj.m_rfootImuAngVelKeys(k),rfootImuGyros(imuIdxArray(k),:)');
                obj.m_initialValues.insert(obj.m_lthighImuAngVelKeys(k),lthighImuGyros(imuIdxArray(k),:)');
                obj.m_initialValues.insert(obj.m_lshankImuAngVelKeys(k),lshankImuGyros(imuIdxArray(k),:)');
                obj.m_initialValues.insert(obj.m_lfootImuAngVelKeys(k),lfootImuGyros(imuIdxArray(k),:)');
            end
        end
        
        function addImuProblemSolutionsToInitialValues(obj)
            obj.m_initialValues.insert(obj.m_sacrumImuPoseProblem.m_estimate);
            obj.m_initialValues.insert(obj.m_rthighImuPoseProblem.m_estimate);
            obj.m_initialValues.insert(obj.m_rshankImuPoseProblem.m_estimate);
            obj.m_initialValues.insert(obj.m_rfootImuPoseProblem.m_estimate);
            obj.m_initialValues.insert(obj.m_lthighImuPoseProblem.m_estimate);
            obj.m_initialValues.insert(obj.m_lshankImuPoseProblem.m_estimate);
            obj.m_initialValues.insert(obj.m_lfootImuPoseProblem.m_estimate);
        end
        
        function addImuProblemGraphsToGraph(obj)
            obj.m_graph.push_back(obj.m_sacrumImuPoseProblem.m_graph);
            obj.m_graph.push_back(obj.m_rthighImuPoseProblem.m_graph);
            obj.m_graph.push_back(obj.m_rshankImuPoseProblem.m_graph);
            obj.m_graph.push_back(obj.m_rfootImuPoseProblem.m_graph);
            obj.m_graph.push_back(obj.m_lthighImuPoseProblem.m_graph);
            obj.m_graph.push_back(obj.m_lshankImuPoseProblem.m_graph);
            obj.m_graph.push_back(obj.m_lfootImuPoseProblem.m_graph);
        end
        
        function setup(obj)
            setupTic=tic;
            fprintf('Setting up lowerBodyPoseEstimator (''%s'').',obj.id);
            % () possible initial value override from files
            if exist(obj.m_setInitialStaticVectorsFromFile,'file') % if is valid file, set initial static vectors
                setInitialStaticVectorsFromH5File(obj,obj.m_setInitialStaticVectorsFromFile);
            end
            obj.m_graph=gtsam.NonlinearFactorGraph;
            obj.m_initialValues=gtsam.Values;
            % () pull out optimized values from individual pose problems
            addImuProblemSolutionsToInitialValues(obj);
            % () add the (disparate) IMU graphs to the knee graph
            addImuProblemGraphsToGraph(obj);
            % initial setup of relevant keys
            obj.m_rkneeAxisThighKey = gtsam.symbol(obj.m_rkneeAxisThighVarChar,0);
            obj.m_rkneeAxisShankKey = gtsam.symbol(obj.m_rkneeAxisShankVarChar,0);
            obj.m_lkneeAxisThighKey = gtsam.symbol(obj.m_lkneeAxisThighVarChar,0);
            obj.m_lkneeAxisShankKey = gtsam.symbol(obj.m_lkneeAxisShankVarChar,0);
            % () setup the angular velocity estimation constraint
            addInstantaneousAngVelEstimationToImuPoses(obj);
            % () hinge axis priors?
            if obj.m_usePriorsOnKneeHingeAxes % add priors on these hinge axes
                obj.m_graph.push_back(gtsam.PriorFactorUnit3(obj.m_rkneeAxisThighKey,obj.priorAxisRThigh,gtsam.noiseModel.Diagonal.Sigmas(obj.rkneeHingeAxisThighPriorStd(:))));
                obj.m_graph.push_back(gtsam.PriorFactorUnit3(obj.m_rkneeAxisShankKey,obj.priorAxisRShank,gtsam.noiseModel.Diagonal.Sigmas(obj.rkneeHingeAxisShankPriorStd(:))));
                obj.m_graph.push_back(gtsam.PriorFactorUnit3(obj.m_lkneeAxisThighKey,obj.priorAxisLThigh,gtsam.noiseModel.Diagonal.Sigmas(obj.lkneeHingeAxisThighPriorStd(:))));
                obj.m_graph.push_back(gtsam.PriorFactorUnit3(obj.m_lkneeAxisShankKey,obj.priorAxisLShank,gtsam.noiseModel.Diagonal.Sigmas(obj.lkneeHingeAxisShankPriorStd(:))));
            end
            % () insert initial estimates for axes
            obj.m_initialValues.insert(obj.m_rkneeAxisThighKey,obj.priorAxisRThigh);
            obj.m_initialValues.insert(obj.m_rkneeAxisShankKey,obj.priorAxisRShank);
            obj.m_initialValues.insert(obj.m_lkneeAxisThighKey,obj.priorAxisLThigh);
            obj.m_initialValues.insert(obj.m_lkneeAxisShankKey,obj.priorAxisLShank);
            % () add keys, Values (initial), and prior factors on the static offset vectors
            obj.m_sacrumImuToRHipCtrKey=gtsam.symbol(obj.m_sacrumImuToRHipCenterVarChar,0);
            obj.m_sacrumImuToLHipCtrKey=gtsam.symbol(obj.m_sacrumImuToLHipCenterVarChar,0);
            obj.m_rthighImuToKneeCtrKey=gtsam.symbol(obj.m_rthighImuToKneeCenterVarChar,0); % keys
            obj.m_rthighImuToHipCtrKey=gtsam.symbol(obj.m_rthighImuToHipCenterVarChar,0);
            obj.m_rshankImuToKneeCtrKey=gtsam.symbol(obj.m_rshankImuToKneeCenterVarChar,0);
            obj.m_rshankImuToAnkleCtrKey=gtsam.symbol(obj.m_rshankImuToAnkleCenterVarChar,0);
            obj.m_rfootImuToAnkleCtrKey=gtsam.symbol(obj.m_rfootImuToAnkleCenterVarChar,0);
            obj.m_lthighImuToKneeCtrKey=gtsam.symbol(obj.m_lthighImuToKneeCenterVarChar,0); % keys
            obj.m_lthighImuToHipCtrKey=gtsam.symbol(obj.m_lthighImuToHipCenterVarChar,0);
            obj.m_lshankImuToKneeCtrKey=gtsam.symbol(obj.m_lshankImuToKneeCenterVarChar,0);
            obj.m_lshankImuToAnkleCtrKey=gtsam.symbol(obj.m_lshankImuToAnkleCenterVarChar,0);
            obj.m_lfootImuToAnkleCtrKey=gtsam.symbol(obj.m_lfootImuToAnkleCenterVarChar,0);
            obj.m_initialValues.insert(obj.m_sacrumImuToRHipCtrKey,obj.priorSacrumImuToRHipCtr); % insert initial values
            obj.m_initialValues.insert(obj.m_sacrumImuToLHipCtrKey,obj.priorSacrumImuToLHipCtr);
            obj.m_initialValues.insert(obj.m_rthighImuToKneeCtrKey,obj.priorRThighImuToKneeCtr);
            obj.m_initialValues.insert(obj.m_rthighImuToHipCtrKey,obj.priorRThighImuToHipCtr);
            obj.m_initialValues.insert(obj.m_rshankImuToKneeCtrKey,obj.priorRShankImuToKneeCtr);
            obj.m_initialValues.insert(obj.m_rshankImuToAnkleCtrKey,obj.priorRShankImuToAnkleCtr);
            obj.m_initialValues.insert(obj.m_rfootImuToAnkleCtrKey,obj.priorRFootImuToAnkleCtr);
            obj.m_initialValues.insert(obj.m_lthighImuToKneeCtrKey,obj.priorLThighImuToKneeCtr);
            obj.m_initialValues.insert(obj.m_lthighImuToHipCtrKey,obj.priorLThighImuToHipCtr);
            obj.m_initialValues.insert(obj.m_lshankImuToKneeCtrKey,obj.priorLShankImuToKneeCtr);
            obj.m_initialValues.insert(obj.m_lshankImuToAnkleCtrKey,obj.priorLShankImuToAnkleCtr);
            obj.m_initialValues.insert(obj.m_lfootImuToAnkleCtrKey,obj.priorLFootImuToAnkleCtr);
            if obj.m_usePriorsOnImuToJointCtrVectors % add priors to these static vectors
                imuToJointCtrVecPriorNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.imuToJointCtrPriorStd(:));
                % until you've implemented the method to clear these priors in order to set new ones for calibraiton purposes, I'm not setitngs these for now.
                % NOTE: you need to implement that method.
                %                 obj.m_graph.push_back(gtsam.PriorFactorPoint3(obj.m_sacrumImuToRHipCtrKey,obj.priorSacrumImuToRHipCtr,imuToJointCtrVecPriorNoiseModel));
                %                 obj.m_graph.push_back(gtsam.PriorFactorPoint3(obj.m_sacrumImuToLHipCtrKey,obj.priorSacrumImuToLHipCtr,imuToJointCtrVecPriorNoiseModel));
                %                 obj.m_graph.push_back(gtsam.PriorFactorPoint3(obj.m_rthighImuToHipCtrKey,obj.priorRThighImuToHipCtr,imuToJointCtrVecPriorNoiseModel));
                obj.m_graph.push_back(gtsam.PriorFactorPoint3(obj.m_rthighImuToKneeCtrKey,obj.priorRThighImuToKneeCtr,imuToJointCtrVecPriorNoiseModel));
                obj.m_graph.push_back(gtsam.PriorFactorPoint3(obj.m_rshankImuToKneeCtrKey,obj.priorRShankImuToKneeCtr,imuToJointCtrVecPriorNoiseModel));
                obj.m_graph.push_back(gtsam.PriorFactorPoint3(obj.m_rshankImuToAnkleCtrKey,obj.priorRShankImuToAnkleCtr,imuToJointCtrVecPriorNoiseModel));
                obj.m_graph.push_back(gtsam.PriorFactorPoint3(obj.m_rfootImuToAnkleCtrKey,obj.priorRFootImuToAnkleCtr,imuToJointCtrVecPriorNoiseModel));
                %                 obj.m_graph.push_back(gtsam.PriorFactorPoint3(obj.m_lthighImuToHipCtrKey,obj.priorLThighImuToHipCtr,imuToJointCtrVecPriorNoiseModel));
                obj.m_graph.push_back(gtsam.PriorFactorPoint3(obj.m_lthighImuToKneeCtrKey,obj.priorLThighImuToKneeCtr,imuToJointCtrVecPriorNoiseModel));
                obj.m_graph.push_back(gtsam.PriorFactorPoint3(obj.m_lshankImuToKneeCtrKey,obj.priorLShankImuToKneeCtr,imuToJointCtrVecPriorNoiseModel));
                obj.m_graph.push_back(gtsam.PriorFactorPoint3(obj.m_lshankImuToAnkleCtrKey,obj.priorLShankImuToAnkleCtr,imuToJointCtrVecPriorNoiseModel));
                obj.m_graph.push_back(gtsam.PriorFactorPoint3(obj.m_lfootImuToAnkleCtrKey,obj.priorLFootImuToAnkleCtr,imuToJointCtrVecPriorNoiseModel));
            end
            % () hinge axis constraint for knees
            for k=1:(length(obj.m_rthighImuPoseProblem.m_poseKeys))
                obj.m_graph.push_back(bioslam.HingeJointConstraintVecErrEstAngVel(obj.m_rthighImuPoseProblem.m_poseKeys(k),obj.m_rthighImuAngVelKeys(k),obj.m_rshankImuPoseProblem.m_poseKeys(k),obj.m_rshankImuAngVelKeys(k),obj.m_rkneeAxisThighKey,obj.m_kneeHingeAxisNoiseModel));
                obj.m_graph.push_back(bioslam.HingeJointConstraintVecErrEstAngVel(obj.m_rshankImuPoseProblem.m_poseKeys(k),obj.m_rshankImuAngVelKeys(k),obj.m_rthighImuPoseProblem.m_poseKeys(k),obj.m_rthighImuAngVelKeys(k),obj.m_rkneeAxisShankKey,obj.m_kneeHingeAxisNoiseModel));
                obj.m_graph.push_back(bioslam.HingeJointConstraintVecErrEstAngVel(obj.m_lthighImuPoseProblem.m_poseKeys(k),obj.m_lthighImuAngVelKeys(k),obj.m_lshankImuPoseProblem.m_poseKeys(k),obj.m_lshankImuAngVelKeys(k),obj.m_lkneeAxisThighKey,obj.m_kneeHingeAxisNoiseModel));
                obj.m_graph.push_back(bioslam.HingeJointConstraintVecErrEstAngVel(obj.m_lshankImuPoseProblem.m_poseKeys(k),obj.m_lshankImuAngVelKeys(k),obj.m_lthighImuPoseProblem.m_poseKeys(k),obj.m_lthighImuAngVelKeys(k),obj.m_lkneeAxisShankKey,obj.m_kneeHingeAxisNoiseModel));
            end
            % () joint center position constraints
            for k=1:(length(obj.m_rthighImuPoseProblem.m_poseKeys))
                % right knee joint center connection
                rKneeJointCenterFactor=bioslam.ConstrainedJointCenterPositionFactor(obj.m_rthighImuPoseProblem.m_poseKeys(k),obj.m_rshankImuPoseProblem.m_poseKeys(k),obj.m_rthighImuToKneeCtrKey,obj.m_rshankImuToKneeCtrKey,obj.m_kneeJointCtrConnectionNoiseModel);
                obj.m_graph.push_back(rKneeJointCenterFactor);
                lKneeJointCenterFactor=bioslam.ConstrainedJointCenterPositionFactor(obj.m_lthighImuPoseProblem.m_poseKeys(k),obj.m_lshankImuPoseProblem.m_poseKeys(k),obj.m_lthighImuToKneeCtrKey,obj.m_lshankImuToKneeCtrKey,obj.m_kneeJointCtrConnectionNoiseModel);
                obj.m_graph.push_back(lKneeJointCenterFactor);
                % right hip joint center connection
                rHipJointCenterFactor=bioslam.ConstrainedJointCenterPositionFactor(obj.m_sacrumImuPoseProblem.m_poseKeys(k),obj.m_rthighImuPoseProblem.m_poseKeys(k),obj.m_sacrumImuToRHipCtrKey,obj.m_rthighImuToHipCtrKey,obj.m_hipJointCtrConnectionNoiseModel);
                obj.m_graph.push_back(rHipJointCenterFactor);
                % left hip joint center connection
                lHipJointCenterFactor=bioslam.ConstrainedJointCenterPositionFactor(obj.m_sacrumImuPoseProblem.m_poseKeys(k),obj.m_lthighImuPoseProblem.m_poseKeys(k),obj.m_sacrumImuToLHipCtrKey,obj.m_lthighImuToHipCtrKey,obj.m_hipJointCtrConnectionNoiseModel);
                obj.m_graph.push_back(lHipJointCenterFactor);
                % right ankle joint center connection
                rAnkleJointCenterFactor=bioslam.ConstrainedJointCenterPositionFactor(obj.m_rshankImuPoseProblem.m_poseKeys(k),obj.m_rfootImuPoseProblem.m_poseKeys(k),obj.m_rshankImuToAnkleCtrKey,obj.m_rfootImuToAnkleCtrKey,obj.m_ankleJointCtrConnectionNoiseModel);
                obj.m_graph.push_back(rAnkleJointCenterFactor);
                % left ankle joint center connection
                lAnkleJointCenterFactor=bioslam.ConstrainedJointCenterPositionFactor(obj.m_lshankImuPoseProblem.m_poseKeys(k),obj.m_lfootImuPoseProblem.m_poseKeys(k),obj.m_lshankImuToAnkleCtrKey,obj.m_lfootImuToAnkleCtrKey,obj.m_ankleJointCtrConnectionNoiseModel);
                obj.m_graph.push_back(lAnkleJointCenterFactor);
            end
            % () optional: joint center velocity constraints
            if obj.m_useJointVelFactor % constrain velocity at joints
                for k=1:length(obj.m_sacrumImuAngVelKeys)
                    % RHip joint
                    obj.m_graph.push_back(bioslam.ConstrainedJointCenterVelocityFactor(obj.m_sacrumImuPoseProblem.m_poseKeys(k),obj.m_sacrumImuPoseProblem.m_velKeys(k),obj.m_sacrumImuAngVelKeys(k),obj.m_sacrumImuToRHipCtrKey,obj.m_rthighImuPoseProblem.m_poseKeys(k),obj.m_rthighImuPoseProblem.m_velKeys(k),obj.m_rthighImuAngVelKeys(k),obj.m_rthighImuToHipCtrKey,obj.m_jointCtrVelConnectionNoiseModel));
                    % RKnee joint
                    obj.m_graph.push_back(bioslam.ConstrainedJointCenterVelocityFactor(obj.m_rthighImuPoseProblem.m_poseKeys(k),obj.m_rthighImuPoseProblem.m_velKeys(k),obj.m_rthighImuAngVelKeys(k),obj.m_rthighImuToKneeCtrKey,obj.m_rshankImuPoseProblem.m_poseKeys(k),obj.m_rshankImuPoseProblem.m_velKeys(k),obj.m_rshankImuAngVelKeys(k),obj.m_rshankImuToKneeCtrKey,obj.m_jointCtrVelConnectionNoiseModel));
                    % RAnkle joint
                    obj.m_graph.push_back(bioslam.ConstrainedJointCenterVelocityFactor(obj.m_rshankImuPoseProblem.m_poseKeys(k),obj.m_rshankImuPoseProblem.m_velKeys(k),obj.m_rshankImuAngVelKeys(k),obj.m_rshankImuToAnkleCtrKey,obj.m_rfootImuPoseProblem.m_poseKeys(k),obj.m_rfootImuPoseProblem.m_velKeys(k),obj.m_rfootImuAngVelKeys(k),obj.m_rfootImuToAnkleCtrKey,obj.m_jointCtrVelConnectionNoiseModel));
                    % LHip joint
                    obj.m_graph.push_back(bioslam.ConstrainedJointCenterVelocityFactor(obj.m_sacrumImuPoseProblem.m_poseKeys(k),obj.m_sacrumImuPoseProblem.m_velKeys(k),obj.m_sacrumImuAngVelKeys(k),obj.m_sacrumImuToLHipCtrKey,obj.m_lthighImuPoseProblem.m_poseKeys(k),obj.m_lthighImuPoseProblem.m_velKeys(k),obj.m_lthighImuAngVelKeys(k),obj.m_lthighImuToHipCtrKey,obj.m_jointCtrVelConnectionNoiseModel));
                    % LKnee joint
                    obj.m_graph.push_back(bioslam.ConstrainedJointCenterVelocityFactor(obj.m_lthighImuPoseProblem.m_poseKeys(k),obj.m_lthighImuPoseProblem.m_velKeys(k),obj.m_lthighImuAngVelKeys(k),obj.m_lthighImuToKneeCtrKey,obj.m_lshankImuPoseProblem.m_poseKeys(k),obj.m_lshankImuPoseProblem.m_velKeys(k),obj.m_lshankImuAngVelKeys(k),obj.m_lshankImuToKneeCtrKey,obj.m_jointCtrVelConnectionNoiseModel));
                    % LAnkle joint
                    obj.m_graph.push_back(bioslam.ConstrainedJointCenterVelocityFactor(obj.m_lshankImuPoseProblem.m_poseKeys(k),obj.m_lshankImuPoseProblem.m_velKeys(k),obj.m_lshankImuAngVelKeys(k),obj.m_lshankImuToAnkleCtrKey,obj.m_lfootImuPoseProblem.m_poseKeys(k),obj.m_lfootImuPoseProblem.m_velKeys(k),obj.m_lfootImuAngVelKeys(k),obj.m_lfootImuToAnkleCtrKey,obj.m_jointCtrVelConnectionNoiseModel));
                end
            end
            % () add segment length constraints
            if obj.m_useRFemurLengthFactor
                rthighSegmentLengthFactor=bioslam.SegmentLengthMagnitudeFactor(obj.m_rthighImuToHipCtrKey,obj.m_rthighImuToKneeCtrKey,obj.m_rfemurLength,obj.m_rfemurLengthNoiseModel);
                obj.m_graph.push_back(rthighSegmentLengthFactor);
                if obj.m_useMaxAnthroConstraint
                    % add 99th percentile male femur length which is 48cm. source: ANSUR II D29 Thigh Link http://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf
                    obj.m_graph.push_back(bioslam.SegmentLengthMaxMagnitudeFactor(obj.m_rthighImuToHipCtrKey,obj.m_rthighImuToKneeCtrKey,0.48,obj.m_segmentLengthMaxNoiseModel));
                end
                if obj.m_useMinAnthroConstraint
                    % add 1st percentile female femur length which is 32.6cm. source: ANSUR II D29 Thigh Link http://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf
                    obj.m_graph.push_back(bioslam.SegmentLengthMinMagnitudeFactor(obj.m_rthighImuToHipCtrKey,obj.m_rthighImuToKneeCtrKey,0.326,obj.m_segmentLengthMinNoiseModel));
                end
            end
            if obj.m_useRTibiaLengthFactor
                rshankSegmentLengthFactor=bioslam.SegmentLengthMagnitudeFactor(obj.m_rshankImuToKneeCtrKey,obj.m_rshankImuToAnkleCtrKey,obj.m_rtibiaLength,obj.m_rtibiaLengthNoiseModel);
                obj.m_graph.push_back(rshankSegmentLengthFactor);
                if obj.m_useMaxAnthroConstraint
                    % add 99th percentile male tibia length which is 47.90cm. source: ANSUR II D6 Calf Link http://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf
                    obj.m_graph.push_back(bioslam.SegmentLengthMaxMagnitudeFactor(obj.m_rshankImuToKneeCtrKey,obj.m_rshankImuToAnkleCtrKey,0.479,obj.m_segmentLengthMaxNoiseModel));
                end
                if obj.m_useMinAnthroConstraint
                    % add 1st percentile female tibia length which is 34.4cm. source: ANSUR II D6 Calf Link http://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf
                    obj.m_graph.push_back(bioslam.SegmentLengthMinMagnitudeFactor(obj.m_rshankImuToKneeCtrKey,obj.m_rshankImuToAnkleCtrKey,0.344,obj.m_segmentLengthMinNoiseModel));
                end
            end
            if obj.m_useLFemurLengthFactor
                lthighSegmentLengthFactor=bioslam.SegmentLengthMagnitudeFactor(obj.m_lthighImuToHipCtrKey,obj.m_lthighImuToKneeCtrKey,obj.m_lfemurLength,obj.m_lfemurLengthNoiseModel);
                obj.m_graph.push_back(lthighSegmentLengthFactor);
                if obj.m_useMaxAnthroConstraint
                    % add 99th percentile male femur length which is 48cm. source: ANSUR II D29 Thigh Link http://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf
                    obj.m_graph.push_back(bioslam.SegmentLengthMaxMagnitudeFactor(obj.m_lthighImuToHipCtrKey,obj.m_lthighImuToKneeCtrKey,0.48,obj.m_segmentLengthMaxNoiseModel));
                end
                if obj.m_useMinAnthroConstraint
                    % add 1st percentile female femur length which is 32.6cm. source: ANSUR II D29 Thigh Link http://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf
                    obj.m_graph.push_back(bioslam.SegmentLengthMinMagnitudeFactor(obj.m_lthighImuToHipCtrKey,obj.m_lthighImuToKneeCtrKey,0.326,obj.m_segmentLengthMinNoiseModel));
                end
            end
            if obj.m_useLTibiaLengthFactor
                lshankSegmentLengthFactor=bioslam.SegmentLengthMagnitudeFactor(obj.m_lshankImuToKneeCtrKey,obj.m_lshankImuToAnkleCtrKey,obj.m_ltibiaLength,obj.m_ltibiaLengthNoiseModel);
                obj.m_graph.push_back(lshankSegmentLengthFactor);
                if obj.m_useMaxAnthroConstraint
                    % add 99th percentile male tibia length which is 47.90cm. source: ANSUR II D6 Calf Link http://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf
                    obj.m_graph.push_back(bioslam.SegmentLengthMaxMagnitudeFactor(obj.m_lshankImuToKneeCtrKey,obj.m_lshankImuToAnkleCtrKey,0.479,obj.m_segmentLengthMaxNoiseModel));
                end
                if obj.m_useMinAnthroConstraint
                    % add 1st percentile female tibia length which is 34.4cm. source: ANSUR II D6 Calf Link http://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf
                    obj.m_graph.push_back(bioslam.SegmentLengthMinMagnitudeFactor(obj.m_lshankImuToKneeCtrKey,obj.m_lshankImuToAnkleCtrKey,0.344,obj.m_segmentLengthMinNoiseModel));
                end
            end
            if obj.m_usePelvicWidthFactor
                pelvicWidthSegmentLengthFactor=bioslam.SegmentLengthMagnitudeFactor(obj.m_sacrumImuToLHipCtrKey,obj.m_sacrumImuToRHipCtrKey,obj.m_pelvicWidth,obj.m_pelvicWidthNoiseModel);
                obj.m_graph.push_back(pelvicWidthSegmentLengthFactor);
                if obj.m_useMaxAnthroConstraint
                    % add 99th percentile male hip breath which is 40.9cm. source: ANSUR II http://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf
                    obj.m_graph.push_back(bioslam.SegmentLengthMaxMagnitudeFactor(obj.m_sacrumImuToLHipCtrKey,obj.m_sacrumImuToRHipCtrKey,0.409,obj.m_segmentLengthMaxNoiseModel));
                end
                if obj.m_useMinAnthroConstraint
                    % todo: get an actual value for this! just gonna use like 10cm for right now. No one should have a smaller femoral head separation than that.
                    obj.m_graph.push_back(bioslam.SegmentLengthMinMagnitudeFactor(obj.m_sacrumImuToLHipCtrKey,obj.m_sacrumImuToRHipCtrKey,0.1,obj.m_segmentLengthMaxNoiseModel));
                end
            end
            % () optional: use femur length discrepancy factor
            if obj.m_useFemurLengthDiscrepancyFactor
                obj.m_graph.push_back( bioslam.SegmentLengthDiscrepancyFactor(obj.m_rthighImuToHipCtrKey,obj.m_rthighImuToKneeCtrKey,obj.m_lthighImuToHipCtrKey,obj.m_lthighImuToKneeCtrKey,obj.m_femurLengthDiscrepancyNoiseModel) );
            end
            % () optional: use tibia length discrepancy factor
            if obj.m_useTibiaLengthDiscrepancyFactor
                obj.m_graph.push_back( bioslam.SegmentLengthDiscrepancyFactor(obj.m_rshankImuToKneeCtrKey,obj.m_rshankImuToAnkleCtrKey,obj.m_lshankImuToKneeCtrKey,obj.m_lshankImuToAnkleCtrKey,obj.m_tibiaLengthDiscrepancyNoiseModel) );
            end
            % () optional: constrain angle between between tibia segment and knee axis as expressed in the shank IMU frame
            if obj.m_useKneeAxisTibiaOrthogonalityFactor
                % remember that the axis should point right! So setup these angles assuming the knee axis points right.
                obj.m_graph.push_back(bioslam.AngleBetweenAxisAndSegmentFactor(obj.m_rkneeAxisShankKey,obj.m_rshankImuToKneeCtrKey,obj.m_rshankImuToAnkleCtrKey,obj.m_angBwKneeAxisAndSegmentMeanRTibia,obj.m_angBwKneeAxisAndSegmentTibiaNoiseModel));
                obj.m_graph.push_back(bioslam.AngleBetweenAxisAndSegmentFactor(obj.m_lkneeAxisShankKey,obj.m_lshankImuToKneeCtrKey,obj.m_lshankImuToAnkleCtrKey,obj.m_angBwKneeAxisAndSegmentMeanLTibia,obj.m_angBwKneeAxisAndSegmentTibiaNoiseModel));
            end
            % () optional: constrain angle between between femur segment and knee axis as expressed in the thigh IMU frame
            if obj.m_useKneeAxisFemurOrthogonalityFactor
                % remember that the axis should point right! So setup these angles assuming the knee axis points right.
                obj.m_graph.push_back(bioslam.AngleBetweenAxisAndSegmentFactor(obj.m_rkneeAxisThighKey,obj.m_rthighImuToHipCtrKey,obj.m_rthighImuToKneeCtrKey,obj.m_angBwKneeAxisAndSegmentMeanRFemur,obj.m_angBwKneeAxisAndSegmentFemurNoiseModel));
                obj.m_graph.push_back(bioslam.AngleBetweenAxisAndSegmentFactor(obj.m_lkneeAxisThighKey,obj.m_lthighImuToHipCtrKey,obj.m_lthighImuToKneeCtrKey,obj.m_angBwKneeAxisAndSegmentMeanLFemur,obj.m_angBwKneeAxisAndSegmentFemurNoiseModel));
            end
            % () optional: if assuming hip is a hinge, then add keys and factors
            %       for this we'll just use the relative angular velocity version.
            if obj.m_assumeHipHinge
                % add keys for these four axes
                obj.addHipHingeAxisVariableKeys();
                % add those factors
                for k=1:(length(obj.m_rthighImuPoseProblem.m_poseKeys))
                    obj.m_graph.push_back(bioslam.HingeJointConstraintVecErrEstAngVel(obj.m_sacrumImuPoseProblem.m_poseKeys(k), obj.m_sacrumImuAngVelKeys(k), obj.m_rthighImuPoseProblem.m_poseKeys(k), obj.m_rthighImuAngVelKeys(k), obj.m_hipAxisSacrumKey, obj.m_hipHingeAxisNoiseModel));
                    obj.m_graph.push_back(bioslam.HingeJointConstraintVecErrEstAngVel(obj.m_sacrumImuPoseProblem.m_poseKeys(k), obj.m_sacrumImuAngVelKeys(k), obj.m_lthighImuPoseProblem.m_poseKeys(k), obj.m_lthighImuAngVelKeys(k), obj.m_hipAxisSacrumKey, obj.m_hipHingeAxisNoiseModel));
                    obj.m_graph.push_back(bioslam.HingeJointConstraintVecErrEstAngVel(obj.m_rthighImuPoseProblem.m_poseKeys(k), obj.m_rthighImuAngVelKeys(k), obj.m_sacrumImuPoseProblem.m_poseKeys(k), obj.m_sacrumImuAngVelKeys(k), obj.m_rhipAxisThighKey, obj.m_hipHingeAxisNoiseModel));
                    obj.m_graph.push_back(bioslam.HingeJointConstraintVecErrEstAngVel(obj.m_lthighImuPoseProblem.m_poseKeys(k), obj.m_lthighImuAngVelKeys(k), obj.m_sacrumImuPoseProblem.m_poseKeys(k), obj.m_sacrumImuAngVelKeys(k), obj.m_lhipAxisThighKey, obj.m_hipHingeAxisNoiseModel));
                end
                % add initial values for hip hinge axes
                obj.m_initialValues.insert(obj.m_hipAxisSacrumKey,obj.priorHipAxisSacrum);
                obj.m_initialValues.insert(obj.m_rhipAxisThighKey,obj.priorRHipAxisThigh);
                obj.m_initialValues.insert(obj.m_lhipAxisThighKey,obj.priorLHipAxisThigh);
                if obj.m_usePriorsOnHipHingeAxes % also add priors
                    obj.m_graph.push_back(gtsam.PriorFactorUnit3(obj.m_hipAxisSacrumKey,obj.priorHipAxisSacrum,gtsam.noiseModel.Diagonal.Sigmas(obj.hipHingeAxisSacrumPriorStd(:))));
                    obj.m_graph.push_back(gtsam.PriorFactorUnit3(obj.m_rhipAxisThighKey,obj.priorRHipAxisThigh,gtsam.noiseModel.Diagonal.Sigmas(obj.rhipHingeAxisThighPriorStd(:))));
                    obj.m_graph.push_back(gtsam.PriorFactorUnit3(obj.m_lhipAxisThighKey,obj.priorLHipAxisThigh,gtsam.noiseModel.Diagonal.Sigmas(obj.lhipHingeAxisThighPriorStd(:))));
                end
            end
            % () optional: if assuming ankle is a hinge, then add keys and factors
            if obj.m_assumeAnkleHinge
                % add keys for these axes
                obj.addAnkleHingeAxisVariableKeys();
                % add those factors
                for k=1:(length(obj.m_rthighImuPoseProblem.m_poseKeys))
                    obj.m_graph.push_back(bioslam.HingeJointConstraintVecErrEstAngVel(obj.m_rfootImuPoseProblem.m_poseKeys(k), obj.m_rfootImuAngVelKeys(k), obj.m_rshankImuPoseProblem.m_poseKeys(k), obj.m_rshankImuAngVelKeys(k), obj.m_rankleAxisFootKey, obj.m_ankleHingeAxisNoiseModel));
                    obj.m_graph.push_back(bioslam.HingeJointConstraintVecErrEstAngVel(obj.m_lfootImuPoseProblem.m_poseKeys(k), obj.m_lfootImuAngVelKeys(k), obj.m_lshankImuPoseProblem.m_poseKeys(k), obj.m_lshankImuAngVelKeys(k), obj.m_lankleAxisFootKey, obj.m_ankleHingeAxisNoiseModel));
                    obj.m_graph.push_back(bioslam.HingeJointConstraintVecErrEstAngVel(obj.m_rshankImuPoseProblem.m_poseKeys(k), obj.m_rshankImuAngVelKeys(k), obj.m_rfootImuPoseProblem.m_poseKeys(k), obj.m_rfootImuAngVelKeys(k), obj.m_rankleAxisShankKey, obj.m_ankleHingeAxisNoiseModel));
                    obj.m_graph.push_back(bioslam.HingeJointConstraintVecErrEstAngVel(obj.m_lshankImuPoseProblem.m_poseKeys(k), obj.m_lshankImuAngVelKeys(k), obj.m_lfootImuPoseProblem.m_poseKeys(k), obj.m_lfootImuAngVelKeys(k), obj.m_lankleAxisShankKey, obj.m_ankleHingeAxisNoiseModel));
                end
                % add initial values for ankle hinge axes
                obj.m_initialValues.insert(obj.m_rankleAxisFootKey,obj.priorAnkleAxisRFoot);
                obj.m_initialValues.insert(obj.m_lankleAxisFootKey,obj.priorAnkleAxisLFoot);
                obj.m_initialValues.insert(obj.m_rankleAxisShankKey,obj.priorAnkleAxisRShank);
                obj.m_initialValues.insert(obj.m_lankleAxisShankKey,obj.priorAnkleAxisLShank);
                if obj.m_usePriorsOnAnkleHingeAxes % also add priors
                    obj.m_graph.push_back(gtsam.PriorFactorUnit3(obj.m_rankleAxisFootKey,obj.priorAnkleAxisRFoot,gtsam.noiseModel.Diagonal.Sigmas(obj.rankleHingeAxisFootPriorStd(:))));
                    obj.m_graph.push_back(gtsam.PriorFactorUnit3(obj.m_lankleAxisFootKey,obj.priorAnkleAxisLFoot,gtsam.noiseModel.Diagonal.Sigmas(obj.lankleHingeAxisFootPriorStd(:))));
                    obj.m_graph.push_back(gtsam.PriorFactorUnit3(obj.m_rankleAxisShankKey,obj.priorAnkleAxisRShank,gtsam.noiseModel.Diagonal.Sigmas(obj.rankleHingeAxisShankPriorStd(:))));
                    obj.m_graph.push_back(gtsam.PriorFactorUnit3(obj.m_lankleAxisShankKey,obj.priorAnkleAxisLShank,gtsam.noiseModel.Diagonal.Sigmas(obj.lankleHingeAxisShankPriorStd(:))));
                end
            end
            % () pull up time to this object
            obj.m_time=obj.m_rthighImuPoseProblem.m_time;
            % --- setup is complete!
            fprintf(' completed! %.4f sec\n',toc(setupTic));
            obj.m_isSetup=1;
            % apply initialization scheme (note: MATLAB implementation does not support all of the same schemes as C++ version)
            switch obj.m_initializationScheme
                case 0
                    % do nothing (same as setting zero to IMU trajectories)
                case 4 % // init imu values via (a) adjust distal imu orientation at each joint to make joint angles in bounds and (b) straighten positions to make joint centers consistent
                    setImuOrientationsBasedOnJointAngleLimits(obj);
                    setImuPositionsBasedOnConsistentInitialStaticVecsToJointCtrs(obj);
                otherwise
                    error('unknown initialization scheme');
            end
        end
        
        function addNewPoint3PriorToKey(obj,key,value,std)
            % clear previous priors and add new prior
            % useful if you've previously calibrated for this quantity
            assert(~isempty(key),'key is empty');
            % () check for any existing PriorFactor<Point3>
            clearGtsamPriorFactorPoint3ByKey(obj,key);
            % () construct new prior factor from input vector value and std
            priorNoiseModel=gtsam.noiseModel.Diagonal.Sigmas([std, std, std]');
            newPrior=gtsam.Point3(value(1),value(2),value(3));
            newPriorFactor=gtsam.PriorFactorPoint3(key,newPrior,priorNoiseModel);
            % () add new prior factor to graph
            obj.m_graph.push_back(newPriorFactor);
            % () update initial value by key
            obj.m_initialValues.update(key,newPrior);
        end
        
        function addHipHingeAxisVariableKeys(obj)
            obj.m_hipAxisSacrumVarStr = strcat(obj.id,'_hipAxisSacrum');
            obj.m_hipAxisSacrumVarChar=VarStrToCharMap.insert(obj.m_hipAxisSacrumVarStr);
            obj.m_hipAxisSacrumKey = gtsam.symbol(obj.m_hipAxisSacrumVarChar,0);
            % also setup unique hip axis thigh keys
            obj.m_rhipAxisThighVarStr = strcat(obj.id,'_rhipAxisThigh');
            obj.m_rhipAxisThighVarChar=VarStrToCharMap.insert(obj.m_rhipAxisThighVarStr);
            obj.m_rhipAxisThighKey = gtsam.symbol(obj.m_rhipAxisThighVarChar,0);
            obj.m_lhipAxisThighVarStr = strcat(obj.id,'_lhipAxisThigh');
            obj.m_lhipAxisThighVarChar=VarStrToCharMap.insert(obj.m_lhipAxisThighVarStr);
            obj.m_lhipAxisThighKey = gtsam.symbol(obj.m_lhipAxisThighVarChar,0);
        end
        
        function addAnkleHingeAxisVariableKeys(obj)
            obj.m_rankleAxisFootVarStr = strcat(obj.id,'_rankleAxisFoot');
            obj.m_rankleAxisFootVarChar=VarStrToCharMap.insert(obj.m_rankleAxisFootVarStr);
            obj.m_rankleAxisFootKey = gtsam.symbol(obj.m_rankleAxisFootVarChar,0);
            obj.m_lankleAxisFootVarStr = strcat(obj.id,'_lankleAxisFoot');
            obj.m_lankleAxisFootVarChar=VarStrToCharMap.insert(obj.m_lankleAxisFootVarStr);
            obj.m_lankleAxisFootKey = gtsam.symbol(obj.m_lankleAxisFootVarChar,0);
            % also setup unique ankle axis shank keys
            obj.m_rankleAxisShankVarStr = strcat(obj.id,'_rankleAxisShank');
            obj.m_rankleAxisShankVarChar=VarStrToCharMap.insert(obj.m_rankleAxisShankVarStr);
            obj.m_rankleAxisShankKey = gtsam.symbol(obj.m_rankleAxisShankVarChar,0);
            obj.m_lankleAxisShankVarStr = strcat(obj.id,'_lankleAxisShank');
            obj.m_lankleAxisShankVarChar=VarStrToCharMap.insert(obj.m_lankleAxisShankVarStr);
            obj.m_lankleAxisShankKey = gtsam.symbol(obj.m_lankleAxisShankVarChar,0);
        end
        
        function defaultOptimize(obj)
            fastOptimize(obj);
        end
        
        function fastOptimize(obj)
            %%%%%% fastOptimize %%%%%
            if obj.m_optimizerType==0 % Gauss-Newton
                params=gtsam.GaussNewtonParams; params.setVerbosity('DELTA');
                optimizer=gtsam.GaussNewtonOptimizer(obj.m_graph, obj.m_initialValues, params);
            elseif obj.m_optimizerType==1 % Levenberg-Marquardt
                params=gtsam.LevenbergMarquardtParams; verbosity='SUMMARY'; % options are: SUMMARY, TERMINATION, LAMBDA, TRYLAMBDA, TRYCONFIG, DAMPED, TRYDELTA
                params.setlambdaInitial(obj.m_lambdaInitial); params.setlambdaFactor(obj.m_lambdaFactor); params.setlambdaUpperBound(obj.m_lambdaUpperBound); params.setlambdaLowerBound(obj.m_lambdaLowerBound);
                params.setRelativeErrorTol(1.0e-20); params.setAbsoluteErrorTol(1.0e-20); % <-set abs and rel params to an extremely small number since you're manually gonna manage the convergence
                params.setVerbosityLM(verbosity); params.setVerbosity(verbosity);
                optimizer=gtsam.LevenbergMarquardtOptimizer(obj.m_graph, obj.m_initialValues, params);
            else; error('unknown optimization type. choose 0 for Gauss-Newton or 1 for Levenberg-Marquardt');
            end
            currentError=optimizer.error(); nIterations=optimizer.iterations();
            fprintf('***** lowerBodyPoseEstimator::fastOptimize() *****\n');
            printErrorsInGraphByFactorType(obj.m_graph,obj.m_initialValues);
            fprintf('\tinitial error: %.8g\n',currentError);
            optBeginTic=tic;
            absErrorDecrease=9.0e9; relErrorDecrease=9.0e9;
            % also create a small gui to stop this loop execution
            figh_kill=figure('units','normalized','position',[.4 .45 .2 .1]); axes('Visible','off'); text(0,0,'Close this window to kill optimization routine'); drawnow;
            % optimization stats
            obj.m_optimizationTotalError(nIterations+1)=optimizer.error(); obj.m_optimizationTotalTime(nIterations+1)=0; % gotta add 1 b/c nIterations starts at zero
            consecSmallIter=0;
            while optimizer.iterations()<obj.m_maxIterations && consecSmallIter<obj.m_convergeAtConsecSmallIter
                % NOTE: if using SUMMARY verbosity, columns are: [iterationNum, postIterError, negCostChange, lambda, success?, iterTime]
                % set previous iteration's info
                previousError=currentError;
                % iterate
                iterationStart=tic;
                optimizer.iterate(); % : returns gtsam::GaussianFactorGraph
                %                 if optimizer.iterations==nIterations % did not iterate
                %                     fprintf('did not iterate, exiting.\n');
                %                     break; % but why?
                %                 end
                nIterations=optimizer.iterations;
                iterationTime=toc(iterationStart);
                % get current info
                currentError=optimizer.error();
                % compute change in errors
                absErrorDecrease=previousError-currentError; relErrorDecrease=absErrorDecrease/previousError;
                if absErrorDecrease<obj.m_absErrDecreaseLimit || relErrorDecrease<obj.m_relErrDecreaseLimit
                    consecSmallIter=consecSmallIter+1;
                else
                    consecSmallIter=0;
                end
                obj.m_optimizationTotalError(nIterations+1)=optimizer.error(); obj.m_optimizationTotalTime(nIterations+1)=toc(optBeginTic); % gotta add 1 b/c nIterations starts at zero
                % print?
                fprintf('  --> end iteration %d (%.4f sec): error=%.6g (decrease= %.4e || %.4f %%) | converged iteration (%d/%d)\n',optimizer.iterations()-1,iterationTime,currentError,absErrorDecrease,relErrorDecrease*100,consecSmallIter,obj.m_convergeAtConsecSmallIter);
                if ~ishandle(figh_kill)
                    fprintf('optimization killed by user.\n');
                    break;
                end
                pause(1.0e-10);
            end
            if ishandle(figh_kill); close(figh_kill); end
            fprintf('---- complete! total time=%.4f sec, %d iterations, final error=%.8g (avg. %.8g per keyframe) ----\n',toc(optBeginTic),optimizer.iterations(),optimizer.error(),optimizer.error()/getNumKeyframes(obj));
            % print convergence condition
            fprintf('Convergence condition:\n');
            if relErrorDecrease<obj.m_relErrDecreaseLimit
                fprintf('    CONVERGED: rel. error decrease < limit (%.6f %% < %.6f %%)\n',relErrorDecrease*100,obj.m_relErrDecreaseLimit*100);
            elseif absErrorDecrease<obj.m_absErrDecreaseLimit
                fprintf('    CONVERGED: abs. error decrease < limit (%.6f < %.6f)\n',absErrorDecrease,obj.m_absErrDecreaseLimit);
            elseif optimizer.iterations()>=(obj.m_maxIterations-1)
                fprintf('    exiting. iteration maximum reached (%d)\n',obj.m_maxIterations);
            else
                fprintf('    no convergence criteria met.\n');
            end
            if optimizer.error()/getNumKeyframes(obj)>10
                warning('this average error per keyframe (%.5f) is higher than typically desired (%d or less). Solution may not be good.',optimizer.error()/getNumKeyframes(obj),10);
            end
            fprintf('*********************************************\n');
            obj.m_estimate=optimizer.values();
            %             jacobianInspector(obj.m_graph,obj.m_estimate);
            printErrorsInGraphByFactorType(obj.m_graph,obj.m_estimate);
            obj.setMemberStatesFromValues(obj.m_estimate);
            postHocAxesCheck(obj,1);
            if obj.m_setMarginals % pull out marginal covariances and principal variances for saving
                setMarginals(obj);
            end
            setDerivedJointAnglesFromEstimatedStates(obj);
            printStaticStatesSummary(obj);
            checkAnkleRelativeAngularVelocityAxesByPca(obj);
            %             setAllInternalPrecessionAngles(obj);
        end % fastOptimize()
        
        function robustOptimize(obj)
            % right now it's just fastOptimize + printing of static parameters at every iteration
            %%%%%% robustOptimize %%%%%
            if obj.m_optimizerType==0 % Gauss-Newton
                params=gtsam.GaussNewtonParams; params.setVerbosity('DELTA');
                optimizer=gtsam.GaussNewtonOptimizer(obj.m_graph, obj.m_initialValues, params);
            elseif obj.m_optimizerType==1 % Levenberg-Marquardt
                params=gtsam.LevenbergMarquardtParams; verbosity='SUMMARY'; % options are: SUMMARY, TERMINATION, LAMBDA, TRYLAMBDA, TRYCONFIG, DAMPED, TRYDELTA
                params.setlambdaInitial(obj.m_lambdaInitial); params.setlambdaFactor(obj.m_lambdaFactor); params.setlambdaUpperBound(obj.m_lambdaUpperBound); params.setlambdaLowerBound(obj.m_lambdaLowerBound);
                params.setRelativeErrorTol(1.0e-20); params.setAbsoluteErrorTol(1.0e-20); % <-set abs and rel params to an extremely small number since you're manually gonna manage the convergence
                params.setVerbosityLM(verbosity); params.setVerbosity(verbosity);
                optimizer=gtsam.LevenbergMarquardtOptimizer(obj.m_graph, obj.m_initialValues, params);
            else; error('unknown optimization type. choose 0 for Gauss-Newton or 1 for Levenberg-Marquardt');
            end
            currentError=optimizer.error(); nIterations=optimizer.iterations();
            fprintf('***** lowerBodyPoseEstimator::robustOptimize() *****\n');
            printErrorsInGraphByFactorType(obj.m_graph,obj.m_initialValues);
            fprintf('\tinitial error: %.8g\n',currentError);
            optBeginTic=tic;
            absErrorDecrease=9.0e9; relErrorDecrease=9.0e9;
            % also create a small gui to stop this loop execution
            figh_kill=figure('units','normalized','position',[.4 .45 .2 .1]); axes('Visible','off'); text(0,0,'Close this window to kill optimization routine'); drawnow;
            % optimization stats
            obj.m_optimizationTotalError(nIterations+1)=optimizer.error(); obj.m_optimizationTotalTime(nIterations+1)=0; % gotta add 1 b/c nIterations starts at zero
            consecSmallIter=0;
            while optimizer.iterations()<obj.m_maxIterations && consecSmallIter<obj.m_convergeAtConsecSmallIter
                % NOTE: if using SUMMARY verbosity, columns are: [iterationNum, postIterError, negCostChange, lambda, success?, iterTime]
                % set previous iteration's info
                previousError=currentError;
                % iterate
                iterationStart=tic;
                optimizer.iterate(); % : returns gtsam::GaussianFactorGraph
                %                 if optimizer.iterations==nIterations % did not iterate
                %                     fprintf('did not iterate, exiting.\n');
                %                     break; % but why?
                %                 end
                nIterations=optimizer.iterations;
                iterationTime=toc(iterationStart);
                % get current info
                currentError=optimizer.error();
                % compute change in errors
                absErrorDecrease=previousError-currentError; relErrorDecrease=absErrorDecrease/previousError;
                if absErrorDecrease<obj.m_absErrDecreaseLimit || relErrorDecrease<obj.m_relErrDecreaseLimit
                    consecSmallIter=consecSmallIter+1;
                else
                    consecSmallIter=0;
                end
                obj.m_optimizationTotalError(nIterations+1)=optimizer.error(); obj.m_optimizationTotalTime(nIterations+1)=toc(optBeginTic); % gotta add 1 b/c nIterations starts at zero
                % pull out static parameters
                currentVals=optimizer.values();
                sacrumImuToRHipCtr=currentVals.atPoint3(obj.m_sacrumImuToRHipCtrKey); rthighImuToHipCtr=currentVals.atPoint3(obj.m_rthighImuToHipCtrKey); rthighImuToKneeCtr=currentVals.atPoint3(obj.m_rthighImuToKneeCtrKey); rshankImuToKneeCtr=currentVals.atPoint3(obj.m_rshankImuToKneeCtrKey); rshankImuToAnkleCtr=currentVals.atPoint3(obj.m_rshankImuToAnkleCtrKey); rfootImuToAnkleCtr=currentVals.atPoint3(obj.m_rfootImuToAnkleCtrKey);
                sacrumImuToLHipCtr=currentVals.atPoint3(obj.m_sacrumImuToLHipCtrKey); lthighImuToHipCtr=currentVals.atPoint3(obj.m_lthighImuToHipCtrKey); lthighImuToKneeCtr=currentVals.atPoint3(obj.m_lthighImuToKneeCtrKey); lshankImuToKneeCtr=currentVals.atPoint3(obj.m_lshankImuToKneeCtrKey); lshankImuToAnkleCtr=currentVals.atPoint3(obj.m_lshankImuToAnkleCtrKey); lfootImuToAnkleCtr=currentVals.atPoint3(obj.m_lfootImuToAnkleCtrKey);
                hipAxisSacrum=currentVals.atUnit3(obj.m_hipAxisSacrumKey); rkneeAxisThigh=currentVals.atUnit3(obj.m_rkneeAxisThighKey); rkneeAxisShank=currentVals.atUnit3(obj.m_rkneeAxisShankKey); rankleAxisFoot=currentVals.atUnit3(obj.m_rankleAxisFootKey);
                lkneeAxisThigh=currentVals.atUnit3(obj.m_lkneeAxisThighKey); lkneeAxisShank=currentVals.atUnit3(obj.m_lkneeAxisShankKey); lankleAxisFoot=currentVals.atUnit3(obj.m_lankleAxisFootKey);
                % print
                fprintf('  sacrumImuToRHipCtr=[%.4f %.4f %.4f], rthighImuToHipCtr=[%.4f %.4f %.4f], rthighImuToKneeCtr=[%.4f %.4f %.4f], rshankImuToKneeCtr=[%.4f %.4f %.4f], rshankImuToAnkleCtr=[%.4f %.4f %.4f], rfootImuToAnkleCtr=[%.4f %.4f %.4f]\n',sacrumImuToRHipCtr.vector(),rthighImuToHipCtr.vector(),rthighImuToKneeCtr.vector(),rshankImuToKneeCtr.vector(),rshankImuToAnkleCtr.vector(),rfootImuToAnkleCtr.vector());
                fprintf('  sacrumImuToLHipCtr=[%.4f %.4f %.4f], lthighImuToHipCtr=[%.4f %.4f %.4f], lthighImuToKneeCtr=[%.4f %.4f %.4f], lshankImuToKneeCtr=[%.4f %.4f %.4f], lshankImuToAnkleCtr=[%.4f %.4f %.4f], lfootImuToAnkleCtr=[%.4f %.4f %.4f]\n',sacrumImuToLHipCtr.vector(),lthighImuToHipCtr.vector(),lthighImuToKneeCtr.vector(),lshankImuToKneeCtr.vector(),lshankImuToAnkleCtr.vector(),lfootImuToAnkleCtr.vector());
                fprintf('  hipAxisSacrum=[%.3f %.3f %.3f], rkneeAxisThigh=[%.3f %.3f %.3f], rkneeAxisShank=[%.3f %.3f %.3f], rankleAxisFoot=[%.3f %.3f %.3f]\n',hipAxisSacrum.point3().vector(),rkneeAxisThigh.point3().vector(),rkneeAxisShank.point3().vector(),rankleAxisFoot.point3().vector());
                fprintf('                                      lkneeAxisThigh=[%.3f %.3f %.3f], lkneeAxisShank=[%.3f %.3f %.3f], lankleAxisFoot=[%.3f %.3f %.3f]\n',lkneeAxisThigh.point3().vector(),lkneeAxisShank.point3().vector(),lankleAxisFoot.point3().vector());
                fprintf('  --> end iteration %d (%.4f sec): error=%.6g (decrease= %.4e || %.4f %%) | converged iteration (%d/%d)\n',optimizer.iterations()-1,iterationTime,currentError,absErrorDecrease,relErrorDecrease*100,consecSmallIter,obj.m_convergeAtConsecSmallIter);
                if ~ishandle(figh_kill)
                    fprintf('optimization killed by user.\n');
                    break;
                end
                pause(1.0e-10);
            end
            if ishandle(figh_kill); close(figh_kill); end
            fprintf('---- complete! total time=%.4f sec, %d iterations, final error=%.8g (avg. %.8g per keyframe) ----\n',toc(optBeginTic),optimizer.iterations(),optimizer.error(),optimizer.error()/getNumKeyframes(obj));
            % print convergence condition
            fprintf('Convergence condition:\n');
            if relErrorDecrease<obj.m_relErrDecreaseLimit
                fprintf('    CONVERGED: rel. error decrease < limit (%.6f %% < %.6f %%)\n',relErrorDecrease*100,obj.m_relErrDecreaseLimit*100);
            elseif absErrorDecrease<obj.m_absErrDecreaseLimit
                fprintf('    CONVERGED: abs. error decrease < limit (%.6f < %.6f)\n',absErrorDecrease,obj.m_absErrDecreaseLimit);
            elseif optimizer.iterations()>=(obj.m_maxIterations-1)
                fprintf('    exiting. iteration maximum reached (%d)\n',obj.m_maxIterations);
            else
                fprintf('    no convergence criteria met.\n');
            end
            if optimizer.error()/getNumKeyframes(obj)>10
                warning('this average error per keyframe (%.5f) is higher than typically desired (%d or less). Solution may not be good.',optimizer.error()/getNumKeyframes(obj),10);
            end
            fprintf('*********************************************\n');
            obj.m_estimate=optimizer.values();
            %             jacobianInspector(obj.m_graph,obj.m_estimate);
            printErrorsInGraphByFactorType(obj.m_graph,obj.m_estimate);
            obj.setMemberStatesFromValues(obj.m_estimate);
            postHocAxesCheck(obj,1);
            if obj.m_setMarginals % pull out marginal covariances and principal variances for saving
                setMarginals(obj);
            end
            setDerivedJointAnglesFromEstimatedStates(obj);
            printStaticStatesSummary(obj);
            checkAnkleRelativeAngularVelocityAxesByPca(obj);
            setAllInternalPrecessionAngles(obj);
        end % robust optimize
        
        function anyChangesMade=postHocAxesCheck(obj,varargin)
            % this method *for both legs*, checks:
            % (1) that both knee axes are pointed in the same direction (b/w thigh and shank frames)
            %   - done by checking average angle between. should be less than 90. if not, flips shank axis.
            % (2) that both axes are pointed to the subject's right (ISB definition).
            %   - done by checking the range of the knee flexion/extension angle. should be between 20 deg and -180 deg.
            % --- settings ----
            if nargin==2 && isa(varargin{1},'double')
                verbose=varargin{1};
            else
                verbose=0; % set to 1 to turn on debug printing
            end
            maxFlexExLimDeg=20; % max limit in degrees to compare to
            minFlexExLimDeg=-180; % min limit in degrees to compare to
            % -----------------
            anyChangesMade=0; % if changes made, this goes to one. it will rerun this method just to make sure no changes are made the second time around.
            % --- right leg first ---
            % () are both axes pointing the same direction in the world frame?
            rThighAxis=obj.m_rkneeAxisThigh.point3().vector(); rShankAxis=obj.m_rkneeAxisShank.point3().vector();
            if verbose; fprintf('---postHocAxesCheck()---\n'); fprintf('\t***checking right leg first***\n');
                fprintf('\trknee axis, thigh frame: [%.4f %.4f %.4f] (norm=%.4f)\n',obj.m_rkneeAxisThigh.point3().vector(),sqrt(sum(obj.m_rkneeAxisThigh.point3().vector().^2)));
                fprintf('\trknee axis, shank frame: [%.4f %.4f %.4f] (norm=%.4f)\n',obj.m_rkneeAxisShank.point3().vector(),sqrt(sum(obj.m_rkneeAxisShank.point3().vector().^2)));
            end
            % get axes in world frame
            rThighAxisNav=zeros(length(obj.m_rthighImuOrientation),3); rShankAxisNav=zeros(length(obj.m_rthighImuOrientation),3);
            for k=1:length(obj.m_rthighImuOrientation)
                rThighAxisNav(k,1:3)=[obj.m_rthighImuOrientation(k).matrix()*rThighAxis]';
                rShankAxisNav(k,1:3)=[obj.m_rshankImuOrientation(k).matrix()*rShankAxis]';
            end
            angBwRightAxesNav=angle_between_vectors_deg(rThighAxisNav,rShankAxisNav);
            if mean(angBwRightAxesNav) <= 90 % same side, stay
                if verbose; fprintf('\tmean of angle b/w: %.2f deg, therefore I think they are pointing in the same direction.\n',mean(angBwRightAxesNav)); end
            else
                obj.m_rkneeAxisShank=gtsam.Unit3(gtsam.Point3(-1.0*obj.m_rkneeAxisShank.point3().vector())); anyChangesMade=1;
                if verbose; fprintf('\tmean of angle b/w: %.2f deg, therefore I think they are pointing in opposite directions. So I flipped the sign of the right knee axis in the shank frame, which is now: [%.4f %.4f %.4f]\n',mean(angBwRightAxesNav),obj.m_rkneeAxisShank.point3().vector()); end
            end
            % () now from the flex/ex knee angle determine if they are both pointed to the subject's right
            % determine this from the 10th and 90th percentile quantiles for outlier rejection
            [rflexionExtensionAngleFirstPass,~,~,~,~]=bruteForceRKneeAngleCalc(obj);
            rKneeFlexExStats=quantile(rad2deg(rflexionExtensionAngleFirstPass),[.1 .9]);
            if rKneeFlexExStats(1)>minFlexExLimDeg && rKneeFlexExStats(2)<maxFlexExLimDeg % this is what you want
                if verbose; fprintf('\tI believe that the knee axes are pointed to the right because the relevant quantiles [%.2f %.2f] are within the acceptable limits [%.1f %.1f]\n',rKneeFlexExStats,minFlexExLimDeg,maxFlexExLimDeg); end
            elseif rKneeFlexExStats(1)>-1.0*maxFlexExLimDeg && rKneeFlexExStats(2)<-1.0*minFlexExLimDeg % classical case where they both point left
                obj.m_rkneeAxisThigh=gtsam.Unit3(gtsam.Point3(-1.0*obj.m_rkneeAxisThigh.point3().vector()));
                obj.m_rkneeAxisShank=gtsam.Unit3(gtsam.Point3(-1.0*obj.m_rkneeAxisShank.point3().vector())); anyChangesMade=1;
                if verbose
                    fprintf('\tI believe that the knee axes are pointed to the left because the relevant quantiles [%.2f %.2f] are within the negative of the acceptable limits [%.1f %.1f]. Therefore I flipped both axes, which are now:\n',rKneeFlexExStats,minFlexExLimDeg,maxFlexExLimDeg);
                    fprintf('\t\trknee axis, thigh frame: [%.4f %.4f %.4f] (norm=%.4f)\n',obj.m_rkneeAxisThigh.point3().vector(),sqrt(sum(obj.m_rkneeAxisThigh.point3().vector().^2)));
                    fprintf('\t\trknee axis, shank frame: [%.4f %.4f %.4f] (norm=%.4f)\n',obj.m_rkneeAxisShank.point3().vector(),sqrt(sum(obj.m_rkneeAxisShank.point3().vector().^2)));
                end
            else; fprintf('WARNING: When comparing right knee flex/ex angle metric quantiles [%.2f %.2f] to acceptable bounds [%.1f %.1f], I can''t figure out which way the axes are pointing. You may have bad estimation.\n',rKneeFlexExStats,minFlexExLimDeg,maxFlexExLimDeg);
            end
            % --- now do the same for the left leg ---
            % () are both axes pointing the same direction in the world frame?
            lThighAxis=obj.m_lkneeAxisThigh.point3().vector(); lShankAxis=obj.m_lkneeAxisShank.point3().vector();
            if verbose; fprintf('\t***now checking left leg***\n');
                fprintf('\tlknee axis, thigh frame: [%.4f %.4f %.4f] (norm=%.4f)\n',obj.m_lkneeAxisThigh.point3().vector(),sqrt(sum(obj.m_lkneeAxisThigh.point3().vector().^2)));
                fprintf('\tlknee axis, shank frame: [%.4f %.4f %.4f] (norm=%.4f)\n',obj.m_lkneeAxisShank.point3().vector(),sqrt(sum(obj.m_lkneeAxisShank.point3().vector().^2)));
            end
            % get axes in world frame
            lThighAxisNav=zeros(length(obj.m_lthighImuOrientation),3); lShankAxisNav=zeros(length(obj.m_lthighImuOrientation),3);
            for k=1:length(obj.m_lthighImuOrientation)
                lThighAxisNav(k,1:3)=[obj.m_lthighImuOrientation(k).matrix()*lThighAxis]';
                lShankAxisNav(k,1:3)=[obj.m_lshankImuOrientation(k).matrix()*lShankAxis]';
            end
            angBwLeftAxesNav=angle_between_vectors_deg(lThighAxisNav,lShankAxisNav);
            if mean(angBwLeftAxesNav) <= 90 % same side, stay
                if verbose; fprintf('\tmean of angle b/w: %.2f deg, therefore I think they are pointing in the same direction.\n',mean(angBwLeftAxesNav)); end
            else
                obj.m_lkneeAxisShank=gtsam.Unit3(gtsam.Point3(-1.0*obj.m_lkneeAxisShank.point3().vector())); anyChangesMade=1;
                if verbose; fprintf('\tmean of angle b/w: %.2f deg, therefore I think they are pointing in opposite directions. So I flipped the sign of the left knee axis in the shank frame, which is now: [%.4f %.4f %.4f]\n',mean(angBwLeftAxesNav),obj.m_lkneeAxisShank.point3().vector()); end
            end
            % () now from the flex/ex knee angle determine if they are both pointed to the subject's right
            % determine this from the 10th and 90th percentile quantiles for outlier rejection
            [lflexionExtensionAngleFirstPass,~,~,~,~]=bruteForceLKneeAngleCalc(obj);
            lKneeFlexExStats=quantile(rad2deg(lflexionExtensionAngleFirstPass),[.1 .9]);
            if lKneeFlexExStats(1)>minFlexExLimDeg && lKneeFlexExStats(2)<maxFlexExLimDeg % this is what you want
                if verbose; fprintf('\tI believe that the knee axes are pointed to the right because the relevant quantiles [%.2f %.2f] are within the acceptable limits [%.1f %.1f]\n',lKneeFlexExStats,minFlexExLimDeg,maxFlexExLimDeg); end
            elseif lKneeFlexExStats(1)>-1.0*maxFlexExLimDeg && lKneeFlexExStats(2)<-1.0*minFlexExLimDeg % classical case where they both point left
                obj.m_lkneeAxisThigh=gtsam.Unit3(gtsam.Point3(-1.0*obj.m_lkneeAxisThigh.point3().vector()));
                obj.m_lkneeAxisShank=gtsam.Unit3(gtsam.Point3(-1.0*obj.m_lkneeAxisShank.point3().vector())); anyChangesMade=1;
                if verbose
                    fprintf('\tI believe that the knee axes are pointed to the left because the relevant quantiles [%.2f %.2f] are within the negative of the acceptable limits [%.1f %.1f]. Therefore I flipped both axes, which are now:\n',lKneeFlexExStats,minFlexExLimDeg,maxFlexExLimDeg);
                    fprintf('\t\tlknee axis, thigh frame: [%.4f %.4f %.4f] (norm=%.4f)\n',obj.m_lkneeAxisThigh.point3().vector(),sqrt(sum(obj.m_lkneeAxisThigh.point3().vector().^2)));
                    fprintf('\t\tlknee axis, shank frame: [%.4f %.4f %.4f] (norm=%.4f)\n',obj.m_lkneeAxisShank.point3().vector(),sqrt(sum(obj.m_lkneeAxisShank.point3().vector().^2)));
                end
            else; fprintf('WARNING: When comparing left knee flex/ex angle metric quantiles [%.2f %.2f] to acceptable bounds [%.1f %.1f], I can''t figure out which way the axes are pointing. You may have bad estimation.\n',lKneeFlexExStats,minFlexExLimDeg,maxFlexExLimDeg);
            end
            % () okay now you've done both legs. finally, rerun this one more time and make sure that no changes are made the second time.
            if anyChangesMade % if any changes made, rerun a second time
                anyChangesMade2ndPass=postHocAxesCheck(obj,0); % silently run
                if anyChangesMade2ndPass==0 % good, what you want
                    if verbose; fprintf('\t***No changes made on the second pass of this method (as expected!). Exiting.***\n'); end
                else; error('Why did something changes on the second pass?');
                end
            end
            if verbose; fprintf('-----------------------\n'); end
        end
        
        function printStaticStatesSummary(obj)
            % a method to print a summary of results for: all static parameter values, joint alignment errors, geometry of IMU-limb segment triangles
            printVectorOptionalVariance('rknee axis, thigh frame',obj.m_rkneeAxisThigh.point3().vector(),diag(obj.m_rkneeAxisThighCovariance));
            printVectorOptionalVariance('rknee axis, shank frame',obj.m_rkneeAxisShank.point3().vector(),diag(obj.m_rkneeAxisShankCovariance));
            printVectorOptionalVariance('lknee axis, thigh frame',obj.m_lkneeAxisThigh.point3().vector(),diag(obj.m_lkneeAxisThighCovariance));
            printVectorOptionalVariance('lknee axis, shank frame',obj.m_lkneeAxisShank.point3().vector(),diag(obj.m_lkneeAxisShankCovariance));
            if obj.m_assumeHipHinge
                printVectorOptionalVariance('both hip axis, sacrum frame',obj.m_hipAxisSacrum.point3().vector(),[]);
            end
            if obj.m_assumeAnkleHinge
                printVectorOptionalVariance('rankle axis, foot frame',obj.m_rankleAxisFoot.point3().vector(),[]);
                printVectorOptionalVariance('lankle axis, foot frame',obj.m_lankleAxisFoot.point3().vector(),[]);
            end
            printVectorOptionalVariance('sacrum IMU to rhip ctr',obj.m_sacrumImuToRHipCtr.vector(),diag(obj.m_sacrumImuToRHipCtrCovariance));
            printVectorOptionalVariance('sacrum IMU to lhip ctr',obj.m_sacrumImuToLHipCtr.vector(),diag(obj.m_sacrumImuToLHipCtrCovariance));
            printVectorOptionalVariance('rthigh IMU to rhip ctr',obj.m_rthighImuToHipCtr.vector(),diag(obj.m_rthighImuToHipCtrCovariance));
            printVectorOptionalVariance('rthigh IMU to rknee ctr',obj.m_rthighImuToKneeCtr.vector(),diag(obj.m_rthighImuToKneeCtrCovariance));
            printVectorOptionalVariance('rshank IMU to rknee ctr',obj.m_rshankImuToKneeCtr.vector(),diag(obj.m_rshankImuToKneeCtrCovariance));
            printVectorOptionalVariance('rshank IMU to rankle ctr',obj.m_rshankImuToAnkleCtr.vector(),diag(obj.m_rshankImuToAnkleCtrCovariance));
            printVectorOptionalVariance('rfoot IMU to rankle ctr',obj.m_rfootImuToAnkleCtr.vector(),diag(obj.m_rfootImuToAnkleCtrCovariance));
            printVectorOptionalVariance('lthigh IMU to lhip ctr',obj.m_lthighImuToHipCtr.vector(),diag(obj.m_lthighImuToHipCtrCovariance));
            printVectorOptionalVariance('lthigh IMU to lknee ctr',obj.m_lthighImuToKneeCtr.vector(),diag(obj.m_lthighImuToKneeCtrCovariance));
            printVectorOptionalVariance('lshank IMU to lknee ctr',obj.m_lshankImuToKneeCtr.vector(),diag(obj.m_lshankImuToKneeCtrCovariance));
            printVectorOptionalVariance('lshank IMU to lankle ctr',obj.m_lshankImuToAnkleCtr.vector(),diag(obj.m_lshankImuToAnkleCtrCovariance));
            printVectorOptionalVariance('lfoot IMU to lankle ctr',obj.m_lfootImuToAnkleCtr.vector(),diag(obj.m_lfootImuToAnkleCtrCovariance));
            % print the angle between knee axis and the segment vector
            fprintf('angle between knee axis and the segment proximal vector:\n');
            fprintf('\tright thigh imu: %.3f\n',angle_between_vectors_deg(obj.m_rkneeAxisThigh.point3().vector()',obj.m_rthighImuToHipCtr.vector()'-obj.m_rthighImuToKneeCtr.vector()'));
            fprintf('\tright shank imu: %.3f\n',angle_between_vectors_deg(obj.m_rkneeAxisShank.point3().vector()',obj.m_rshankImuToKneeCtr.vector()'-obj.m_rshankImuToAnkleCtr.vector()'));
            fprintf('\tleft thigh imu: %.3f\n',angle_between_vectors_deg(obj.m_lkneeAxisThigh.point3().vector()',obj.m_lthighImuToHipCtr.vector()'-obj.m_lthighImuToKneeCtr.vector()'));
            fprintf('\tleft shank imu: %.3f\n',angle_between_vectors_deg(obj.m_lkneeAxisShank.point3().vector()',obj.m_lshankImuToKneeCtr.vector()'-obj.m_lshankImuToAnkleCtr.vector()'));
            % joint alignment errors
            rhipAlignmentError=legPoseEstimator.jointAlignmentError(obj.m_sacrumImuOrientation,obj.m_sacrumImuPosition,obj.m_sacrumImuToRHipCtr,obj.m_rthighImuOrientation,obj.m_rthighImuPosition,obj.m_rthighImuToHipCtr);
            rkneeAlignmentError=legPoseEstimator.jointAlignmentError(obj.m_rthighImuOrientation,obj.m_rthighImuPosition,obj.m_rthighImuToKneeCtr,obj.m_rshankImuOrientation,obj.m_rshankImuPosition,obj.m_rshankImuToKneeCtr);
            rankleAlignmentError=legPoseEstimator.jointAlignmentError(obj.m_rshankImuOrientation,obj.m_rshankImuPosition,obj.m_rshankImuToAnkleCtr,obj.m_rfootImuOrientation,obj.m_rfootImuPosition,obj.m_rfootImuToAnkleCtr);
            lhipAlignmentError=legPoseEstimator.jointAlignmentError(obj.m_sacrumImuOrientation,obj.m_sacrumImuPosition,obj.m_sacrumImuToLHipCtr,obj.m_lthighImuOrientation,obj.m_lthighImuPosition,obj.m_lthighImuToHipCtr);
            lkneeAlignmentError=legPoseEstimator.jointAlignmentError(obj.m_lthighImuOrientation,obj.m_lthighImuPosition,obj.m_lthighImuToKneeCtr,obj.m_lshankImuOrientation,obj.m_lshankImuPosition,obj.m_lshankImuToKneeCtr);
            lankleAlignmentError=legPoseEstimator.jointAlignmentError(obj.m_lshankImuOrientation,obj.m_lshankImuPosition,obj.m_lshankImuToAnkleCtr,obj.m_lfootImuOrientation,obj.m_lfootImuPosition,obj.m_lfootImuToAnkleCtr);
            % geometry of each IMU's triangle
            estRThighLength=sqrt(sum([obj.m_rthighImuToHipCtr.vector()-obj.m_rthighImuToKneeCtr.vector()].^2,1));
            estRShankLength=sqrt(sum([obj.m_rshankImuToKneeCtr.vector()-obj.m_rshankImuToAnkleCtr.vector()].^2,1));
            estLThighLength=sqrt(sum([obj.m_lthighImuToHipCtr.vector()-obj.m_lthighImuToKneeCtr.vector()].^2,1));
            estLShankLength=sqrt(sum([obj.m_lshankImuToKneeCtr.vector()-obj.m_lshankImuToAnkleCtr.vector()].^2,1));
            estPelvicWidth=sqrt(sum([obj.m_sacrumImuToRHipCtr.vector()-obj.m_sacrumImuToLHipCtr.vector()].^2,1));
            fprintf('rhip alignment error: mean=[%.4f %.4f %.4f], RMSE=[%.4f %.4f %.4f], avg norm=%.4f\n',mean(rhipAlignmentError,1),sqrt(mean(rhipAlignmentError.^2,1)),mean(sqrt(sum(rhipAlignmentError.^2,2))));
            fprintf('rknee alignment error: mean=[%.4f %.4f %.4f], RMSE=[%.4f %.4f %.4f], avg norm=%.4f\n',mean(rkneeAlignmentError,1),sqrt(mean(rkneeAlignmentError.^2,1)),mean(sqrt(sum(rkneeAlignmentError.^2,2))));
            fprintf('rankle alignment error: mean=[%.4f %.4f %.4f], RMSE=[%.4f %.4f %.4f], avg norm=%.4f\n',mean(rankleAlignmentError,1),sqrt(mean(rankleAlignmentError.^2,1)),mean(sqrt(sum(rankleAlignmentError.^2,2))));
            fprintf('lhip alignment error: mean=[%.4f %.4f %.4f], RMSE=[%.4f %.4f %.4f], avg norm=%.4f\n',mean(lhipAlignmentError,1),sqrt(mean(lhipAlignmentError.^2,1)),mean(sqrt(sum(lhipAlignmentError.^2,2))));
            fprintf('lknee alignment error: mean=[%.4f %.4f %.4f], RMSE=[%.4f %.4f %.4f], avg norm=%.4f\n',mean(lkneeAlignmentError,1),sqrt(mean(lkneeAlignmentError.^2,1)),mean(sqrt(sum(lkneeAlignmentError.^2,2))));
            fprintf('lankle alignment error: mean=[%.4f %.4f %.4f], RMSE=[%.4f %.4f %.4f], avg norm=%.4f\n',mean(lankleAlignmentError,1),sqrt(mean(lankleAlignmentError.^2,1)),mean(sqrt(sum(lankleAlignmentError.^2,2))));
            fprintf('geometry of lumbar IMU-pelvic triangle:\n')
            [sacrumImuInteriorAngle,sacrumLHipInteriorAngle,sacrumRHipInteriorAngle,sacrumImuDistToPelvis]=imuToAdjacentJointCtrsTriangularGeometry(obj.m_sacrumImuToLHipCtr.vector(),obj.m_sacrumImuToRHipCtr.vector());
            fprintf('\tsacrum IMU interior angle: %.2f deg\n',sacrumImuInteriorAngle);
            fprintf('\tsacrum IMU - lhip joint interior angle: %.2f deg\n',sacrumLHipInteriorAngle);
            fprintf('\tsacrum IMU - rhip joint interior angle: %.2f deg\n',sacrumRHipInteriorAngle);
            fprintf('\tsacrum IMU dist to femoral head segment: %.4f m\n',sacrumImuDistToPelvis);
            fprintf('\tfemoral head separation = %.4f, anthro mean = %.4f (error=%.4f)\n',estPelvicWidth,obj.m_pelvicWidth,estPelvicWidth-obj.m_pelvicWidth);
            % bonus: for sacrum imu, also compute angle hip centerline makes out of sacrum +z-normal (~coronal) and -x-normal (~transverse) and +y-normal (~saggital) planes
            hipCenterline=(obj.m_sacrumImuToRHipCtr.vector()-obj.m_sacrumImuToLHipCtr.vector())/sqrt(sum([obj.m_sacrumImuToRHipCtr.vector()-obj.m_sacrumImuToLHipCtr.vector()].^2,1))';
            fprintf('\thip centerline (+right, [%.3f %.3f %.3f]) angles between sacrum...\n',hipCenterline);
            fprintf('\t\t+y: %.3f deg (expected 0 deg)\n',angle_between_vectors_deg([0 1 0],hipCenterline'));
            fprintf('\t\t+z: %.3f deg (expected 90 deg)\n',angle_between_vectors_deg([0 0 1],hipCenterline'));
            fprintf('\t\t+x: %.3f deg (expected 90 deg)\n',angle_between_vectors_deg([1 0 0],hipCenterline'));
            fprintf('geometry of rthigh IMU to hip/knee triangle:\n');
            [rthighImuInteriorAngle,rhipThighInteriorAng,rkneeThighInteriorAng,rthighImuDistanceToFemur]=imuToAdjacentJointCtrsTriangularGeometry(obj.m_rthighImuToHipCtr.vector(),obj.m_rthighImuToKneeCtr.vector());
            fprintf('\trthigh IMU interior angle: %.2f deg\n',rthighImuInteriorAngle);
            fprintf('\trthigh IMU - hip joint interior angle: %.2f deg\n',rhipThighInteriorAng);
            fprintf('\trthigh IMU - knee joint interior angle: %.2f deg\n',rkneeThighInteriorAng);
            fprintf('\trthigh IMU dist to femur: %.4f m\n',rthighImuDistanceToFemur);
            fprintf('\trthigh segment length = %.4f, anthro mean = %.4f (error=%.4f)\n',estRThighLength,obj.m_rfemurLength,estRThighLength-obj.m_rfemurLength);
            fprintf('geometry of rshank IMU to knee/ankle triangle:\n');
            [rshankImuInteriorAngle,rkneeShankInteriorAng,rankleShankInteriorAng,rshankImuDistanceToTibia]=imuToAdjacentJointCtrsTriangularGeometry(obj.m_rshankImuToKneeCtr.vector(),obj.m_rshankImuToAnkleCtr.vector());
            fprintf('\trshank IMU interior angle: %.2f deg\n',rshankImuInteriorAngle);
            fprintf('\trshank IMU - knee joint interior angle: %.2f deg\n',rkneeShankInteriorAng);
            fprintf('\trshank IMU - ankle joint interior angle: %.2f deg\n',rankleShankInteriorAng);
            fprintf('\trshank IMU dist to femur: %.4f m\n',rshankImuDistanceToTibia);
            fprintf('\trshank segment length = %.4f, anthro mean = %.4f (error=%.4f)\n',estRShankLength,obj.m_rtibiaLength,estRShankLength-obj.m_rtibiaLength);
            fprintf('geometry of lthigh IMU to hip/knee triangle:\n');
            [lthighImuInteriorAngle,lhipThighInteriorAng,lkneeThighInteriorAng,lthighImuDistanceToFemur]=imuToAdjacentJointCtrsTriangularGeometry(obj.m_lthighImuToHipCtr.vector(),obj.m_lthighImuToKneeCtr.vector());
            fprintf('\tlthigh IMU interior angle: %.2f deg\n',lthighImuInteriorAngle);
            fprintf('\tlthigh IMU - hip joint interior angle: %.2f deg\n',lhipThighInteriorAng);
            fprintf('\tlthigh IMU - knee joint interior angle: %.2f deg\n',lkneeThighInteriorAng);
            fprintf('\tlthigh IMU dist to femur: %.4f m\n',lthighImuDistanceToFemur);
            fprintf('\tlthigh segment length = %.4f, anthro mean = %.4f (error=%.4f)\n',estLThighLength,obj.m_lfemurLength,estLThighLength-obj.m_lfemurLength);
            fprintf('geometry of lshank IMU to knee/ankle triangle:\n');
            [lshankImuInteriorAngle,lkneeShankInteriorAng,lankleShankInteriorAng,lshankImuDistanceToTibia]=imuToAdjacentJointCtrsTriangularGeometry(obj.m_lshankImuToKneeCtr.vector(),obj.m_lshankImuToAnkleCtr.vector());
            fprintf('\tlshank IMU interior angle: %.2f deg\n',lshankImuInteriorAngle);
            fprintf('\tlshank IMU - knee joint interior angle: %.2f deg\n',lkneeShankInteriorAng);
            fprintf('\tlshank IMU - ankle joint interior angle: %.2f deg\n',lankleShankInteriorAng);
            fprintf('\tlshank IMU dist to femur: %.4f m\n',lshankImuDistanceToTibia);
            fprintf('\tlshank segment length = %.4f, anthro mean = %.4f (error=%.4f)\n',estLShankLength,obj.m_ltibiaLength,estLShankLength-obj.m_ltibiaLength);
            if obj.m_sternumImuAdded
                fprintf('sacrum imu to sternum: [%.4f %.4f %.4f] (norm=%.4f)\n',obj.m_sacrumImuToSternum.vector(),sqrt(sum(obj.m_sacrumImuToSternum.vector().^2)));
                fprintf('sternum imu to sacrum: [%.4f %.4f %.4f] (norm=%.4f)\n',obj.m_sternumImuToSacrum.vector(),sqrt(sum(obj.m_sternumImuToSacrum.vector().^2)));
            end
            % leg length discrepency calculation
            fprintf('leg length discrepency:\n');
            fprintf('\tright leg length = %.6f m (RFemur=%.4f + RShank=%.4f)\n',estRThighLength+estRShankLength,estRThighLength,estRShankLength);
            fprintf('\tleft leg length = %.6f m (LFemur=%.4f + LShank=%.4f)\n',estLThighLength+estLShankLength,estLThighLength,estLShankLength);
            fprintf('\t\ttotal LLD = %.3f mm\n',abs((estRThighLength+estRShankLength)-(estLThighLength+estLShankLength))*1000);
            fprintf('\t\tthigh LD = %.3f mm\n',abs(estRThighLength-estLThighLength)*1000);
            fprintf('\t\tshank LD = %.3f mm\n',abs(estRShankLength-estLShankLength)*1000);
            % debugging of axes against other methods
            printKneeAxisFromOtherMethods(obj);
            if obj.m_assumeHipHinge
                printHipAxisFromOtherMethods(obj);
            end
        end
        
        function checkAnkleRelativeAngularVelocityAxesByPca(obj)
            % a debugging method that checks the relative angular velocity of the ankle joints
            if isempty(obj.m_rankleAxisFoot)
                fprintf('did not estimate rankleAxisFoot, exiting.\n');
                return
            end
            fprintf('--- debugging analysis of the right ankle rel. angular velocity ---\n');
            [~,~,~,omegaRelBmA_B]=hingeAxesAccordingToMcGrath2018(obj.m_rshankImuOrientation,obj.m_rfootImuOrientation,obj.m_rshankImuAngVel,obj.m_rfootImuAngVel);
            hingeAxesB=pca(omegaRelBmA_B,'NumComponents',2)';
            primaryPcaAxis=hingeAxesB(1,:);
            secondaryPcaAxis=hingeAxesB(2,:);
            printVectorOptionalVariance('primary axis from PCA',primaryPcaAxis,[]);
            printVectorOptionalVariance('est. rankle axis foot frame',obj.m_rankleAxisFoot.point3().vector()',[]);
            fprintf('\tangle b/w est axis and primary PCA axis: %.2f deg\n',angle_between_vectors_deg_eitherDir(primaryPcaAxis,obj.m_rankleAxisFoot.point3().vector()'));
            printVectorOptionalVariance('secondary axis from PCA',secondaryPcaAxis,[]);
            fprintf('\tangle b/w est axis and secondary PCA axis: %.2f deg\n',angle_between_vectors_deg_eitherDir(secondaryPcaAxis,obj.m_rankleAxisFoot.point3().vector()'));
            % () now, remove all primary components from the rel. angular velocity set, and then redo PCA. What's this new axis now? Is it the same as the secondary axis above?
            fprintf('--- debugging analysis of the left ankle rel. angular velocity ---\n');
            [~,~,~,omegaRelBmA_B]=hingeAxesAccordingToMcGrath2018(obj.m_lshankImuOrientation,obj.m_lfootImuOrientation,obj.m_lshankImuAngVel,obj.m_lfootImuAngVel);
            hingeAxesB=pca(omegaRelBmA_B,'NumComponents',2)';
            primaryPcaAxis=hingeAxesB(1,:);
            secondaryPcaAxis=hingeAxesB(2,:);
            printVectorOptionalVariance('primary axis from PCA',primaryPcaAxis,[]);
            printVectorOptionalVariance('est. lankle axis foot frame',obj.m_lankleAxisFoot.point3().vector()',[]);
            fprintf('\tangle b/w est axis and primary PCA axis: %.2f deg\n',angle_between_vectors_deg_eitherDir(primaryPcaAxis,obj.m_lankleAxisFoot.point3().vector()'));
            printVectorOptionalVariance('secondary axis from PCA',secondaryPcaAxis,[]);
            fprintf('\tangle b/w est axis and secondary PCA axis: %.2f deg\n',angle_between_vectors_deg_eitherDir(secondaryPcaAxis,obj.m_lankleAxisFoot.point3().vector()'));
            % () now, remove all primary components from the rel. angular velocity set, and then redo PCA. What's this new axis now? Is it the same as the secondary axis above?
            fprintf('-------------------------------------------------------------------\n');
        end
        
        function numKeyframes=getNumKeyframes(obj)
            % function returns int number of keyframes. keyframe = each time the IMU states are computed.
            % this should just be equivalent to the number of keyframes in a single IMU pose problem
            %             numKeyframes=floor(length(obj.m_rthighImuPoseProblem.m_imu.ax)/obj.m_rthighImuPoseProblem.m_numMeasToPreint);
            numKeyframes=length(obj.m_sacrumImuPoseProblem.m_poseKeys); % why did this above equation ^ return a different value?
            %    ^ this is: num measurements / number of measurements to preintegrate = number of keyframes for the single IMU problem
            %    and we assume this is the same for all 7 IMUs in the lowerbody model
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % - methods for computing segment orientations and joint angles - %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % --- methods for segment orientation relationships ---
        function R_SacrumImu_to_Pelvis=get_R_SacrumImu_to_Pelvis(obj)
            % get relationship between imu and segment
            Pz=averageVerticalVectorInSacrumImuFrame(obj)'; % segment proximal vector
            R_SacrumImu_to_Pelvis=get_R_SacrumImu_to_Pelvis(Pz, obj.m_sacrumImuToRHipCtr,obj.m_sacrumImuToLHipCtr,true);
        end
        function R_Pelvis_to_N=get_R_Pelvis_to_N(obj)
            % get relationship between segment orientation and navigation frame
            Pz=averageVerticalVectorInSacrumImuFrame(obj)'; % segment proximal vector
            R_Pelvis_to_N = get_R_Pelvis_to_N(obj.m_sacrumImuOrientation, Pz, obj.m_sacrumImuToRHipCtr,obj.m_sacrumImuToLHipCtr, true);
        end
        function R_RThighImu_to_RFemur=get_R_RThighImu_to_RFemur(obj)
            % get relationship between imu and segment
            R_RThighImu_to_RFemur=get_R_Imu_to_Segment(obj.m_rkneeAxisThigh, obj.m_rthighImuToHipCtr, obj.m_rthighImuToKneeCtr,true);
        end
        function R_RFemur_to_N=get_R_RFemur_to_N(obj)
            % get relationship between segment orientation and navigation frame
            R_RFemur_to_N = get_R_Segment_to_N(obj.m_rthighImuOrientation, obj.m_rkneeAxisThigh, obj.m_rthighImuToHipCtr, obj.m_rthighImuToKneeCtr ,true);
        end
        function R_RShankImu_to_RTibia=get_R_RShankImu_to_RTibia(obj)
            % get relationship between imu and segment
            R_RShankImu_to_RTibia=get_R_Imu_to_Segment(obj.m_rkneeAxisShank, obj.m_rshankImuToKneeCtr, obj.m_rshankImuToAnkleCtr,true);
        end
        function R_RTibia_to_N=get_R_RTibia_to_N(obj)
            % get relationship between segment orientation and navigation frame
            R_RTibia_to_N = get_R_Segment_to_N(obj.m_rshankImuOrientation, obj.m_rkneeAxisShank, obj.m_rshankImuToKneeCtr, obj.m_rshankImuToAnkleCtr ,true);
        end
        function R_LThighImu_to_LFemur=get_R_LThighImu_to_LFemur(obj)
            % get relationship between imu and segment
            R_LThighImu_to_LFemur=get_R_Imu_to_Segment(obj.m_lkneeAxisThigh, obj.m_lthighImuToHipCtr, obj.m_lthighImuToKneeCtr,true);
        end
        function R_LFemur_to_N=get_R_LFemur_to_N(obj)
            % get relationship between segment orientation and navigation frame
            R_LFemur_to_N = get_R_Segment_to_N(obj.m_lthighImuOrientation, obj.m_lkneeAxisThigh, obj.m_lthighImuToHipCtr, obj.m_lthighImuToKneeCtr,true);
        end
        function R_LShankImu_to_LTibia=get_R_LShankImu_to_LTibia(obj)
            % get relationship between imu and segment
            R_LShankImu_to_LTibia=get_R_Imu_to_Segment(obj.m_lkneeAxisShank, obj.m_lshankImuToKneeCtr, obj.m_lshankImuToAnkleCtr,true);
        end
        function R_LTibia_to_N=get_R_LTibia_to_N(obj)
            % get relationship between segment orientation and navigation frame
            R_LTibia_to_N = get_R_Segment_to_N(obj.m_lshankImuOrientation, obj.m_lkneeAxisShank, obj.m_lshankImuToKneeCtr, obj.m_lshankImuToAnkleCtr,true);
        end
        function v=averageVerticalVectorInSacrumImuFrame(obj)
            % for finding the hip's angles, we need to construct the pelvic coordinate system. at the moment,
            %   this coordinate system isn't easily observed. We make the assumption that the pelvic vertical vector
            %   is equivalent to the vertical gravity vector average* in the sacrum IMU frame.
            %   this would be a decent assumption if the subject is always upright.
            % *note: this isn't really an "average" on the S(3) manifold. it's ugly and you should fix it later.
            % () get the vertical vector (assumed [0 0 1]) in the sacrum IMU frame (for all t)
            R_SacrumImu_to_N=Rot3ArrayToMatrices(obj.m_sacrumImuOrientation);
            gB=zeros(length(R_SacrumImu_to_N),3);
            for k=1:length(R_SacrumImu_to_N)
                gB(k,:)=[R_SacrumImu_to_N(:,:,k)'*[0 0 1]']';
            end
            % () calculate "average" vector.
            % this is a bad approach. just taking an average of each dimension and then re-normalizing that.
            % ideally you should convert this to a homogenous manifold, like polar coordinates and do your stats there.
            v=median(gB); v=v./sqrt(sum(v.^2,2));
            % to debug, plot with: figure; hold on; plot(gB); plot(1:length(gB),repmat(v,[length(gB) 1]),'--');
        end
        
        % --- methods for joint angle calculation ---
        function [rflexionExtensionAngle,rintExtAng,rAbAdAng,R_Pelvis_to_N,R_RFemur_to_N]=bruteForceRHipAngleCalc(obj)
            % it is preferred to call the knee angle calc first to straighten out the knee axes
            % calculate the hip's angles according to the ISB recommended coordinate system (the "clinical" system)
            % () get pelvis to N
            R_Pelvis_to_N=get_R_Pelvis_to_N(obj);
            % ()----- get RFemur to N
            % NOTE: frustratingly, the ISB is inconsistent with the femur and tibia coordiante frame definitions.
            % so below is the from the knee paper, what we like to use. the pelvis was redefined to be consistent, i.e., pelvis is +z proximal
            % go ahead and set these as the stored values (Unit3)
            % () get R[RFemur->N]
            R_RFemur_to_N=get_R_RFemur_to_N(obj);
            % --------
            % () compute angles as "clinical" system: positive in hip flexion, adduction, internal rotation
            [rflexionExtensionAngle,rAbAdAng,rintExtAng]=clinical3DofJcsAngles(R_Pelvis_to_N, R_RFemur_to_N,'R');
        end
        
        function [lflexionExtensionAngle,lintExtAng,lAbAdAng,R_Pelvis_to_N,R_LFemur_to_N]=bruteForceLHipAngleCalc(obj)
            % it is preferred to call the knee angle calc first to straighten out the knee axes
            % calculate the hip's angles according to the ISB recommended coordinate system
            % () get pelvis to N
            R_Pelvis_to_N=get_R_Pelvis_to_N(obj);
            % ()----- get LFemur to N
            % NOTE: frustratingly, the ISB is inconsistent with the femur and tibia coordiante frame definitions.
            % so below is the from the knee paper, what we like to use. the pelvis was redefined to be consistent.
            % () get R[LFemur->N]
            R_LFemur_to_N=get_R_LFemur_to_N(obj);
            % () compute angles as "clinical" system: positive in hip flexion, adduction, internal rotation
            [lflexionExtensionAngle,lAbAdAng,lintExtAng]=clinical3DofJcsAngles(R_Pelvis_to_N, R_LFemur_to_N,'L');
        end
        
        function [rflexionExtensionAngle,rintExtAng,adAdAngleRad,R_RFemur_to_N,R_RTibia_to_N]=bruteForceRKneeAngleCalc(obj)
            % method to derive the knee's angles from brute force of the estimated parameters of this model
            % () get R[RFemur->N]
            R_RFemur_to_N=get_R_RFemur_to_N(obj);
            % () get R[RTibia->N]
            R_RTibia_to_N=get_R_RTibia_to_N(obj);
            % () compute angles
            [rflexionExtensionAngle,adAdAngleRad,rintExtAng]=clinical3DofJcsAngles(R_RFemur_to_N, R_RTibia_to_N,'R');
        end
        
        function [lflexionExtensionAngle,lintExtAng,ladAdAngleRad,R_LFemur_to_N,R_LTibia_to_N]=bruteForceLKneeAngleCalc(obj)
            % method to derive the knee's angles from brute force of the estimated parameters of this model
            % () pull out R[LFemur->N]
            R_LFemur_to_N=get_R_LFemur_to_N(obj);
            % () pull out R[LTibia->N]
            R_LTibia_to_N=get_R_LTibia_to_N(obj);
            % () compute angles
            [lflexionExtensionAngle,ladAdAngleRad,lintExtAng]=clinical3DofJcsAngles(R_LFemur_to_N, R_LTibia_to_N,'L');
        end
        
        % --------------------------------------------------------------- %
        % ------------------- methods for plotting ---------------------- %
        % --------------------------------------------------------------- %
        function figh=plotEstimatedImuStates(obj)
            % it would be difficult to display seven IMUs at once. so plot seven individual IMU plots.
            % returns an array of 7 fig handles, one for each IMU plot.
            % get time as a variable
            time=obj.m_time;
            % sacrum IMU
            figh(1)=imuPoseEstimator.plotSingleImu(time,obj.m_sacrumImuOrientation,obj.m_sacrumImuVelocity,obj.m_sacrumImuPosition,obj.m_sacrumImuGyroBias,obj.m_sacrumImuAccelBias,...
                obj.m_sacrumImuOrientationPrincipalVariance,obj.m_sacrumImuVelocityPrincipalVariance,obj.m_sacrumImuPositionPrincipalVariance,obj.m_sacrumImuGyroBiasPrincipalVariance,obj.m_sacrumImuAccelBiasPrincipalVariance);
            figh(1).Name=sprintf('%s_sacrumImuResults',obj.id);
            sgtitle(sprintf('Estimated IMU states for LowerBodyPoseEstimator (''%s''): sacrum IMU',obj.id),'Interpreter','None');
            % rthigh IMU
            figh(2)=imuPoseEstimator.plotSingleImu(time,obj.m_rthighImuOrientation,obj.m_rthighImuVelocity,obj.m_rthighImuPosition,obj.m_rthighImuGyroBias,obj.m_rthighImuAccelBias,...
                obj.m_rthighImuOrientationPrincipalVariance,obj.m_rthighImuVelocityPrincipalVariance,obj.m_rthighImuPositionPrincipalVariance,obj.m_rthighImuGyroBiasPrincipalVariance,obj.m_rthighImuAccelBiasPrincipalVariance);
            figh(2).Name=sprintf('%s_rthighImuResults',obj.id);
            sgtitle(sprintf('Estimated IMU states for LowerBodyPoseEstimator (''%s''): rthigh IMU',obj.id),'Interpreter','None');
            % rshank IMU
            figh(3)=imuPoseEstimator.plotSingleImu(time,obj.m_rshankImuOrientation,obj.m_rshankImuVelocity,obj.m_rshankImuPosition,obj.m_rshankImuGyroBias,obj.m_rshankImuAccelBias,...
                obj.m_rshankImuOrientationPrincipalVariance,obj.m_rshankImuVelocityPrincipalVariance,obj.m_rshankImuPositionPrincipalVariance,obj.m_rshankImuGyroBiasPrincipalVariance,obj.m_rshankImuAccelBiasPrincipalVariance);
            figh(3).Name=sprintf('%s_rshankImuResults',obj.id);
            sgtitle(sprintf('Estimated IMU states for LowerBodyPoseEstimator (''%s''): rshank IMU',obj.id),'Interpreter','None');
            % rfoot IMU
            figh(4)=imuPoseEstimator.plotSingleImu(time,obj.m_rfootImuOrientation,obj.m_rfootImuVelocity,obj.m_rfootImuPosition,obj.m_rfootImuGyroBias,obj.m_rfootImuAccelBias,...
                obj.m_rfootImuOrientationPrincipalVariance,obj.m_rfootImuVelocityPrincipalVariance,obj.m_rfootImuPositionPrincipalVariance,obj.m_rfootImuGyroBiasPrincipalVariance,obj.m_rfootImuAccelBiasPrincipalVariance);
            figh(4).Name=sprintf('%s_rfootImuResults',obj.id);
            sgtitle(sprintf('Estimated IMU states for LowerBodyPoseEstimator (''%s''): rfoot IMU',obj.id),'Interpreter','None');
            % lthigh IMU
            figh(5)=imuPoseEstimator.plotSingleImu(time,obj.m_lthighImuOrientation,obj.m_lthighImuVelocity,obj.m_lthighImuPosition,obj.m_lthighImuGyroBias,obj.m_lthighImuAccelBias,...
                obj.m_lthighImuOrientationPrincipalVariance,obj.m_lthighImuVelocityPrincipalVariance,obj.m_lthighImuPositionPrincipalVariance,obj.m_lthighImuGyroBiasPrincipalVariance,obj.m_lthighImuAccelBiasPrincipalVariance);
            figh(5).Name=sprintf('%s_lthighImuResults',obj.id);
            sgtitle(sprintf('Estimated IMU states for LowerBodyPoseEstimator (''%s''): lthigh IMU',obj.id),'Interpreter','None');
            % lshank IMU
            figh(6)=imuPoseEstimator.plotSingleImu(time,obj.m_lshankImuOrientation,obj.m_lshankImuVelocity,obj.m_lshankImuPosition,obj.m_lshankImuGyroBias,obj.m_lshankImuAccelBias,...
                obj.m_lshankImuOrientationPrincipalVariance,obj.m_lshankImuVelocityPrincipalVariance,obj.m_lshankImuPositionPrincipalVariance,obj.m_lshankImuGyroBiasPrincipalVariance,obj.m_lshankImuAccelBiasPrincipalVariance);
            figh(6).Name=sprintf('%s_lshankImuResults',obj.id);
            sgtitle(sprintf('Estimated IMU states for LowerBodyPoseEstimator (''%s''): lshank IMU',obj.id),'Interpreter','None');
            % lfoot IMU
            figh(7)=imuPoseEstimator.plotSingleImu(time,obj.m_lfootImuOrientation,obj.m_lfootImuVelocity,obj.m_lfootImuPosition,obj.m_lfootImuGyroBias,obj.m_lfootImuAccelBias,...
                obj.m_lfootImuOrientationPrincipalVariance,obj.m_lfootImuVelocityPrincipalVariance,obj.m_lfootImuPositionPrincipalVariance,obj.m_lfootImuGyroBiasPrincipalVariance,obj.m_lfootImuAccelBiasPrincipalVariance);
            figh(7).Name=sprintf('%s_lfootImuResults',obj.id);
            sgtitle(sprintf('Estimated IMU states for LowerBodyPoseEstimator (''%s''): lfoot IMU',obj.id),'Interpreter','None');
        end % plot estimated values
        
        function figh=plotInternalPrecessionAngles(obj)
            % plot all internal precession angles on a single plot
            try
                figh=figure('units','normalized','position',[0.05 0.05 0.8 0.8]); hold on;
                plot(obj.m_time,rad2deg(obj.m_sacrumImuInternalPrecessionAng));
                plot(obj.m_time,rad2deg(obj.m_rthighImuInternalPrecessionAng));
                plot(obj.m_time,rad2deg(obj.m_rshankImuInternalPrecessionAng));
                plot(obj.m_time,rad2deg(obj.m_rfootImuInternalPrecessionAng));
                plot(obj.m_time,rad2deg(obj.m_lthighImuInternalPrecessionAng));
                plot(obj.m_time,rad2deg(obj.m_lshankImuInternalPrecessionAng));
                plot(obj.m_time,rad2deg(obj.m_lfootImuInternalPrecessionAng));
                legend({'sacrum','rthigh','rshank','rfoot','lthigh','lshank','lfoot'});
                grid on; xlabel('time (sec)'); ylabel('internal precession angle of IMU (deg)');
                title('internal precession angles of each IMU about their proximal joint vector, relative to first index');
            catch mException
                fprintf('could not plot internal precession angles: error was %s\n',mException.message);
            end
        end
        
        function figh=plotRelativeInternalPrecessionAngles(obj)
            % plot all relative internal precession angles on a single plot
            try
                figh=figure('units','normalized','position',[0.05 0.05 0.8 0.8]); hold on;
                plot(obj.m_time,rad2deg(obj.m_sacrumImuInternalPrecessionAng - obj.m_rthighImuInternalPrecessionAng));
                plot(obj.m_time,rad2deg(obj.m_rthighImuInternalPrecessionAng - obj.m_rshankImuInternalPrecessionAng));
                plot(obj.m_time,rad2deg(obj.m_rshankImuInternalPrecessionAng - obj.m_rfootImuInternalPrecessionAng));
                plot(obj.m_time,rad2deg(obj.m_sacrumImuInternalPrecessionAng - obj.m_lthighImuInternalPrecessionAng));
                plot(obj.m_time,rad2deg(obj.m_lthighImuInternalPrecessionAng - obj.m_lshankImuInternalPrecessionAng));
                plot(obj.m_time,rad2deg(obj.m_lshankImuInternalPrecessionAng - obj.m_lfootImuInternalPrecessionAng));
                legend({'sacrum-rthigh','rthigh-rshank','rshank-rfoot','sacrum-lthigh','lthigh-lshank','lshank-lfoot'});
                grid on; xlabel('time (sec)'); ylabel('relative internal precession angle of IMU (deg)');
                title('relative (proximal IMU - distal IMU) internal precession angles of each IMU about their proximal joint vector, relative to first index');
            catch mException
                fprintf('could not plot internal precession angles: error was %s\n',mException.message);
            end
        end
        
        function figh=plotYawAnglesFromMemberOrientations(obj)
            % plot all 7 yaw angles on the same plot
            figh=figure('units','normalized','position',[0.05 0.05 0.8 0.8]);
            hold on;
            plot(obj.m_time,unwrappedYawAngle(obj.m_sacrumImuOrientation));
            plot(obj.m_time,unwrappedYawAngle(obj.m_rthighImuOrientation));
            plot(obj.m_time,unwrappedYawAngle(obj.m_rshankImuOrientation));
            plot(obj.m_time,unwrappedYawAngle(obj.m_rfootImuOrientation));
            plot(obj.m_time,unwrappedYawAngle(obj.m_lthighImuOrientation));
            plot(obj.m_time,unwrappedYawAngle(obj.m_lshankImuOrientation));
            plot(obj.m_time,unwrappedYawAngle(obj.m_lfootImuOrientation));
            grid on; xlabel('time (sec)'); ylabel('unwrapped yaw angle (rad)');
            legend({'sacrum','rthigh','rshank','rfoot','lthigh','lshank','lfoot'});
            title('unwrapped yaw angles of each IMU''s estimated orientation');
        end
        
        function figh=plotYawAngleBetweenConsecutiveJoints(obj)
            % any two joints in the model should have relatively similar yaw angles over time--the joints shouldn't be able to wrap themselves around
            % this function computes the relative yaw angle between consecutive IMUs (6 yaw angles in total) and plots them
            % () get relative yaw angles for each joint
            yawRShankMinusRFoot=relativeYawAngle(obj.m_rfootImuOrientation,obj.m_rshankImuOrientation);
            yawRThighMinusRShank=relativeYawAngle(obj.m_rshankImuOrientation,obj.m_rthighImuOrientation);
            yawSacrumMinusRThigh=relativeYawAngle(obj.m_rthighImuOrientation,obj.m_sacrumImuOrientation);
            yawSacrumMinusLThigh=relativeYawAngle(obj.m_lthighImuOrientation,obj.m_sacrumImuOrientation);
            yawLShankMinusLFoot=relativeYawAngle(obj.m_lfootImuOrientation,obj.m_lshankImuOrientation);
            yawLThighMinusLShank=relativeYawAngle(obj.m_lshankImuOrientation,obj.m_lthighImuOrientation);
            figh=figure('units','normalized','position',[0.05 0.05 0.8 0.8]);
            plot(obj.m_time,yawRShankMinusRFoot); hold on;
            plot(obj.m_time,yawRThighMinusRShank);
            plot(obj.m_time,yawSacrumMinusRThigh);
            plot(obj.m_time,yawSacrumMinusLThigh);
            plot(obj.m_time,yawLThighMinusLShank);
            plot(obj.m_time,yawLShankMinusLFoot);
            grid on; xlabel('time (sec)'); ylabel('yaw angle difference, radians');
            legend({'RShank-RFoot','RThigh-RShank','Sacrum-RThigh','Sacrum-LThigh','LThigh-LShank','LShank-LFoot'});
            title('absolute yaw differences between neighboring IMUs');
        end
        
        function figh=plotRelativeEstimatedAngularVelocityVectorsRThighFrame(obj)
            % --- options --- %
            plotKneeAxes=1;
            % --------------- %
            % assumes you estimated the angular velocities of each IMU
            for k=1:length(obj.m_rthighImuOrientation)
                relAngVelRThigh(k,:)= obj.m_rthighImuAngVel(k,:)- (obj.m_rthighImuOrientation(k).inverse().matrix()*obj.m_rshankImuOrientation(k).matrix()*obj.m_rshankImuAngVel(k,:)')';   % omega [A-B] in A
            end
            L=max(sqrt(sum(relAngVelRThigh.^2,2)))*1.2;
            figh=figure; hold on;
            for k=1:length(obj.m_rthighImuOrientation)
                line([0 relAngVelRThigh(k,1)],[0 relAngVelRThigh(k,2)],[0 relAngVelRThigh(k,3)],'color',[.7 .7 .7 .8]);
            end
            if plotKneeAxes
                line(L*[0 obj.m_rkneeAxisThigh.point3().x()],L*[0 obj.m_rkneeAxisThigh.point3().y()],L*[0 obj.m_rkneeAxisThigh.point3().z()],'color',[1 0 0]);
            end
            xlabel('x'); ylabel('y'); zlabel('z'); grid on; axis equal;
        end
        
        function figh=plotRKneeRelativeAngularVelocityVectors(obj)
            % --- options --- %
            plotAxes=1;
            nToSkip=50;
            plotType=2; % 1 for spherical model, 2 for heatmap
            % --------------- %
            % assumes you estimated the angular velocities of each IMU
            rkneeAxisThigh=obj.m_rkneeAxisThigh.point3().vector()';
            rkneeAxisShank=obj.m_rkneeAxisShank.point3().vector()';
            relAngVelRThigh=zeros(length(obj.m_rthighImuOrientation),3);
            relAngVelRShank=zeros(length(obj.m_rthighImuOrientation),3);
            for k=1:length(obj.m_rthighImuOrientation)
                relAngVelRThigh(k,:)= obj.m_rthighImuAngVel(k,:)- (obj.m_rthighImuOrientation(k).inverse().matrix()*obj.m_rshankImuOrientation(k).matrix()*obj.m_rshankImuAngVel(k,:)')';   % omega [A-B] in A
                relAngVelRShank(k,:)= obj.m_rshankImuAngVel(k,:)- (obj.m_rshankImuOrientation(k).inverse().matrix()*obj.m_rthighImuOrientation(k).matrix()*obj.m_rthighImuAngVel(k,:)')';   % omega [B-A] in B
            end
            L=max(sqrt(sum(relAngVelRThigh.^2,2)))*1.2;
            figh=figure;
            subplot(2,2,1); hold on; % sacrum frame
            for k=1:nToSkip:length(obj.m_rthighImuOrientation)
                line([0 relAngVelRThigh(k,1)],[0 relAngVelRThigh(k,2)],[0 relAngVelRThigh(k,3)],'color',[.7 .7 .7 .8]);
            end
            if plotAxes
                line(L*[0 rkneeAxisThigh(1)],L*[0 rkneeAxisThigh(2)],L*[0 rkneeAxisThigh(3)],'color',[1 0 0],'linewidth',2);
            end
            xlabel('x'); ylabel('y'); zlabel('z'); grid on; axis equal; title('RThigh IMU frame');
            subplot(2,2,2); hold on; % rthigh frame
            for k=1:nToSkip:length(obj.m_rthighImuOrientation)
                line([0 relAngVelRShank(k,1)],[0 relAngVelRShank(k,2)],[0 relAngVelRShank(k,3)],'color',[.7 .7 .7 .8]);
            end
            if plotAxes
                line(L*[0 rkneeAxisShank(1)],L*[0 rkneeAxisShank(2)],L*[0 rkneeAxisShank(3)],'color',[1 0 0],'linewidth',2);
            end
            xlabel('x'); ylabel('y'); zlabel('z'); grid on; axis equal; title('RShank IMU frame');
            sgtitle('RKnee relative angular velocity in neighboring IMU frames');
            subplot(2,2,3);
            hingeFactorErrorSurface(obj.m_rthighImuOrientation,obj.m_rthighImuAngVel,obj.m_rshankImuOrientation,obj.m_rshankImuAngVel,'A',1,plotType);
            subplot(2,2,4);
            hingeFactorErrorSurface(obj.m_rthighImuOrientation,obj.m_rthighImuAngVel,obj.m_rshankImuOrientation,obj.m_rshankImuAngVel,'B',1,plotType);
        end
        
        function figh=plotLKneeRelativeAngularVelocityVectors(obj)
            % --- options --- %
            plotAxes=1;
            nToSkip=50;
            plotType=2; % 1 for spherical model, 2 for heatmap
            % --------------- %
            % assumes you estimated the angular velocities of each IMU
            lkneeAxisThigh=obj.m_lkneeAxisThigh.point3().vector()';
            lkneeAxisShank=obj.m_lkneeAxisShank.point3().vector()';
            relAngVelLThigh=zeros(length(obj.m_rthighImuOrientation),3);
            relAngVelLShank=zeros(length(obj.m_rthighImuOrientation),3);
            for k=1:length(obj.m_rthighImuOrientation)
                relAngVelLThigh(k,:)= obj.m_lthighImuAngVel(k,:)- (obj.m_lthighImuOrientation(k).inverse().matrix()*obj.m_lshankImuOrientation(k).matrix()*obj.m_lshankImuAngVel(k,:)')';   % omega [A-B] in A
                relAngVelLShank(k,:)= obj.m_lshankImuAngVel(k,:)- (obj.m_lshankImuOrientation(k).inverse().matrix()*obj.m_lthighImuOrientation(k).matrix()*obj.m_lthighImuAngVel(k,:)')';   % omega [B-A] in B
            end
            L=max(sqrt(sum(relAngVelLThigh.^2,2)))*1.2;
            figh=figure;
            subplot(2,2,1); hold on; % sacrum frame
            for k=1:nToSkip:length(obj.m_rthighImuOrientation)
                line([0 relAngVelLThigh(k,1)],[0 relAngVelLThigh(k,2)],[0 relAngVelLThigh(k,3)],'color',[.7 .7 .7 .8]);
            end
            if plotAxes
                line(L*[0 lkneeAxisThigh(1)],L*[0 lkneeAxisThigh(2)],L*[0 lkneeAxisThigh(3)],'color',[1 0 0],'linewidth',2);
            end
            xlabel('x'); ylabel('y'); zlabel('z'); grid on; axis equal; title('LThigh IMU frame');
            subplot(2,2,2); hold on; % rthigh frame
            for k=1:nToSkip:length(obj.m_rthighImuOrientation)
                line([0 relAngVelLShank(k,1)],[0 relAngVelLShank(k,2)],[0 relAngVelLShank(k,3)],'color',[.7 .7 .7 .8]);
            end
            if plotAxes
                line(L*[0 lkneeAxisShank(1)],L*[0 lkneeAxisShank(2)],L*[0 lkneeAxisShank(3)],'color',[1 0 0],'linewidth',2);
            end
            xlabel('x'); ylabel('y'); zlabel('z'); grid on; axis equal; title('LShank IMU frame');
            sgtitle('LKnee relative angular velocity in neighboring IMU frames');
            subplot(2,2,3);
            hingeFactorErrorSurface(obj.m_lthighImuOrientation,obj.m_lthighImuAngVel,obj.m_lshankImuOrientation,obj.m_lshankImuAngVel,'A',1,plotType);
            subplot(2,2,4);
            hingeFactorErrorSurface(obj.m_lthighImuOrientation,obj.m_lthighImuAngVel,obj.m_lshankImuOrientation,obj.m_lshankImuAngVel,'B',1,plotType);
        end
        
        function figh=plotRHipRelativeAngularVelocityVectors(obj)
            % --- options --- %
            plotAxes=1;
            nToSkip=50;
            plotType=2; % 1 for spherical model, 2 for heatmap
            % --------------- %
            % assumes you estimated the angular velocities of each IMU
            rhipAxisSacrum=obj.m_hipAxisSacrum.point3().vector()';
            rhipAxisRThigh=obj.m_rkneeAxisThigh.point3().vector()';
            relAngVelSacrum=zeros(length(obj.m_rthighImuOrientation),3);
            relAngVelRThigh=zeros(length(obj.m_rthighImuOrientation),3);
            for k=1:length(obj.m_rthighImuOrientation)
                relAngVelSacrum(k,:)= obj.m_sacrumImuAngVel(k,:)- (obj.m_sacrumImuOrientation(k).inverse().matrix()*obj.m_rthighImuOrientation(k).matrix()*obj.m_rthighImuAngVel(k,:)')';   % omega [A-B] in A
                relAngVelRThigh(k,:)= obj.m_rthighImuAngVel(k,:)- (obj.m_rthighImuOrientation(k).inverse().matrix()*obj.m_sacrumImuOrientation(k).matrix()*obj.m_sacrumImuAngVel(k,:)')';   % omega [B-A] in B
            end
            L=max(sqrt(sum(relAngVelSacrum.^2,2)))*1.2;
            figh=figure;
            subplot(2,2,1); hold on; % sacrum frame
            for k=1:nToSkip:length(obj.m_rthighImuOrientation)
                line([0 relAngVelSacrum(k,1)],[0 relAngVelSacrum(k,2)],[0 relAngVelSacrum(k,3)],'color',[.7 .7 .7 .8]);
            end
            if plotAxes
                line(L*[0 rhipAxisSacrum(1)],L*[0 rhipAxisSacrum(2)],L*[0 rhipAxisSacrum(3)],'color',[1 0 0],'linewidth',2);
            end
            xlabel('x'); ylabel('y'); zlabel('z'); grid on; axis equal; title('Sacrum IMU frame');
            subplot(2,2,2); hold on; % rthigh frame
            for k=1:nToSkip:length(obj.m_rthighImuOrientation)
                line([0 relAngVelRThigh(k,1)],[0 relAngVelRThigh(k,2)],[0 relAngVelRThigh(k,3)],'color',[.7 .7 .7 .8]);
            end
            if plotAxes
                line(L*[0 rhipAxisRThigh(1)],L*[0 rhipAxisRThigh(2)],L*[0 rhipAxisRThigh(3)],'color',[1 0 0],'linewidth',2);
            end
            xlabel('x'); ylabel('y'); zlabel('z'); grid on; axis equal; title('RThigh IMU frame');
            sgtitle('RHip relative angular velocity in neighboring IMU frames');
            subplot(2,2,3);
            hingeFactorErrorSurface(obj.m_sacrumImuOrientation,obj.m_sacrumImuAngVel,obj.m_rthighImuOrientation,obj.m_rthighImuAngVel,'A',1,plotType);
            subplot(2,2,4);
            hingeFactorErrorSurface(obj.m_sacrumImuOrientation,obj.m_sacrumImuAngVel,obj.m_rthighImuOrientation,obj.m_rthighImuAngVel,'B',1,plotType);
        end
        
        function figh=plotLHipRelativeAngularVelocityVectors(obj)
            % --- options --- %
            plotAxes=1;
            nToSkip=50;
            plotType=2; % 1 for spherical model, 2 for heatmap
            % --------------- %
            % assumes you estimated the angular velocities of each IMU
            rhipAxisSacrum=obj.m_hipAxisSacrum.point3().vector()';
            lhipAxisLThigh=obj.m_lkneeAxisThigh.point3().vector()';
            relAngVelSacrum=zeros(length(obj.m_lthighImuOrientation),3);
            relAngVelLThigh=zeros(length(obj.m_lthighImuOrientation),3);
            for k=1:length(obj.m_lthighImuOrientation)
                relAngVelSacrum(k,:)= obj.m_sacrumImuAngVel(k,:)- (obj.m_sacrumImuOrientation(k).inverse().matrix()*obj.m_lthighImuOrientation(k).matrix()*obj.m_lthighImuAngVel(k,:)')';   % omega [A-B] in A
                relAngVelLThigh(k,:)= obj.m_lthighImuAngVel(k,:)- (obj.m_lthighImuOrientation(k).inverse().matrix()*obj.m_sacrumImuOrientation(k).matrix()*obj.m_sacrumImuAngVel(k,:)')';   % omega [B-A] in B
            end
            L=max(sqrt(sum(relAngVelSacrum.^2,2)))*1.2;
            figh=figure;
            subplot(2,2,1); hold on; % sacrum frame
            for k=1:nToSkip:length(obj.m_lthighImuOrientation)
                line([0 relAngVelSacrum(k,1)],[0 relAngVelSacrum(k,2)],[0 relAngVelSacrum(k,3)],'color',[.7 .7 .7 .8]);
            end
            if plotAxes
                line(L*[0 rhipAxisSacrum(1)],L*[0 rhipAxisSacrum(2)],L*[0 rhipAxisSacrum(3)],'color',[1 0 0],'linewidth',2);
            end
            xlabel('x'); ylabel('y'); zlabel('z'); grid on; axis equal; title('Sacrum IMU frame');
            subplot(2,2,2); hold on; % lthigh frame
            for k=1:nToSkip:length(obj.m_lthighImuOrientation)
                line([0 relAngVelLThigh(k,1)],[0 relAngVelLThigh(k,2)],[0 relAngVelLThigh(k,3)],'color',[.7 .7 .7 .8]);
            end
            if plotAxes
                line(L*[0 lhipAxisLThigh(1)],L*[0 lhipAxisLThigh(2)],L*[0 lhipAxisLThigh(3)],'color',[1 0 0],'linewidth',2);
            end
            xlabel('x'); ylabel('y'); zlabel('z'); grid on; axis equal; title('LThigh IMU frame');
            sgtitle('LHip relative angular velocity in neighboring IMU frames');
            subplot(2,2,3);
            hingeFactorErrorSurface(obj.m_sacrumImuOrientation,obj.m_sacrumImuAngVel,obj.m_lthighImuOrientation,obj.m_lthighImuAngVel,'A',1,plotType);
            subplot(2,2,4);
            hingeFactorErrorSurface(obj.m_sacrumImuOrientation,obj.m_sacrumImuAngVel,obj.m_lthighImuOrientation,obj.m_lthighImuAngVel,'B',1,plotType);
        end
        
        function figh=plotPelvisRelativeAngularVelocityVectors(obj)
            % the "pelvis" joint here is the fictional hinge joint between the thigh IMUs
            % --- options --- %
            plotAxes=1;
            nToSkip=50;
            plotType=2; % 1 for spherical model, 2 for heatmap
            % --------------- %
            % assumes you estimated the angular velocities of each IMU
            % also need hip axes for this
            if isempty(obj.m_rhipAxisThigh) || isempty(obj.m_rhipAxisThigh)
                figh=figure; text(0,0.5,'could not plot this because you didnt estimate hip axes');
            else
                rhipAxisLThigh=obj.m_lhipAxisThigh.point3().vector()';
                rhipAxisRThigh=obj.m_rhipAxisThigh.point3().vector()';
                relAngVelLThigh=zeros(length(obj.m_rthighImuOrientation),3);
                relAngVelRThigh=zeros(length(obj.m_rthighImuOrientation),3);
                for k=1:length(obj.m_rthighImuOrientation)
                    relAngVelLThigh(k,:)= obj.m_lthighImuAngVel(k,:)- (obj.m_lthighImuOrientation(k).inverse().matrix()*obj.m_rthighImuOrientation(k).matrix()*obj.m_rthighImuAngVel(k,:)')';   % omega [A-B] in A
                    relAngVelRThigh(k,:)= obj.m_rthighImuAngVel(k,:)- (obj.m_rthighImuOrientation(k).inverse().matrix()*obj.m_lthighImuOrientation(k).matrix()*obj.m_lthighImuAngVel(k,:)')';   % omega [B-A] in B
                end
                L=max(sqrt(sum(relAngVelLThigh.^2,2)))*1.2;
                figh=figure;
                subplot(2,2,1); hold on; % sacrum frame
                for k=1:nToSkip:length(obj.m_rthighImuOrientation)
                    line([0 relAngVelLThigh(k,1)],[0 relAngVelLThigh(k,2)],[0 relAngVelLThigh(k,3)],'color',[.7 .7 .7 .8]);
                end
                if plotAxes
                    line(L*[0 rhipAxisLThigh(1)],L*[0 rhipAxisLThigh(2)],L*[0 rhipAxisLThigh(3)],'color',[1 0 0],'linewidth',2);
                end
                xlabel('x'); ylabel('y'); zlabel('z'); grid on; axis equal; title('LThigh IMU frame');
                subplot(2,2,2); hold on; % rthigh frame
                for k=1:nToSkip:length(obj.m_rthighImuOrientation)
                    line([0 relAngVelRThigh(k,1)],[0 relAngVelRThigh(k,2)],[0 relAngVelRThigh(k,3)],'color',[.7 .7 .7 .8]);
                end
                if plotAxes
                    line(L*[0 rhipAxisRThigh(1)],L*[0 rhipAxisRThigh(2)],L*[0 rhipAxisRThigh(3)],'color',[1 0 0],'linewidth',2);
                end
                xlabel('x'); ylabel('y'); zlabel('z'); grid on; axis equal; title('RThigh IMU frame');
                sgtitle('Pelvis relative angular velocity in neighboring IMU frames');
                subplot(2,2,3);
                hingeFactorErrorSurface(obj.m_lthighImuOrientation,obj.m_lthighImuAngVel,obj.m_rthighImuOrientation,obj.m_rthighImuAngVel,'A',1,plotType);
                subplot(2,2,4);
                hingeFactorErrorSurface(obj.m_lthighImuOrientation,obj.m_lthighImuAngVel,obj.m_rthighImuOrientation,obj.m_rthighImuAngVel,'B',1,plotType);
            end
        end
        
        function figh=plotJointConnectionPositionErrors(obj)
            % plot the hip, knee, and ankle joint alignment errors (position)
            % according to the connection factor
            errRHip=legPoseEstimator.jointAlignmentError(obj.m_sacrumImuOrientation,obj.m_sacrumImuPosition,obj.m_sacrumImuToRHipCtr,obj.m_rthighImuOrientation,obj.m_rthighImuPosition,obj.m_rthighImuToHipCtr);
            errRKnee=legPoseEstimator.jointAlignmentError(obj.m_rthighImuOrientation,obj.m_rthighImuPosition,obj.m_rthighImuToKneeCtr,obj.m_rshankImuOrientation,obj.m_rshankImuPosition,obj.m_rshankImuToKneeCtr);
            errRAnkle=legPoseEstimator.jointAlignmentError(obj.m_rshankImuOrientation,obj.m_rshankImuPosition,obj.m_rshankImuToAnkleCtr,obj.m_rfootImuOrientation,obj.m_rfootImuPosition,obj.m_rfootImuToAnkleCtr);
            errLHip=legPoseEstimator.jointAlignmentError(obj.m_sacrumImuOrientation,obj.m_sacrumImuPosition,obj.m_sacrumImuToLHipCtr,obj.m_lthighImuOrientation,obj.m_lthighImuPosition,obj.m_lthighImuToHipCtr);
            errLKnee=legPoseEstimator.jointAlignmentError(obj.m_lthighImuOrientation,obj.m_lthighImuPosition,obj.m_lthighImuToKneeCtr,obj.m_lshankImuOrientation,obj.m_lshankImuPosition,obj.m_lshankImuToKneeCtr);
            errLAnkle=legPoseEstimator.jointAlignmentError(obj.m_lshankImuOrientation,obj.m_lshankImuPosition,obj.m_lshankImuToAnkleCtr,obj.m_lfootImuOrientation,obj.m_lfootImuPosition,obj.m_lfootImuToAnkleCtr);
            t=obj.m_time;
            figh=figure('units','normalized','position',[.05 .05 .8 .5]); hold on;
            plot(t,sqrt(sum(errRHip.^2,2)));
            plot(t,sqrt(sum(errRKnee.^2,2)));
            plot(t,sqrt(sum(errRAnkle.^2,2)));
            plot(t,sqrt(sum(errLHip.^2,2)));
            plot(t,sqrt(sum(errLKnee.^2,2)));
            plot(t,sqrt(sum(errLAnkle.^2,2)));
            legend({'RHip','RKnee','RAnkle','LHip','LKnee','LAnkle'});
            grid on; ylabel('norm error (m)'); title('joint connection err for all 6 joints present in model'); xlabel('time (sec)');
        end
        
        function figh=plotJointConnectionVelocityErrors(obj)
            % plot the hip, knee, and ankle joint alignment errors (velocity)
            % according to the connection factor
            errRHip=lowerBodyPoseEstimator.jointVelocityError(obj.m_sacrumImuOrientation,obj.m_sacrumImuVelocity,obj.m_sacrumImuAngVel,obj.m_sacrumImuToRHipCtr,obj.m_rthighImuOrientation,obj.m_rthighImuVelocity,obj.m_rthighImuAngVel,obj.m_rthighImuToHipCtr);
            errRKnee=lowerBodyPoseEstimator.jointVelocityError(obj.m_rthighImuOrientation,obj.m_rthighImuVelocity,obj.m_rthighImuAngVel,obj.m_rthighImuToKneeCtr,obj.m_rshankImuOrientation,obj.m_rshankImuVelocity,obj.m_rshankImuAngVel,obj.m_rshankImuToKneeCtr);
            errRAnkle=lowerBodyPoseEstimator.jointVelocityError(obj.m_rshankImuOrientation,obj.m_rshankImuVelocity,obj.m_rshankImuAngVel,obj.m_rshankImuToAnkleCtr,obj.m_rfootImuOrientation,obj.m_rfootImuVelocity,obj.m_rfootImuAngVel,obj.m_rfootImuToAnkleCtr);
            errLHip=lowerBodyPoseEstimator.jointVelocityError(obj.m_sacrumImuOrientation,obj.m_sacrumImuVelocity,obj.m_sacrumImuAngVel,obj.m_sacrumImuToLHipCtr,obj.m_lthighImuOrientation,obj.m_lthighImuVelocity,obj.m_lthighImuAngVel,obj.m_lthighImuToHipCtr);
            errLKnee=lowerBodyPoseEstimator.jointVelocityError(obj.m_lthighImuOrientation,obj.m_lthighImuVelocity,obj.m_lthighImuAngVel,obj.m_lthighImuToKneeCtr,obj.m_lshankImuOrientation,obj.m_lshankImuVelocity,obj.m_lshankImuAngVel,obj.m_lshankImuToKneeCtr);
            errLAnkle=lowerBodyPoseEstimator.jointVelocityError(obj.m_lshankImuOrientation,obj.m_lshankImuVelocity,obj.m_lshankImuAngVel,obj.m_lshankImuToAnkleCtr,obj.m_lfootImuOrientation,obj.m_lfootImuVelocity,obj.m_lfootImuAngVel,obj.m_lfootImuToAnkleCtr);
            t=obj.m_time;
            figh=figure('units','normalized','position',[.05 .05 .8 .5]); hold on;
            plot(t,sqrt(sum(errRHip.^2,2)));
            plot(t,sqrt(sum(errRKnee.^2,2)));
            plot(t,sqrt(sum(errRAnkle.^2,2)));
            plot(t,sqrt(sum(errLHip.^2,2)));
            plot(t,sqrt(sum(errLKnee.^2,2)));
            plot(t,sqrt(sum(errLAnkle.^2,2)));
            legend({'RHip','RKnee','RAnkle','LHip','LKnee','LAnkle'});
            grid on; ylabel('norm difference (m/s)'); title('joint connection velocity difference for all 6 joints present in model'); xlabel('time (sec)');
        end
        
        function figh=plotRHipAngles(obj)
            % compute and then plot the right hip's 3 angles
            % remember sign convention for clinical angles: positive in hip flexion, adduction, internal rotation
            [flexionExtensionAngle,intExtAng,abAdAng]=bruteForceRHipAngleCalc(obj);
            figh=plotClinicalHipAngles(obj.m_time,flexionExtensionAngle,abAdAng,intExtAng,strcat(obj.id,'_RHip'));
        end
        
        function figh=plotLHipAngles(obj)
            % compute and then plot the left hip's 3 angles
            % remember sign convention for clinical angles: positive in hip flexion, adduction, internal rotation
            [flexionExtensionAngle,intExtAng,abAdAng]=bruteForceLHipAngleCalc(obj);
            figh=plotClinicalHipAngles(obj.m_time,flexionExtensionAngle,abAdAng,intExtAng,strcat(obj.id,'_LHip'));
        end
        
        function figh=plotRKneeAngles(obj)
            % compute and then plot the knee's 3 angles
            % if you constrained these angles in the model, add a dotted red line to represent the limit you constrained to
            % remember sign convention for clinical angles: positive in knee extension, adduction, internal rotation
            [flexionExtensionAngle,intExtAng,varusValgusAng]=bruteForceRKneeAngleCalc(obj);
            figh=figure('units','normalized','position',[.05 .05 .6 .8]);
            subplot(3,1,1); hold on;
            plot(obj.m_time,rad2deg(flexionExtensionAngle),'k','linewidth',2);
            ylabel('\leftarrow flexion | extension \rightarrow'); grid on;
            title(sprintf('right knee angles (deg) for ''%s''',obj.id),'interpreter','none');
            subplot(3,1,2);
            plot(obj.m_time,rad2deg(intExtAng),'k','linewidth',2);
            ylabel('\leftarrow ext. rot | int. rot \rightarrow'); grid on;
            subplot(3,1,3);
            plot(obj.m_time,rad2deg(varusValgusAng),'k','linewidth',2);
            ylabel('\leftarrow abduction | adduction \rightarrow'); grid on; xlabel('time (sec)');
        end
        
        function figh=plotLKneeAngles(obj)
            % compute and then plot the knee's 3 angles
            % if you constrained these angles in the model, add a dotted red line to represent the limit you constrained to
            % remember sign convention for clinical angles: positive in knee extension, adduction, internal rotation
            [flexionExtensionAngle,intExtAng,varusValgusAng]=bruteForceLKneeAngleCalc(obj);
            figh=figure('units','normalized','position',[.05 .05 .6 .8]);
            subplot(3,1,1);
            plot(obj.m_time,rad2deg(flexionExtensionAngle),'k','linewidth',2);
            ylabel('\leftarrow flexion | extension \rightarrow'); grid on;
            title(sprintf('left knee angles (deg) for ''%s''',obj.id),'interpreter','none');
            subplot(3,1,2);
            plot(obj.m_time,rad2deg(intExtAng),'k','linewidth',2);
            ylabel('\leftarrow ext. rot | int. rot \rightarrow'); grid on;
            subplot(3,1,3);
            plot(obj.m_time,rad2deg(varusValgusAng),'k','linewidth',2);
            ylabel('\leftarrow abduction | adduction \rightarrow'); grid on; xlabel('time (sec)');
        end
        
        function figh=plotKneeHingeAxesInNavFrame(obj)
            % intuitively you'd hope that when the thigh and shank hinge axis are rotated into the nav frame, that
            % they point in the same direction for every time step. This function computes the error and plots it.
            % get data for right knee
            rthighAxis=obj.m_rkneeAxisThigh.point3().vector()';
            rshankAxis=obj.m_rkneeAxisShank.point3().vector()';
            [errRmseR,axisErrDegR]=hingeAxesNavFrameComparisonRmse(rthighAxis,rshankAxis,obj.m_rthighImuOrientation,obj.m_rshankImuOrientation);
            % get data for left knee
            lthighAxis=obj.m_lkneeAxisThigh.point3().vector()';
            lshankAxis=obj.m_lkneeAxisShank.point3().vector()';
            [errRmseL,axisErrDegL]=hingeAxesNavFrameComparisonRmse(lthighAxis,lshankAxis,obj.m_lthighImuOrientation,obj.m_lshankImuOrientation);
            figh=figure('units','normalized','position',[2 2 6 12]);
            subplot(2,1,1)
            plot(obj.m_time,axisErrDegR,'r','linewidth',2);
            grid on; ylabel('angle between axes rotated into nav frame');
            title(sprintf('(Right knee) angle error, axes in nav frame: %.4f deg RMSE',errRmseR));
            subplot(2,1,2)
            plot(obj.m_time,axisErrDegL,'b','linewidth',2);
            grid on; xlabel('Time (sec)'); ylabel('angle between axes rotated into nav frame');
            title(sprintf('(Left knee) Angle error, axes in nav frame: %.4f deg RMSE',errRmseL));
        end
        
        function figh=plotHipHingeAxesInNavFrame(obj)
            % intuitively you'd hope that when the thigh and shank hinge axis are rotated into the nav frame, that
            % they point in the same direction for every time step. This function computes the error and plots it.
            sacrumAxis=obj.m_hipAxisSacrum.point3().vector()';
            rthighAxis=obj.m_rkneeAxisThigh.point3().vector()';
            lthighAxis=obj.m_lkneeAxisThigh.point3().vector()';
            [errRmseR,axisErrDegR]=hingeAxesNavFrameComparisonRmse(sacrumAxis,rthighAxis,obj.m_sacrumImuOrientation,obj.m_rthighImuOrientation);
            [errRmseL,axisErrDegL]=hingeAxesNavFrameComparisonRmse(sacrumAxis,lthighAxis,obj.m_sacrumImuOrientation,obj.m_lthighImuOrientation);
            figh=figure('units','normalized','position',[2 2 6 12]);
            subplot(2,1,1)
            plot(obj.m_time,axisErrDegR,'r','linewidth',2);
            grid on; ylabel('angle between axes rotated into nav frame');
            title(sprintf('(Right hip) angle error, axes in nav frame: %.4f deg RMSE',errRmseR));
            subplot(2,1,2)
            plot(obj.m_time,axisErrDegL,'b','linewidth',2);
            grid on; xlabel('Time (sec)'); ylabel('angle between axes rotated into nav frame');
            title(sprintf('(Left hip) Angle error, axes in nav frame: %.4f deg RMSE',errRmseL));
        end
        
        function figh=plotOptimizerErrorOverTime(obj)
            % plot on a semilogy the optimization error per iteration/time
            figh=figure('units','normalized','position',[.2 .2 .4 .4]);
            ax1=axes('Position',[0.1 0.1100 0.8 0.8150],'XAxisLocation','bottom','YAxisLocation','left','Color','none');
            line(obj.m_optimizationTotalTime,obj.m_optimizationTotalError,'Color',[.1 .1 .6],'linestyle',':'); hold on;
            scatter(obj.m_optimizationTotalTime,obj.m_optimizationTotalError,10,'k','filled');
            grid on; set(ax1,'YScale','log'); xlabel('time (sec)'); ylabel('global error');
            title(sprintf('Optimizer error over time for LowerBodyPoseEstimator ''%s''',obj.id),'interpreter','none');
            % finally, add text at last point about total error, time, iter
            yl=ylim; txtpt=[.7*obj.m_optimizationTotalTime(end) 5*obj.m_optimizationTotalError(end)];
            textstr=sprintf('final: err=%.4f, nIter=%d, time=%.3f sec',obj.m_optimizationTotalError(end),length(obj.m_optimizationTotalTime),obj.m_optimizationTotalTime(end));
            text(txtpt(1),txtpt(2),textstr);
        end
        
        function figh=plotEstimatedAngularVelocity(obj)
            % if you estimated the angular velocity, it's worth plotting this against the measurements for comparison
            figh=figure('units','normalized','position',[.05 .05 .8 .8]);
            subplot(4,2,1); hold on;
            plot(obj.m_time,obj.m_sacrumImuAngVel(:,1),'color','r');
            plot(obj.m_time,obj.m_sacrumImuAngVel(:,2),'color',[.1 .5 .1]);
            plot(obj.m_time,obj.m_sacrumImuAngVel(:,3),'color','b');
            plot(obj.m_sacrumImuPoseProblem.m_imu.time,obj.m_sacrumImuPoseProblem.m_imu.gx,'--','color','r');
            plot(obj.m_sacrumImuPoseProblem.m_imu.time,obj.m_sacrumImuPoseProblem.m_imu.gy,'--','color',[.1 .5 .1]);
            plot(obj.m_sacrumImuPoseProblem.m_imu.time,obj.m_sacrumImuPoseProblem.m_imu.gz,'--','color','b'); grid on; ylabel('Sacrum IMU');
            subplot(4,2,3); hold on;
            plot(obj.m_time,obj.m_rthighImuAngVel(:,1),'color','r');
            plot(obj.m_time,obj.m_rthighImuAngVel(:,2),'color',[.1 .5 .1]);
            plot(obj.m_time,obj.m_rthighImuAngVel(:,3),'color','b');
            plot(obj.m_rthighImuPoseProblem.m_imu.time,obj.m_rthighImuPoseProblem.m_imu.gx,'--','color','r');
            plot(obj.m_rthighImuPoseProblem.m_imu.time,obj.m_rthighImuPoseProblem.m_imu.gy,'--','color',[.1 .5 .1]);
            plot(obj.m_rthighImuPoseProblem.m_imu.time,obj.m_rthighImuPoseProblem.m_imu.gz,'--','color','b'); grid on; ylabel('RThigh IMU');
            subplot(4,2,5); hold on;
            plot(obj.m_time,obj.m_rshankImuAngVel(:,1),'color','r');
            plot(obj.m_time,obj.m_rshankImuAngVel(:,2),'color',[.1 .5 .1]);
            plot(obj.m_time,obj.m_rshankImuAngVel(:,3),'color','b');
            plot(obj.m_rshankImuPoseProblem.m_imu.time,obj.m_rshankImuPoseProblem.m_imu.gx,'--','color','r');
            plot(obj.m_rshankImuPoseProblem.m_imu.time,obj.m_rshankImuPoseProblem.m_imu.gy,'--','color',[.1 .5 .1]);
            plot(obj.m_rshankImuPoseProblem.m_imu.time,obj.m_rshankImuPoseProblem.m_imu.gz,'--','color','b'); grid on; ylabel('RShank IMU');
            subplot(4,2,7); hold on;
            plot(obj.m_time,obj.m_rfootImuAngVel(:,1),'color','r');
            plot(obj.m_time,obj.m_rfootImuAngVel(:,2),'color',[.1 .5 .1]);
            plot(obj.m_time,obj.m_rfootImuAngVel(:,3),'color','b');
            plot(obj.m_rfootImuPoseProblem.m_imu.time,obj.m_rfootImuPoseProblem.m_imu.gx,'--','color','r');
            plot(obj.m_rfootImuPoseProblem.m_imu.time,obj.m_rfootImuPoseProblem.m_imu.gy,'--','color',[.1 .5 .1]);
            plot(obj.m_rfootImuPoseProblem.m_imu.time,obj.m_rfootImuPoseProblem.m_imu.gz,'--','color','b'); grid on; ylabel('RFoot IMU'); xlabel('time (sec)');
            subplot(4,2,4); hold on;
            plot(obj.m_time,obj.m_lthighImuAngVel(:,1),'color','r');
            plot(obj.m_time,obj.m_lthighImuAngVel(:,2),'color',[.1 .5 .1]);
            plot(obj.m_time,obj.m_lthighImuAngVel(:,3),'color','b');
            plot(obj.m_lthighImuPoseProblem.m_imu.time,obj.m_lthighImuPoseProblem.m_imu.gx,'--','color','r');
            plot(obj.m_lthighImuPoseProblem.m_imu.time,obj.m_lthighImuPoseProblem.m_imu.gy,'--','color',[.1 .5 .1]);
            plot(obj.m_lthighImuPoseProblem.m_imu.time,obj.m_lthighImuPoseProblem.m_imu.gz,'--','color','b'); grid on; ylabel('LThigh IMU');
            subplot(4,2,6); hold on;
            plot(obj.m_time,obj.m_lshankImuAngVel(:,1),'color','r');
            plot(obj.m_time,obj.m_lshankImuAngVel(:,2),'color',[.1 .5 .1]);
            plot(obj.m_time,obj.m_lshankImuAngVel(:,3),'color','b');
            plot(obj.m_lshankImuPoseProblem.m_imu.time,obj.m_lshankImuPoseProblem.m_imu.gx,'--','color','r');
            plot(obj.m_lshankImuPoseProblem.m_imu.time,obj.m_lshankImuPoseProblem.m_imu.gy,'--','color',[.1 .5 .1]);
            plot(obj.m_lshankImuPoseProblem.m_imu.time,obj.m_lshankImuPoseProblem.m_imu.gz,'--','color','b'); grid on; ylabel('LShank IMU');
            subplot(4,2,8); hold on;
            plot(obj.m_time,obj.m_lfootImuAngVel(:,1),'color','r');
            plot(obj.m_time,obj.m_lfootImuAngVel(:,2),'color',[.1 .5 .1]);
            plot(obj.m_time,obj.m_lfootImuAngVel(:,3),'color','b');
            plot(obj.m_lfootImuPoseProblem.m_imu.time,obj.m_lfootImuPoseProblem.m_imu.gx,'--','color','r');
            plot(obj.m_lfootImuPoseProblem.m_imu.time,obj.m_lfootImuPoseProblem.m_imu.gy,'--','color',[.1 .5 .1]);
            plot(obj.m_lfootImuPoseProblem.m_imu.time,obj.m_lfootImuPoseProblem.m_imu.gz,'--','color','b'); grid on; ylabel('LFoot IMU'); xlabel('time (sec)');
            sgtitle('Estimated (solid) vs. IMU measured (dotted) angular velocities for each IMU in rad/s');
        end
        
        %%% animation method %%%%
        function animate(obj,animFileName)
            if ~exist('animFileName'); animFileName=[]; end % if you didn't pass in animFileName, call it empty
            % () pull out data for ease in variables
            pSacrumImu=Point3ArrayToMatrix(obj.m_sacrumImuPosition);
            RSacrumImuToN=Rot3ArrayToMatrices(obj.m_sacrumImuOrientation);
            pRThighImu=Point3ArrayToMatrix(obj.m_rthighImuPosition);
            RRThighImuToN=Rot3ArrayToMatrices(obj.m_rthighImuOrientation);
            pLThighImu=Point3ArrayToMatrix(obj.m_lthighImuPosition);
            RLThighImuToN=Rot3ArrayToMatrices(obj.m_lthighImuOrientation);
            pRShankImu=Point3ArrayToMatrix(obj.m_rshankImuPosition);
            RRShankImuToN=Rot3ArrayToMatrices(obj.m_rshankImuOrientation);
            pLShankImu=Point3ArrayToMatrix(obj.m_lshankImuPosition);
            RLShankImuToN=Rot3ArrayToMatrices(obj.m_lshankImuOrientation);
            pRFootImu=Point3ArrayToMatrix(obj.m_rfootImuPosition);
            RRFootImuToN=Rot3ArrayToMatrices(obj.m_rfootImuOrientation);
            pLFootImu=Point3ArrayToMatrix(obj.m_lfootImuPosition);
            RLFootImuToN=Rot3ArrayToMatrices(obj.m_lfootImuOrientation);
            % () anatomical R
            R_Pelvis_to_N=get_R_Pelvis_to_N(obj);
            [~,~,~,R_RFemur_to_N,R_RTibia_to_N]=bruteForceRKneeAngleCalc(obj);
            [~,~,~,R_LFemur_to_N,R_LTibia_to_N]=bruteForceLKneeAngleCalc(obj);
            % () set structs
            pos=struct('pSacrumImu',pSacrumImu,'pRThighImu',pRThighImu,'pLThighImu',pLThighImu,'pRShankImu',pRShankImu,'pLShankImu',pLShankImu,'pRFootImu',pRFootImu,'pLFootImu',pLFootImu);
            Rimu=struct('RSacrumImuToN',RSacrumImuToN,'RRThighImuToN',RRThighImuToN,'RLThighImuToN',RLThighImuToN,'RRShankImuToN',RRShankImuToN,'RLShankImuToN',RLShankImuToN,'RRFootImuToN',RRFootImuToN,'RLFootImuToN',RLFootImuToN);
            Rseg=struct('RPelvisToN',R_Pelvis_to_N,'RRFemurToN',R_RFemur_to_N,'RRTibiaToN',R_RTibia_to_N,'RLFemurToN',R_LFemur_to_N,'RLTibiaToN',R_LTibia_to_N);
            vec=struct('sacrumImuToRHipCtr',obj.m_sacrumImuToRHipCtr.vector()','sacrumImuToLHipCtr',obj.m_sacrumImuToLHipCtr.vector()','rthighImuToHipCtr',obj.m_rthighImuToHipCtr.vector()','rthighImuToKneeCtr',obj.m_rthighImuToKneeCtr.vector()',...
                'lthighImuToHipCtr',obj.m_lthighImuToHipCtr.vector()','lthighImuToKneeCtr',obj.m_lthighImuToKneeCtr.vector()','rshankImuToKneeCtr',obj.m_rshankImuToKneeCtr.vector()','rshankImuToAnkleCtr',obj.m_rshankImuToAnkleCtr.vector()',...
                'lshankImuToKneeCtr',obj.m_lshankImuToKneeCtr.vector()','lshankImuToAnkleCtr',obj.m_lshankImuToAnkleCtr.vector()','rfootImuToAnkleCtr',obj.m_rfootImuToAnkleCtr.vector()','lfootImuToAnkleCtr',obj.m_lfootImuToAnkleCtr.vector()');
            % () export to function
            animateLowerBodySkeletalSystem(pos,Rimu,vec,Rseg,animFileName);
        end
        
        %++++++++++++++++++++++++++++++++++++++++++++++++%
        % -------- methods for editing priors ---------- %
        %++++++++++++++++++++++++++++++++++++++++++++++++%
        function setConsistentHeadingPriorVariancesRThighImuOrigin(obj)
            % you have a multiple IMU system. it doesn't make sense to have strong priors on each one's orientation--if you are using magnetometer-free esitmation,
            %   then they likely won't all start at the same heading! set the rthigh IMU as the strong orientation prior and weaken the others.
            % TODO: can you weaken only the heading component?
            % () first pull out my values
            fv=NonlinearFactorGraphToFactorVector(obj.m_graph);
            priorPose3Factors={}; priorPose3FactorsIdx=[]; priorPose3FactorsKeys={};
            for k=1:length(fv)
                if strcmp(class(fv{k}),'gtsam.PriorFactorPose3')
                    priorPose3Factors={priorPose3Factors{:},fv{k}};
                    priorPose3FactorsIdx=[priorPose3FactorsIdx,k];
                end
            end
            % () what should we consider a large and small variance on orientation?
            largePriorRotNoise=1e3*[1 1 1];
            smallPriorRotNoise=1e-1*[1 1 1];
            %%% hardcoding (bad!) order of these priors. 1: sacrum, 2: rthigh, 3: rshank, 4: rfoot, 5: lthigh, 6: lshank, 7:lfoot
            % () set these to named variables
            sacrumImuPriorPose3FactorOld=priorPose3Factors{1};
            rthighImuPriorPose3FactorOld=priorPose3Factors{2};
            rshankImuPriorPose3FactorOld=priorPose3Factors{3};
            rfootImuPriorPose3FactorOld=priorPose3Factors{4};
            lthighImuPriorPose3FactorOld=priorPose3Factors{5};
            lshankImuPriorPose3FactorOld=priorPose3Factors{6};
            lfootImuPriorPose3FactorOld=priorPose3Factors{7};
            % () pull out old means (gtsam.Pose3)
            sacrumImuPriorPose3FactorOldMean=sacrumImuPriorPose3FactorOld.prior;
            rthighImuPriorPose3FactorOldMean=rthighImuPriorPose3FactorOld.prior;
            rshankImuPriorPose3FactorOldMean=rshankImuPriorPose3FactorOld.prior;
            rfootImuPriorPose3FactorOldMean=rfootImuPriorPose3FactorOld.prior;
            lthighImuPriorPose3FactorOldMean=lthighImuPriorPose3FactorOld.prior;
            lshankImuPriorPose3FactorOldMean=lshankImuPriorPose3FactorOld.prior;
            lfootImuPriorPose3FactorOldMean=lfootImuPriorPose3FactorOld.prior;
            % () pull out old sigmas too (6x1 array)
            sacrumImuPriorPose3FactorOldSigmas=sacrumImuPriorPose3FactorOld.noiseModel.sigmas;
            rthighImuPriorPose3FactorOldSigmas=rthighImuPriorPose3FactorOld.noiseModel.sigmas;
            rshankImuPriorPose3FactorOldSigmas=rshankImuPriorPose3FactorOld.noiseModel.sigmas;
            rfootImuPriorPose3FactorOldSigmas=rfootImuPriorPose3FactorOld.noiseModel.sigmas;
            lthighImuPriorPose3FactorOldSigmas=lthighImuPriorPose3FactorOld.noiseModel.sigmas;
            lshankImuPriorPose3FactorOldSigmas=lshankImuPriorPose3FactorOld.noiseModel.sigmas;
            lfootImuPriorPose3FactorOldSigmas=lfootImuPriorPose3FactorOld.noiseModel.sigmas;
            % () now given the old sigmas what should the new ones be?
            % --> change thigh IMU orientation sigma to the small, and the others to the large. Remember that orientation is the first 3 of the 6x1 pose variance.
            sacrumImuPriorPose3FactorNewSigmas=vertcat(largePriorRotNoise(:),sacrumImuPriorPose3FactorOldSigmas(4:6));
            rthighImuPriorPose3FactorNewSigmas=vertcat(smallPriorRotNoise(:),rthighImuPriorPose3FactorOldSigmas(4:6));
            rshankImuPriorPose3FactorNewSigmas=vertcat(largePriorRotNoise(:),rshankImuPriorPose3FactorOldSigmas(4:6));
            rfootImuPriorPose3FactorNewSigmas=vertcat(largePriorRotNoise(:),rfootImuPriorPose3FactorOldSigmas(4:6));
            lthighImuPriorPose3FactorNewSigmas=vertcat(largePriorRotNoise(:),lthighImuPriorPose3FactorOldSigmas(4:6));
            lshankImuPriorPose3FactorNewSigmas=vertcat(largePriorRotNoise(:),lshankImuPriorPose3FactorOldSigmas(4:6));
            lfootImuPriorPose3FactorNewSigmas=vertcat(largePriorRotNoise(:),lfootImuPriorPose3FactorOldSigmas(4:6));
            % () create new noise models
            sacrumImuPriorPose3ModelNew=gtsam.noiseModel.Diagonal.Sigmas(sacrumImuPriorPose3FactorNewSigmas); % rad, rad, rad, m, m, m
            rthighImuPriorPose3ModelNew=gtsam.noiseModel.Diagonal.Sigmas(rthighImuPriorPose3FactorNewSigmas); % rad, rad, rad, m, m, m
            rshankImuPriorPose3ModelNew=gtsam.noiseModel.Diagonal.Sigmas(rshankImuPriorPose3FactorNewSigmas); % rad, rad, rad, m, m, m
            rfootImuPriorPose3ModelNew=gtsam.noiseModel.Diagonal.Sigmas(rfootImuPriorPose3FactorNewSigmas); % rad, rad, rad, m, m, m
            lthighImuPriorPose3ModelNew=gtsam.noiseModel.Diagonal.Sigmas(lthighImuPriorPose3FactorNewSigmas); % rad, rad, rad, m, m, m
            lshankImuPriorPose3ModelNew=gtsam.noiseModel.Diagonal.Sigmas(lshankImuPriorPose3FactorNewSigmas); % rad, rad, rad, m, m, m
            lfootImuPriorPose3ModelNew=gtsam.noiseModel.Diagonal.Sigmas(lfootImuPriorPose3FactorNewSigmas); % rad, rad, rad, m, m, m
            % () construct new prior factors
            sacrumImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_sacrumPoseVarChar,0),sacrumImuPriorPose3FactorOldMean,sacrumImuPriorPose3ModelNew);
            rthighImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_rthighPoseVarChar,0),rthighImuPriorPose3FactorOldMean,rthighImuPriorPose3ModelNew);
            rshankImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_rshankPoseVarChar,0),rshankImuPriorPose3FactorOldMean,rshankImuPriorPose3ModelNew);
            rfootImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_rfootPoseVarChar,0),rfootImuPriorPose3FactorOldMean,rfootImuPriorPose3ModelNew);
            lthighImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_lthighPoseVarChar,0),lthighImuPriorPose3FactorOldMean,lthighImuPriorPose3ModelNew);
            lshankImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_lshankPoseVarChar,0),lshankImuPriorPose3FactorOldMean,lshankImuPriorPose3ModelNew);
            lfootImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_lfootPoseVarChar,0),lfootImuPriorPose3FactorOldMean,lfootImuPriorPose3ModelNew);
            % () now replace() those old factors with the new one
            obj.m_graph.replace(priorPose3FactorsIdx(1)-1,sacrumImuPriorPose3FactorNew); % sacrum IMU prior
            obj.m_graph.replace(priorPose3FactorsIdx(2)-1,rthighImuPriorPose3FactorNew); % thigh IMU prior
            obj.m_graph.replace(priorPose3FactorsIdx(3)-1,rshankImuPriorPose3FactorNew); % shank IMU prior
            obj.m_graph.replace(priorPose3FactorsIdx(4)-1,rfootImuPriorPose3FactorNew); % foot IMU prior
            obj.m_graph.replace(priorPose3FactorsIdx(5)-1,lthighImuPriorPose3FactorNew); % thigh IMU prior
            obj.m_graph.replace(priorPose3FactorsIdx(6)-1,lshankImuPriorPose3FactorNew); % shank IMU prior
            obj.m_graph.replace(priorPose3FactorsIdx(7)-1,lfootImuPriorPose3FactorNew); % foot IMU prior
            fprintf('\tset consistent prior rotation sigmas: rthigh IMU small @ [%.2g %.2g %.2g], others large @ [%.2g %.2g %.2g]\n',smallPriorRotNoise,largePriorRotNoise);
        end
        
        function setConsistentPositionPriorVariancesRFootImuOrigin(obj)
            % you have a multi IMU system. Inevitably one IMU needs to be the origin IMU. It should have a strong prior on position while the others
            %   have a weak prior on position.
            % () first pull out my values
            fv=NonlinearFactorGraphToFactorVector(obj.m_graph);
            priorPose3Factors={}; priorPose3FactorsIdx=[]; priorPose3FactorsKeys={};
            for k=1:length(fv)
                if strcmp(class(fv{k}),'gtsam.PriorFactorPose3')
                    priorPose3Factors={priorPose3Factors{:},fv{k}};
                    priorPose3FactorsIdx=[priorPose3FactorsIdx,k];
                end
            end
            % () what should we consider a large and small variance on orientation?
            largePriorPosNoise=1e1*[1 1 1];
            smallPriorPosNoise=1e-6*[1 1 1];
            %%% hardcoding (bad!) order of these priors.  1: sacrum, 2: rthigh, 3: rshank, 4: rfoot, 5: lthigh, 6: lshank, 7:lfoot
            % () set these to named variables
            sacrumImuPriorPose3FactorOld=priorPose3Factors{1};
            rthighImuPriorPose3FactorOld=priorPose3Factors{2};
            rshankImuPriorPose3FactorOld=priorPose3Factors{3};
            rfootImuPriorPose3FactorOld=priorPose3Factors{4};
            lthighImuPriorPose3FactorOld=priorPose3Factors{5};
            lshankImuPriorPose3FactorOld=priorPose3Factors{6};
            lfootImuPriorPose3FactorOld=priorPose3Factors{7};
            % () pull out old means (gtsam.Pose3)
            sacrumImuPriorPose3FactorOldMean=sacrumImuPriorPose3FactorOld.prior;
            rthighImuPriorPose3FactorOldMean=rthighImuPriorPose3FactorOld.prior;
            rshankImuPriorPose3FactorOldMean=rshankImuPriorPose3FactorOld.prior;
            rfootImuPriorPose3FactorOldMean=rfootImuPriorPose3FactorOld.prior;
            lthighImuPriorPose3FactorOldMean=lthighImuPriorPose3FactorOld.prior;
            lshankImuPriorPose3FactorOldMean=lshankImuPriorPose3FactorOld.prior;
            lfootImuPriorPose3FactorOldMean=lfootImuPriorPose3FactorOld.prior;
            % () pull out old sigmas too (6x1 array)
            sacrumImuPriorPose3FactorOldSigmas=sacrumImuPriorPose3FactorOld.noiseModel.sigmas;
            rthighImuPriorPose3FactorOldSigmas=rthighImuPriorPose3FactorOld.noiseModel.sigmas;
            rshankImuPriorPose3FactorOldSigmas=rshankImuPriorPose3FactorOld.noiseModel.sigmas;
            rfootImuPriorPose3FactorOldSigmas=rfootImuPriorPose3FactorOld.noiseModel.sigmas;
            lthighImuPriorPose3FactorOldSigmas=lthighImuPriorPose3FactorOld.noiseModel.sigmas;
            lshankImuPriorPose3FactorOldSigmas=lshankImuPriorPose3FactorOld.noiseModel.sigmas;
            lfootImuPriorPose3FactorOldSigmas=lfootImuPriorPose3FactorOld.noiseModel.sigmas;
            % () now given the old sigmas what should the new ones be?
            % --> change thigh IMU orientation sigma to the small, and the others to the large. Remember that orientation is the first 3 of the 6x1 pose variance.
            sacrumImuPriorPose3FactorNewSigmas=vertcat(sacrumImuPriorPose3FactorOldSigmas(1:3),largePriorPosNoise(:));
            rthighImuPriorPose3FactorNewSigmas=vertcat(rthighImuPriorPose3FactorOldSigmas(1:3),largePriorPosNoise(:));
            rshankImuPriorPose3FactorNewSigmas=vertcat(rshankImuPriorPose3FactorOldSigmas(1:3),largePriorPosNoise(:));
            rfootImuPriorPose3FactorNewSigmas=vertcat(rfootImuPriorPose3FactorOldSigmas(1:3),smallPriorPosNoise(:));
            lthighImuPriorPose3FactorNewSigmas=vertcat(lthighImuPriorPose3FactorOldSigmas(1:3),largePriorPosNoise(:));
            lshankImuPriorPose3FactorNewSigmas=vertcat(lshankImuPriorPose3FactorOldSigmas(1:3),largePriorPosNoise(:));
            lfootImuPriorPose3FactorNewSigmas=vertcat(lfootImuPriorPose3FactorOldSigmas(1:3),largePriorPosNoise(:));
            % () create new noise models
            sacrumImuPriorPose3ModelNew=gtsam.noiseModel.Diagonal.Sigmas(sacrumImuPriorPose3FactorNewSigmas); % rad, rad, rad, m, m, m
            rthighImuPriorPose3ModelNew=gtsam.noiseModel.Diagonal.Sigmas(rthighImuPriorPose3FactorNewSigmas); % rad, rad, rad, m, m, m
            rshankImuPriorPose3ModelNew=gtsam.noiseModel.Diagonal.Sigmas(rshankImuPriorPose3FactorNewSigmas); % rad, rad, rad, m, m, m
            rfootImuPriorPose3ModelNew=gtsam.noiseModel.Diagonal.Sigmas(rfootImuPriorPose3FactorNewSigmas); % rad, rad, rad, m, m, m
            lthighImuPriorPose3ModelNew=gtsam.noiseModel.Diagonal.Sigmas(lthighImuPriorPose3FactorNewSigmas); % rad, rad, rad, m, m, m
            lshankImuPriorPose3ModelNew=gtsam.noiseModel.Diagonal.Sigmas(lshankImuPriorPose3FactorNewSigmas); % rad, rad, rad, m, m, m
            lfootImuPriorPose3ModelNew=gtsam.noiseModel.Diagonal.Sigmas(lfootImuPriorPose3FactorNewSigmas); % rad, rad, rad, m, m, m
            % () construct new prior factors
            sacrumImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_sacrumPoseVarChar,0),sacrumImuPriorPose3FactorOldMean,sacrumImuPriorPose3ModelNew);
            rthighImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_rthighPoseVarChar,0),rthighImuPriorPose3FactorOldMean,rthighImuPriorPose3ModelNew);
            rshankImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_rshankPoseVarChar,0),rshankImuPriorPose3FactorOldMean,rshankImuPriorPose3ModelNew);
            rfootImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_rfootPoseVarChar,0),rfootImuPriorPose3FactorOldMean,rfootImuPriorPose3ModelNew);
            lthighImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_lthighPoseVarChar,0),lthighImuPriorPose3FactorOldMean,lthighImuPriorPose3ModelNew);
            lshankImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_lshankPoseVarChar,0),lshankImuPriorPose3FactorOldMean,lshankImuPriorPose3ModelNew);
            lfootImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_lfootPoseVarChar,0),lfootImuPriorPose3FactorOldMean,lfootImuPriorPose3ModelNew);
            % () now replace() those old factors with the new one
            obj.m_graph.replace(priorPose3FactorsIdx(1)-1,sacrumImuPriorPose3FactorNew); % sacrum IMU prior
            obj.m_graph.replace(priorPose3FactorsIdx(2)-1,rthighImuPriorPose3FactorNew); % thigh IMU prior
            obj.m_graph.replace(priorPose3FactorsIdx(3)-1,rshankImuPriorPose3FactorNew); % shank IMU prior
            obj.m_graph.replace(priorPose3FactorsIdx(4)-1,rfootImuPriorPose3FactorNew); % foot IMU prior
            obj.m_graph.replace(priorPose3FactorsIdx(5)-1,lthighImuPriorPose3FactorNew); % thigh IMU prior
            obj.m_graph.replace(priorPose3FactorsIdx(6)-1,lshankImuPriorPose3FactorNew); % shank IMU prior
            obj.m_graph.replace(priorPose3FactorsIdx(7)-1,lfootImuPriorPose3FactorNew); % foot IMU prior
            fprintf('\tset consistent prior position sigmas: rfoot IMU small @ [%.2g %.2g %.2g], others large @ [%.2g %.2g %.2g]\n',smallPriorPosNoise,largePriorPosNoise);
        end
        
        function setConsistentPositionPriorMeansRFootImuOrigin(obj)
            % you have a multiple IMU system. it doesn't make sense to set the prior positions on every IMU to zero--that is physically inconsistent.
            % set the rfoot IMU as a prior on position (0,0,0) in the navigation frame.
            % () first pull out my values
            fv=NonlinearFactorGraphToFactorVector(obj.m_graph);
            priorPose3Factors={}; priorPose3FactorsIdx=[]; priorPose3FactorsKeys={};
            for k=1:length(fv)
                if strcmp(class(fv{k}),'gtsam.PriorFactorPose3')
                    priorPose3Factors={priorPose3Factors{:},fv{k}};
                    priorPose3FactorsIdx=[priorPose3FactorsIdx,k];
                    
                end
            end
            %%% hardcoding (bad!) order of these priors.  1: sacrum, 2: rthigh, 3: rshank, 4: rfoot, 5: lthigh, 6: lshank, 7:lfoot
            % () set these to named variables
            sacrumImuPriorPose3FactorOld=priorPose3Factors{1};
            rthighImuPriorPose3FactorOld=priorPose3Factors{2};
            rshankImuPriorPose3FactorOld=priorPose3Factors{3};
            rfootImuPriorPose3FactorOld=priorPose3Factors{4};
            lthighImuPriorPose3FactorOld=priorPose3Factors{5};
            lshankImuPriorPose3FactorOld=priorPose3Factors{6};
            lfootImuPriorPose3FactorOld=priorPose3Factors{7};
            % () pull out old means (gtsam.Pose3)
            sacrumImuPriorPose3FactorOldMean=sacrumImuPriorPose3FactorOld.prior;
            rthighImuPriorPose3FactorOldMean=rthighImuPriorPose3FactorOld.prior;
            rshankImuPriorPose3FactorOldMean=rshankImuPriorPose3FactorOld.prior;
            rfootImuPriorPose3FactorOldMean=rfootImuPriorPose3FactorOld.prior;
            lthighImuPriorPose3FactorOldMean=lthighImuPriorPose3FactorOld.prior;
            lshankImuPriorPose3FactorOldMean=lshankImuPriorPose3FactorOld.prior;
            lfootImuPriorPose3FactorOldMean=lfootImuPriorPose3FactorOld.prior;
            % () now construct the new means (gtsam.Pose3). keep same orientations.
            [sacrum, rthigh, rshank, rfoot, lthigh, lshank, lfoot]=lowerBodyPoseEstimator.standardAnatomicalPosePriorImuPositions();
            sacrumImuPriorPose3FactorNewMean=gtsam.Pose3(sacrumImuPriorPose3FactorOldMean.rotation,gtsam.Point3(sacrum(:)));
            rthighImuPriorPose3FactorNewMean=gtsam.Pose3(rthighImuPriorPose3FactorOldMean.rotation,gtsam.Point3(rthigh(:)));
            rshankImuPriorPose3FactorNewMean=gtsam.Pose3(rshankImuPriorPose3FactorOldMean.rotation,gtsam.Point3(rshank(:)));
            rfootImuPriorPose3FactorNewMean=gtsam.Pose3(rfootImuPriorPose3FactorOldMean.rotation,gtsam.Point3(rfoot(:)));
            lthighImuPriorPose3FactorNewMean=gtsam.Pose3(lthighImuPriorPose3FactorOldMean.rotation,gtsam.Point3(lthigh(:)));
            lshankImuPriorPose3FactorNewMean=gtsam.Pose3(lshankImuPriorPose3FactorOldMean.rotation,gtsam.Point3(lshank(:)));
            lfootImuPriorPose3FactorNewMean=gtsam.Pose3(lfootImuPriorPose3FactorOldMean.rotation,gtsam.Point3(lfoot(:)));
            % () construct new prior factors with new means (keeping same old noise models!)
            sacrumImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_sacrumPoseVarChar,0),sacrumImuPriorPose3FactorNewMean,sacrumImuPriorPose3FactorOld.noiseModel);
            rthighImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_rthighPoseVarChar,0),rthighImuPriorPose3FactorNewMean,rthighImuPriorPose3FactorOld.noiseModel);
            rshankImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_rshankPoseVarChar,0),rshankImuPriorPose3FactorNewMean,rshankImuPriorPose3FactorOld.noiseModel);
            rfootImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_rfootPoseVarChar,0),rfootImuPriorPose3FactorNewMean,rfootImuPriorPose3FactorOld.noiseModel);
            lthighImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_lthighPoseVarChar,0),lthighImuPriorPose3FactorNewMean,lthighImuPriorPose3FactorOld.noiseModel);
            lshankImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_lshankPoseVarChar,0),lshankImuPriorPose3FactorNewMean,lshankImuPriorPose3FactorOld.noiseModel);
            lfootImuPriorPose3FactorNew=gtsam.PriorFactorPose3(gtsam.symbol(obj.m_lfootPoseVarChar,0),lfootImuPriorPose3FactorNewMean,lfootImuPriorPose3FactorOld.noiseModel);
            % () now replace() those old factors with the new one
            obj.m_graph.replace(priorPose3FactorsIdx(1)-1,sacrumImuPriorPose3FactorNew); % sacrum IMU prior
            obj.m_graph.replace(priorPose3FactorsIdx(2)-1,rthighImuPriorPose3FactorNew); % thigh IMU prior
            obj.m_graph.replace(priorPose3FactorsIdx(3)-1,rshankImuPriorPose3FactorNew); % shank IMU prior
            obj.m_graph.replace(priorPose3FactorsIdx(4)-1,rfootImuPriorPose3FactorNew); % foot IMU prior
            obj.m_graph.replace(priorPose3FactorsIdx(5)-1,lthighImuPriorPose3FactorNew); % thigh IMU prior
            obj.m_graph.replace(priorPose3FactorsIdx(6)-1,lshankImuPriorPose3FactorNew); % shank IMU prior
            obj.m_graph.replace(priorPose3FactorsIdx(7)-1,lfootImuPriorPose3FactorNew); % foot IMU prior
        end
        
        function printPriorInformation(obj)
            % method to print to console information about the priors in the graph
            fv=NonlinearFactorGraphToFactorVector(obj.m_graph);
            priorPose3Factors={}; priorPose3FactorsIdx=[]; priorPose3FactorsKeys={};
            for k=1:length(fv)
                if strcmp(class(fv{k}),'gtsam.PriorFactorPose3')
                    priorPose3Factors={priorPose3Factors{:},fv{k}};
                    priorPose3FactorsIdx=[priorPose3FactorsIdx,k];
                end
            end
            % () set these to named variables
            sacrumImuPriorPose3Factor=priorPose3Factors{1};
            rthighImuPriorPose3Factor=priorPose3Factors{2};
            rshankImuPriorPose3Factor=priorPose3Factors{3};
            rfootImuPriorPose3Factor=priorPose3Factors{4};
            lthighImuPriorPose3Factor=priorPose3Factors{5};
            lshankImuPriorPose3Factor=priorPose3Factors{6};
            lfootImuPriorPose3Factor=priorPose3Factors{7};
            % () pull out  means (gtsam.Pose3)
            sacrumImuPriorPose3FactorMean=sacrumImuPriorPose3Factor.prior;
            rthighImuPriorPose3FactorMean=rthighImuPriorPose3Factor.prior;
            rshankImuPriorPose3FactorMean=rshankImuPriorPose3Factor.prior;
            rfootImuPriorPose3FactorMean=rfootImuPriorPose3Factor.prior;
            lthighImuPriorPose3FactorMean=lthighImuPriorPose3Factor.prior;
            lshankImuPriorPose3FactorMean=lshankImuPriorPose3Factor.prior;
            lfootImuPriorPose3FactorMean=lfootImuPriorPose3Factor.prior;
            % () pull out  sigmas too (6x1 array)
            sacrumImuPriorPose3FactorSigmas=sacrumImuPriorPose3Factor.noiseModel.sigmas;
            rthighImuPriorPose3FactorSigmas=rthighImuPriorPose3Factor.noiseModel.sigmas;
            rshankImuPriorPose3FactorSigmas=rshankImuPriorPose3Factor.noiseModel.sigmas;
            rfootImuPriorPose3FactorSigmas=rfootImuPriorPose3Factor.noiseModel.sigmas;
            lthighImuPriorPose3FactorSigmas=lthighImuPriorPose3Factor.noiseModel.sigmas;
            lshankImuPriorPose3FactorSigmas=lshankImuPriorPose3Factor.noiseModel.sigmas;
            lfootImuPriorPose3FactorSigmas=lfootImuPriorPose3Factor.noiseModel.sigmas;
            fprintf('\tPriors used in model:\n');
            fprintf('\t\tSacrum Imu Pose3: q=[%.3f %.3f %.3f %.3f], p=[%.3f %.3f %.3f], sigmas (rot, pos)=[%.3f %.3f %.3f %.3f %.3f %.3f]\n',sacrumImuPriorPose3FactorMean.rotation().quaternion()',sacrumImuPriorPose3FactorMean.translation().vector()',sacrumImuPriorPose3FactorSigmas(:)');
            fprintf('\t\tRThigh Imu Pose3: q=[%.3f %.3f %.3f %.3f], p=[%.3f %.3f %.3f], sigmas (rot, pos)=[%.3f %.3f %.3f %.3f %.3f %.3f]\n',rthighImuPriorPose3FactorMean.rotation().quaternion()',rthighImuPriorPose3FactorMean.translation().vector()',rthighImuPriorPose3FactorSigmas(:)');
            fprintf('\t\tRShank Imu Pose3: q=[%.3f %.3f %.3f %.3f], p=[%.3f %.3f %.3f], sigmas (rot, pos)=[%.3f %.3f %.3f %.3f %.3f %.3f]\n',rshankImuPriorPose3FactorMean.rotation().quaternion()',rshankImuPriorPose3FactorMean.translation().vector()',rshankImuPriorPose3FactorSigmas(:)');
            fprintf('\t\tRFoot Imu Pose3: q=[%.3f %.3f %.3f %.3f], p=[%.3f %.3f %.3f], sigmas (rot, pos)=[%.3f %.3f %.3f %.3f %.3f %.3f]\n',rfootImuPriorPose3FactorMean.rotation().quaternion()',rfootImuPriorPose3FactorMean.translation().vector()',rfootImuPriorPose3FactorSigmas(:)');
            fprintf('\t\tLThigh Imu Pose3: q=[%.3f %.3f %.3f %.3f], p=[%.3f %.3f %.3f], sigmas (rot, pos)=[%.3f %.3f %.3f %.3f %.3f %.3f]\n',lthighImuPriorPose3FactorMean.rotation().quaternion()',lthighImuPriorPose3FactorMean.translation().vector()',lthighImuPriorPose3FactorSigmas(:)');
            fprintf('\t\tLShank Imu Pose3: q=[%.3f %.3f %.3f %.3f], p=[%.3f %.3f %.3f], sigmas (rot, pos)=[%.3f %.3f %.3f %.3f %.3f %.3f]\n',lshankImuPriorPose3FactorMean.rotation().quaternion()',lshankImuPriorPose3FactorMean.translation().vector()',lshankImuPriorPose3FactorSigmas(:)');
            fprintf('\t\tLFoot Imu Pose3: q=[%.3f %.3f %.3f %.3f], p=[%.3f %.3f %.3f], sigmas (rot, pos)=[%.3f %.3f %.3f %.3f %.3f %.3f]\n',lfootImuPriorPose3FactorMean.rotation().quaternion()',lfootImuPriorPose3FactorMean.translation().vector()',lfootImuPriorPose3FactorSigmas(:)');
        end
        
        %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++%
        %---- Methods for getting marginals and setting variances of variables ----%
        %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++%
        function setMarginals(obj)
            % NOTE: in this object, we choose to only save the principal (diagonal) variances for the time-series variables, but we store the full marginal covariance for the static states
            %   this is just for space efficiency. This can be changed if needed. (it's only the principal variances I tend to look at)
            marginalsTic=tic; fprintf('attempting to set marginals... ');
            assert(~isempty(obj.m_rthighImuOrientation),'estimated orientation unset. have you run an optimizer?');
            try
                obj.m_marginals=gtsam.Marginals(obj.m_graph,obj.m_estimate);
                fprintf('\tset global marginals!\n');
            catch mException
                if isIndeterminantLinearSystemException(mException)
                    fprintf('\texception in setMarginals(): indeterminant system, could not get marginals\n');
                else
                    error('what kind of error is this?');
                end
            end
            if ~isempty(obj.m_marginals) % you got the marginals, now set principal variances of estimated variables
                try
                    fprintf('\tattempting to pull out principal variances... completed: ');
                    % sacrum IMU states
                    t=tic;
                    [obj.m_sacrumImuOrientationPrincipalVariance,obj.m_sacrumImuPositionPrincipalVariance,obj.m_sacrumImuVelocityPrincipalVariance,obj.m_sacrumImuGyroBiasPrincipalVariance,...
                        obj.m_sacrumImuAccelBiasPrincipalVariance]=parseImuStatesPrincipalVariance(obj.m_sacrumImuPoseProblem.m_poseKeys,obj.m_sacrumImuPoseProblem.m_velKeys,obj.m_sacrumImuPoseProblem.m_imuBiasKeys,obj.m_marginals);
                    fprintf('Sacrum IMU (%.2f sec), ',toc(t));
                    % rthigh IMU states
                    t=tic;
                    [obj.m_rthighImuOrientationPrincipalVariance,obj.m_rthighImuPositionPrincipalVariance,obj.m_rthighImuVelocityPrincipalVariance,obj.m_rthighImuGyroBiasPrincipalVariance,...
                        obj.m_rthighImuAccelBiasPrincipalVariance]=parseImuStatesPrincipalVariance(obj.m_rthighImuPoseProblem.m_poseKeys,obj.m_rthighImuPoseProblem.m_velKeys,obj.m_rthighImuPoseProblem.m_imuBiasKeys,obj.m_marginals);
                    fprintf('RThigh IMU (%.2f sec), ',toc(t));
                    % rshank IMU states
                    t=tic;
                    [obj.m_rshankImuOrientationPrincipalVariance,obj.m_rshankImuPositionPrincipalVariance,obj.m_rshankImuVelocityPrincipalVariance,obj.m_rshankImuGyroBiasPrincipalVariance,...
                        obj.m_rshankImuAccelBiasPrincipalVariance]=parseImuStatesPrincipalVariance(obj.m_rshankImuPoseProblem.m_poseKeys,obj.m_rshankImuPoseProblem.m_velKeys,obj.m_rshankImuPoseProblem.m_imuBiasKeys,obj.m_marginals);
                    fprintf('RShank IMU (%.2f sec), ',toc(t));
                    % rfoot IMU states
                    t=tic;
                    [obj.m_rfootImuOrientationPrincipalVariance,obj.m_rfootImuPositionPrincipalVariance,obj.m_rfootImuVelocityPrincipalVariance,obj.m_rfootImuGyroBiasPrincipalVariance,...
                        obj.m_rfootImuAccelBiasPrincipalVariance]=parseImuStatesPrincipalVariance(obj.m_rfootImuPoseProblem.m_poseKeys,obj.m_rfootImuPoseProblem.m_velKeys,obj.m_rfootImuPoseProblem.m_imuBiasKeys,obj.m_marginals);
                    fprintf('RFoot IMU (%.2f sec), ',toc(t));
                    % lthigh IMU states
                    t=tic;
                    [obj.m_lthighImuOrientationPrincipalVariance,obj.m_lthighImuPositionPrincipalVariance,obj.m_lthighImuVelocityPrincipalVariance,obj.m_lthighImuGyroBiasPrincipalVariance,...
                        obj.m_lthighImuAccelBiasPrincipalVariance]=parseImuStatesPrincipalVariance(obj.m_lthighImuPoseProblem.m_poseKeys,obj.m_lthighImuPoseProblem.m_velKeys,obj.m_lthighImuPoseProblem.m_imuBiasKeys,obj.m_marginals);
                    fprintf('LThigh IMU (%.2f sec), ',toc(t));
                    % lshank IMU states
                    t=tic;
                    [obj.m_lshankImuOrientationPrincipalVariance,obj.m_lshankImuPositionPrincipalVariance,obj.m_lshankImuVelocityPrincipalVariance,obj.m_lshankImuGyroBiasPrincipalVariance,...
                        obj.m_lshankImuAccelBiasPrincipalVariance]=parseImuStatesPrincipalVariance(obj.m_lshankImuPoseProblem.m_poseKeys,obj.m_lshankImuPoseProblem.m_velKeys,obj.m_lshankImuPoseProblem.m_imuBiasKeys,obj.m_marginals);
                    fprintf('LShank IMU (%.2f sec), ',toc(t));
                    % lfoot IMU states
                    t=tic;
                    [obj.m_lfootImuOrientationPrincipalVariance,obj.m_lfootImuPositionPrincipalVariance,obj.m_lfootImuVelocityPrincipalVariance,obj.m_lfootImuGyroBiasPrincipalVariance,...
                        obj.m_lfootImuAccelBiasPrincipalVariance]=parseImuStatesPrincipalVariance(obj.m_lfootImuPoseProblem.m_poseKeys,obj.m_lfootImuPoseProblem.m_velKeys,obj.m_lfootImuPoseProblem.m_imuBiasKeys,obj.m_marginals);
                    fprintf('LFoot IMU (%.2f sec), ',toc(t));
                    % Now all the static variables
                    t=tic; % now for all of the static variables principal variances
                    [~,obj.m_rkneeAxisThighCovariance]=computePrincipalVariance(obj.m_rkneeAxisThighKey,obj.m_marginals,2);
                    [~,obj.m_rkneeAxisShankCovariance]=computePrincipalVariance(obj.m_rkneeAxisShankKey,obj.m_marginals,2);
                    [~,obj.m_lkneeAxisThighCovariance]=computePrincipalVariance(obj.m_lkneeAxisThighKey,obj.m_marginals,2);
                    [~,obj.m_lkneeAxisShankCovariance]=computePrincipalVariance(obj.m_lkneeAxisShankKey,obj.m_marginals,2);
                    [~,obj.m_sacrumImuToRHipCtrCovariance]=computePrincipalVariance(obj.m_sacrumImuToRHipCtrKey,obj.m_marginals,3);
                    [~,obj.m_rthighImuToHipCtrCovariance]=computePrincipalVariance(obj.m_rthighImuToHipCtrKey,obj.m_marginals,3);
                    [~,obj.m_rthighImuToKneeCtrCovariance]=computePrincipalVariance(obj.m_rthighImuToKneeCtrKey,obj.m_marginals,3);
                    [~,obj.m_rshankImuToKneeCtrCovariance]=computePrincipalVariance(obj.m_rshankImuToKneeCtrKey,obj.m_marginals,3);
                    [~,obj.m_rshankImuToAnkleCtrCovariance]=computePrincipalVariance(obj.m_rshankImuToAnkleCtrKey,obj.m_marginals,3);
                    [~,obj.m_rfootImuToAnkleCtrCovariance]=computePrincipalVariance(obj.m_rfootImuToAnkleCtrKey,obj.m_marginals,3);
                    [~,obj.m_sacrumImuToLHipCtrCovariance]=computePrincipalVariance(obj.m_sacrumImuToLHipCtrKey,obj.m_marginals,3);
                    [~,obj.m_lthighImuToHipCtrCovariance]=computePrincipalVariance(obj.m_lthighImuToHipCtrKey,obj.m_marginals,3);
                    [~,obj.m_lthighImuToKneeCtrCovariance]=computePrincipalVariance(obj.m_lthighImuToKneeCtrKey,obj.m_marginals,3);
                    [~,obj.m_lshankImuToKneeCtrCovariance]=computePrincipalVariance(obj.m_lshankImuToKneeCtrKey,obj.m_marginals,3);
                    [~,obj.m_lshankImuToAnkleCtrCovariance]=computePrincipalVariance(obj.m_lshankImuToAnkleCtrKey,obj.m_marginals,3);
                    [~,obj.m_lfootImuToAnkleCtrCovariance]=computePrincipalVariance(obj.m_lfootImuToAnkleCtrKey,obj.m_marginals,3);
                    fprintf(' all static variables (%.2f sec).',toc(t));
                    % Optionally, angular velocities principal variance
                    t=tic;
                    [obj.m_sacrumImuAngVelPrincipalVariance]=computePrincipalVariance(obj.m_sacrumImuAngVelKeys,obj.m_marginals,3);
                    [obj.m_rthighImuAngVelPrincipalVariance]=computePrincipalVariance(obj.m_rthighImuAngVelKeys,obj.m_marginals,3);
                    [obj.m_rshankImuAngVelPrincipalVariance]=computePrincipalVariance(obj.m_rshankImuAngVelKeys,obj.m_marginals,3);
                    [obj.m_rfootImuAngVelPrincipalVariance]=computePrincipalVariance(obj.m_rfootImuAngVelKeys,obj.m_marginals,3);
                    [obj.m_lthighImuAngVelPrincipalVariance]=computePrincipalVariance(obj.m_lthighImuAngVelKeys,obj.m_marginals,3);
                    [obj.m_lshankImuAngVelPrincipalVariance]=computePrincipalVariance(obj.m_lshankImuAngVelKeys,obj.m_marginals,3);
                    [obj.m_lfootImuAngVelPrincipalVariance]=computePrincipalVariance(obj.m_lfootImuAngVelKeys,obj.m_marginals,3);
                    fprintf(' all angular velocities (%.2f sec).\n',toc(t));
                catch mException
                    fprintf('failed when setting marginal covariances.\n'); % no need to pull out the error message here; it's already in the individual functions above
                    return
                end
                % if you get here, you succeeded!
                fprintf('\tfully set marginals and computed principal variances! (total time = %.3f seconds)\n',toc(marginalsTic));
                obj.m_allMarginalsSuccessful=1;
            end
        end % setMarginals(obj)
        
        %++++++++++++++++++++++++++++++++++++++++++++%
        %---- Methods for printing data to files ----%
        %++++++++++++++++++++++++++++++++++++++++++++%
        function saveResultsToH5(obj,filename)
            % take all of the results and print them to an .h5 file
            % () check: is this filename valid?
            % check that it is indeed an .h5 file
            % does the parent directory exist?
            startTic=tic;
            filenamesplit=strsplit(filename,filesep);
            if isunix; parentdir=fullfile(filesep,filenamesplit{1:end-1}); else; parentdir=fullfile(filenamesplit{1:end-1}); end
            if exist(parentdir,'dir')~=7; error('parent directory %s does not exist!',parentdir); end
            if exist(filename,'file')==2 % change name to a new file.
                filename=filenameAppender(filename,1);
                warning('file already exists. changing filename to: %s',filename);
            end
            % () write the file!
            fprintf('writing results to .h5 file: %s... ',filename);
            % Time
            H5EasyAdd(filename,'/Time',obj.m_time(:));
            % write estimated imu states to file: Sacrum IMU
            H5EasyAdd(filename,'/Estimated/R_SacrumImu_to_N',Rot3ArrayToMatrices(obj.m_sacrumImuOrientation));
            H5EasyAdd(filename,'/Estimated/q_SacrumImu_to_N',Rot3ArrayToQuaternions(obj.m_sacrumImuOrientation));
            H5EasyAdd(filename,'/Estimated/p_SacrumImu',Point3ArrayToMatrix(obj.m_sacrumImuPosition));
            H5EasyAdd(filename,'/Estimated/v_SacrumImu',obj.m_sacrumImuVelocity);
            H5EasyAdd(filename,'/Estimated/accelbias_SacrumImu',obj.m_sacrumImuAccelBias);
            H5EasyAdd(filename,'/Estimated/gyrobias_SacrumImu',obj.m_sacrumImuGyroBias);
            % RThigh IMU
            H5EasyAdd(filename,'/Estimated/R_RThighImu_to_N',Rot3ArrayToMatrices(obj.m_rthighImuOrientation));
            H5EasyAdd(filename,'/Estimated/q_RThighImu_to_N',Rot3ArrayToQuaternions(obj.m_rthighImuOrientation));
            H5EasyAdd(filename,'/Estimated/p_RThighImu',Point3ArrayToMatrix(obj.m_rthighImuPosition));
            H5EasyAdd(filename,'/Estimated/v_RThighImu',obj.m_rthighImuVelocity);
            H5EasyAdd(filename,'/Estimated/accelbias_RThighImu',obj.m_rthighImuAccelBias);
            H5EasyAdd(filename,'/Estimated/gyrobias_RThighImu',obj.m_rthighImuGyroBias);
            % RShank IMU
            H5EasyAdd(filename,'/Estimated/R_RShankImu_to_N',Rot3ArrayToMatrices(obj.m_rshankImuOrientation));
            H5EasyAdd(filename,'/Estimated/q_RShankImu_to_N',Rot3ArrayToQuaternions(obj.m_rshankImuOrientation));
            H5EasyAdd(filename,'/Estimated/p_RShankImu',Point3ArrayToMatrix(obj.m_rshankImuPosition));
            H5EasyAdd(filename,'/Estimated/v_RShankImu',obj.m_rshankImuVelocity);
            H5EasyAdd(filename,'/Estimated/accelbias_RShankImu',obj.m_rshankImuAccelBias);
            H5EasyAdd(filename,'/Estimated/gyrobias_RShankImu',obj.m_rshankImuGyroBias);
            % RFoot IMU
            H5EasyAdd(filename,'/Estimated/R_RFootImu_to_N',Rot3ArrayToMatrices(obj.m_rfootImuOrientation));
            H5EasyAdd(filename,'/Estimated/q_RFootImu_to_N',Rot3ArrayToQuaternions(obj.m_rfootImuOrientation));
            H5EasyAdd(filename,'/Estimated/p_RFootImu',Point3ArrayToMatrix(obj.m_rfootImuPosition));
            H5EasyAdd(filename,'/Estimated/v_RFootImu',obj.m_rfootImuVelocity);
            H5EasyAdd(filename,'/Estimated/accelbias_RFootImu',obj.m_rfootImuAccelBias);
            H5EasyAdd(filename,'/Estimated/gyrobias_RFootImu',obj.m_rfootImuGyroBias);
            % LThigh IMU
            H5EasyAdd(filename,'/Estimated/R_LThighImu_to_N',Rot3ArrayToMatrices(obj.m_lthighImuOrientation));
            H5EasyAdd(filename,'/Estimated/q_LThighImu_to_N',Rot3ArrayToQuaternions(obj.m_lthighImuOrientation));
            H5EasyAdd(filename,'/Estimated/p_LThighImu',Point3ArrayToMatrix(obj.m_lthighImuPosition));
            H5EasyAdd(filename,'/Estimated/v_LThighImu',obj.m_lthighImuVelocity);
            H5EasyAdd(filename,'/Estimated/accelbias_LThighImu',obj.m_lthighImuAccelBias);
            H5EasyAdd(filename,'/Estimated/gyrobias_LThighImu',obj.m_lthighImuGyroBias);
            % LShank IMU
            H5EasyAdd(filename,'/Estimated/R_LShankImu_to_N',Rot3ArrayToMatrices(obj.m_lshankImuOrientation));
            H5EasyAdd(filename,'/Estimated/q_LShankImu_to_N',Rot3ArrayToQuaternions(obj.m_lshankImuOrientation));
            H5EasyAdd(filename,'/Estimated/p_LShankImu',Point3ArrayToMatrix(obj.m_lshankImuPosition));
            H5EasyAdd(filename,'/Estimated/v_LShankImu',obj.m_lshankImuVelocity);
            H5EasyAdd(filename,'/Estimated/accelbias_LShankImu',obj.m_lshankImuAccelBias);
            H5EasyAdd(filename,'/Estimated/gyrobias_LShankImu',obj.m_lshankImuGyroBias);
            % LFoot IMU
            H5EasyAdd(filename,'/Estimated/R_LFootImu_to_N',Rot3ArrayToMatrices(obj.m_lfootImuOrientation));
            H5EasyAdd(filename,'/Estimated/q_LFootImu_to_N',Rot3ArrayToQuaternions(obj.m_lfootImuOrientation));
            H5EasyAdd(filename,'/Estimated/p_LFootImu',Point3ArrayToMatrix(obj.m_lfootImuPosition));
            H5EasyAdd(filename,'/Estimated/v_LFootImu',obj.m_lfootImuVelocity);
            H5EasyAdd(filename,'/Estimated/accelbias_LFootImu',obj.m_lfootImuAccelBias);
            H5EasyAdd(filename,'/Estimated/gyrobias_LFootImu',obj.m_lfootImuGyroBias);
            % Knee axes
            H5EasyAdd(filename,'/Estimated/RKneeAxis_RThigh',obj.m_rkneeAxisThigh.point3().vector());
            H5EasyAdd(filename,'/Estimated/RKneeAxis_RShank',obj.m_rkneeAxisShank.point3().vector());
            H5EasyAdd(filename,'/Estimated/LKneeAxis_LThigh',obj.m_lkneeAxisThigh.point3().vector());
            H5EasyAdd(filename,'/Estimated/LKneeAxis_LShank',obj.m_lkneeAxisShank.point3().vector());
            % static vectors to joint rotation centers
            H5EasyAdd(filename,'/Estimated/SacrumImuToRHipCtr',obj.m_sacrumImuToRHipCtr.vector());
            H5EasyAdd(filename,'/Estimated/RThighImuToRHipCtr',obj.m_rthighImuToHipCtr.vector());
            H5EasyAdd(filename,'/Estimated/RThighImuToRKneeCtr',obj.m_rthighImuToKneeCtr.vector());
            H5EasyAdd(filename,'/Estimated/RShankImuToRKneeCtr',obj.m_rshankImuToKneeCtr.vector());
            H5EasyAdd(filename,'/Estimated/RShankImuToRAnkleCtr',obj.m_rshankImuToAnkleCtr.vector());
            H5EasyAdd(filename,'/Estimated/RFootImuToRAnkleCtr',obj.m_rfootImuToAnkleCtr.vector());
            H5EasyAdd(filename,'/Estimated/SacrumImuToLHipCtr',obj.m_sacrumImuToLHipCtr.vector());
            H5EasyAdd(filename,'/Estimated/LThighImuToLHipCtr',obj.m_lthighImuToHipCtr.vector());
            H5EasyAdd(filename,'/Estimated/LThighImuToLKneeCtr',obj.m_lthighImuToKneeCtr.vector());
            H5EasyAdd(filename,'/Estimated/LShankImuToLKneeCtr',obj.m_lshankImuToKneeCtr.vector());
            H5EasyAdd(filename,'/Estimated/LShankImuToLAnkleCtr',obj.m_lshankImuToAnkleCtr.vector());
            H5EasyAdd(filename,'/Estimated/LFootImuToLAnkleCtr',obj.m_lfootImuToAnkleCtr.vector());
            % if you estimated angular velocity, add that
            H5EasyAdd(filename,'/Estimated/SacrumImuAngVel',obj.m_sacrumImuAngVel);
            H5EasyAdd(filename,'/Estimated/RThighImuAngVel',obj.m_rthighImuAngVel);
            H5EasyAdd(filename,'/Estimated/RShankImuAngVel',obj.m_rshankImuAngVel);
            H5EasyAdd(filename,'/Estimated/RFootImuAngVel',obj.m_rfootImuAngVel);
            H5EasyAdd(filename,'/Estimated/LThighImuAngVel',obj.m_lthighImuAngVel);
            H5EasyAdd(filename,'/Estimated/LShankImuAngVel',obj.m_lshankImuAngVel);
            H5EasyAdd(filename,'/Estimated/LFootImuAngVel',obj.m_lfootImuAngVel);
            if obj.m_assumeHipHinge % add hip hinge axis
                try
                    H5EasyAdd(filename,'/Estimated/HipAxis_Sacrum',obj.m_hipAxisSacrum.point3().vector());
                catch
                end
            end
            if obj.m_assumeAnkleHinge % add ankle hinge axis
                try
                    H5EasyAdd(filename,'/Estimated/RAnkleAxis_Foot',obj.m_rankleAxisFoot.point3().vector());
                    H5EasyAdd(filename,'/Estimated/LAnkleAxis_Foot',obj.m_lankleAxisFoot.point3().vector());
                catch
                end
            end
            % write derived quantities to file: segment coordinate systems (relative to IMUs)
            H5EasyAdd(filename,'/Derived/R_SacrumImu_to_Pelvis',obj.get_R_SacrumImu_to_Pelvis);
            H5EasyAdd(filename,'/Derived/R_RThighImu_to_RFemur',obj.get_R_RThighImu_to_RFemur);
            H5EasyAdd(filename,'/Derived/R_RShankImu_to_RTibia',obj.get_R_RShankImu_to_RTibia);
            H5EasyAdd(filename,'/Derived/R_LThighImu_to_LFemur',obj.get_R_LThighImu_to_LFemur);
            H5EasyAdd(filename,'/Derived/R_LShankImu_to_LTibia',obj.get_R_LShankImu_to_LTibia);
            % segment coordinate systems (relative to N)
            H5EasyAdd(filename,'/Derived/R_Pelvis_to_N',obj.m_R_Pelvis_to_N);
            H5EasyAdd(filename,'/Derived/R_RFemur_to_N',obj.m_R_RFemur_to_N);
            H5EasyAdd(filename,'/Derived/R_LFemur_to_N',obj.m_R_LFemur_to_N);
            H5EasyAdd(filename,'/Derived/R_RTibia_to_N',obj.m_R_RTibia_to_N);
            H5EasyAdd(filename,'/Derived/R_LTibia_to_N',obj.m_R_LTibia_to_N);
            % segment lengths
            estRThighLength=sqrt(sum([obj.m_rthighImuToHipCtr.vector()-obj.m_rthighImuToKneeCtr.vector()].^2,1));
            estRShankLength=sqrt(sum([obj.m_rshankImuToKneeCtr.vector()-obj.m_rshankImuToAnkleCtr.vector()].^2,1));
            estLThighLength=sqrt(sum([obj.m_lthighImuToHipCtr.vector()-obj.m_lthighImuToKneeCtr.vector()].^2,1));
            estLShankLength=sqrt(sum([obj.m_lshankImuToKneeCtr.vector()-obj.m_lshankImuToAnkleCtr.vector()].^2,1));
            estPelvicWidth=sqrt(sum([obj.m_sacrumImuToRHipCtr.vector()-obj.m_sacrumImuToLHipCtr.vector()].^2,1));
            H5EasyAdd(filename,'/Derived/PelvicWidth',estPelvicWidth);
            H5EasyAdd(filename,'/Derived/RFemurLength',estRThighLength);
            H5EasyAdd(filename,'/Derived/RTibiaLength',estRShankLength);
            H5EasyAdd(filename,'/Derived/LFemurLength',estLThighLength);
            H5EasyAdd(filename,'/Derived/LTibiaLength',estLShankLength);
            % joint angles
            H5EasyAdd(filename,'/Derived/RKneeAngles_FlexEx',obj.m_rkneeFlexExAng);
            H5EasyAdd(filename,'/Derived/RKneeAngles_IntExtRot',obj.m_rkneeIntExtRotAng);
            H5EasyAdd(filename,'/Derived/RKneeAngles_AbAd',obj.m_rkneeAbAdAng);
            H5EasyAdd(filename,'/Derived/LKneeAngles_FlexEx',obj.m_lkneeFlexExAng);
            H5EasyAdd(filename,'/Derived/LKneeAngles_IntExtRot',obj.m_lkneeIntExtRotAng);
            H5EasyAdd(filename,'/Derived/LKneeAngles_AbAd',obj.m_lkneeAbAdAng);
            H5EasyAdd(filename,'/Derived/RHipAngles_FlexEx',obj.m_rhipFlexExAng);
            H5EasyAdd(filename,'/Derived/RHipAngles_IntExtRot',obj.m_rhipIntExtRotAng);
            H5EasyAdd(filename,'/Derived/RHipAngles_AbAd',obj.m_rhipAbAdAng);
            H5EasyAdd(filename,'/Derived/LHipAngles_FlexEx',obj.m_lhipFlexExAng);
            H5EasyAdd(filename,'/Derived/LHipAngles_IntExtRot',obj.m_lhipIntExtRotAng);
            H5EasyAdd(filename,'/Derived/LHipAngles_AbAd',obj.m_lhipAbAdAng);
            % internal precession angles
            try
                H5EasyAdd(filename,'/Derived/sacrumImuInternalPrecessionAngle',obj.m_sacrumImuInternalPrecessionAng);
                H5EasyAdd(filename,'/Derived/rthighImuInternalPrecessionAngle',obj.m_rthighImuInternalPrecessionAng);
                H5EasyAdd(filename,'/Derived/rshankImuInternalPrecessionAngle',obj.m_rshankImuInternalPrecessionAng);
                H5EasyAdd(filename,'/Derived/rfootImuInternalPrecessionAngle',obj.m_rfootImuInternalPrecessionAng);
                H5EasyAdd(filename,'/Derived/lthighImuInternalPrecessionAngle',obj.m_lthighImuInternalPrecessionAng);
                H5EasyAdd(filename,'/Derived/lshankImuInternalPrecessionAngle',obj.m_lshankImuInternalPrecessionAng);
                H5EasyAdd(filename,'/Derived/lfootImuInternalPrecessionAngle',obj.m_lfootImuInternalPrecessionAng);
            catch
            end
            % optionally, add (co)variance information
            if obj.m_allMarginalsSuccessful
                H5EasyAdd(filename,'/CovarianceInfo/sacrumImuOrientationPrincipalVariance',obj.m_sacrumImuOrientationPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/sacrumImuPositionPrincipalVariance',obj.m_sacrumImuPositionPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/sacrumImuVelocityPrincipalVariance',obj.m_sacrumImuVelocityPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/sacrumImuGyroBiasPrincipalVariance',obj.m_sacrumImuGyroBiasPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/sacrumImuAccelBiasPrincipalVariance',obj.m_sacrumImuAccelBiasPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/rthighImuOrientationPrincipalVariance',obj.m_rthighImuOrientationPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/rthighImuPositionPrincipalVariance',obj.m_rthighImuPositionPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/rthighImuVelocityPrincipalVariance',obj.m_rthighImuVelocityPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/rthighImuGyroBiasPrincipalVariance',obj.m_rthighImuGyroBiasPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/rthighImuAccelBiasPrincipalVariance',obj.m_rthighImuAccelBiasPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/rshankImuOrientationPrincipalVariance',obj.m_rshankImuOrientationPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/rshankImuPositionPrincipalVariance',obj.m_rshankImuPositionPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/rshankImuVelocityPrincipalVariance',obj.m_rshankImuVelocityPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/rshankImuGyroBiasPrincipalVariance',obj.m_rshankImuGyroBiasPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/rshankImuAccelBiasPrincipalVariance',obj.m_rshankImuAccelBiasPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/rfootImuOrientationPrincipalVariance',obj.m_rfootImuOrientationPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/rfootImuPositionPrincipalVariance',obj.m_rfootImuPositionPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/rfootImuVelocityPrincipalVariance',obj.m_rfootImuVelocityPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/rfootImuGyroBiasPrincipalVariance',obj.m_rfootImuGyroBiasPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/rfootImuAccelBiasPrincipalVariance',obj.m_rfootImuAccelBiasPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/lthighImuOrientationPrincipalVariance',obj.m_lthighImuOrientationPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/lthighImuPositionPrincipalVariance',obj.m_lthighImuPositionPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/lthighImuVelocityPrincipalVariance',obj.m_lthighImuVelocityPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/lthighImuGyroBiasPrincipalVariance',obj.m_lthighImuGyroBiasPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/lthighImuAccelBiasPrincipalVariance',obj.m_lthighImuAccelBiasPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/lshankImuOrientationPrincipalVariance',obj.m_lshankImuOrientationPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/lshankImuPositionPrincipalVariance',obj.m_lshankImuPositionPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/lshankImuVelocityPrincipalVariance',obj.m_lshankImuVelocityPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/lshankImuGyroBiasPrincipalVariance',obj.m_lshankImuGyroBiasPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/lshankImuAccelBiasPrincipalVariance',obj.m_lshankImuAccelBiasPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/lfootImuOrientationPrincipalVariance',obj.m_lfootImuOrientationPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/lfootImuPositionPrincipalVariance',obj.m_lfootImuPositionPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/lfootImuVelocityPrincipalVariance',obj.m_lfootImuVelocityPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/lfootImuGyroBiasPrincipalVariance',obj.m_lfootImuGyroBiasPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/lfootImuAccelBiasPrincipalVariance',obj.m_lfootImuAccelBiasPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/rkneeAxisThighCovariance',obj.m_rkneeAxisThighCovariance);
                H5EasyAdd(filename,'/CovarianceInfo/rkneeAxisShankCovariance',obj.m_rkneeAxisShankCovariance);
                H5EasyAdd(filename,'/CovarianceInfo/lkneeAxisThighCovariance',obj.m_lkneeAxisThighCovariance);
                H5EasyAdd(filename,'/CovarianceInfo/lkneeAxisShankCovariance',obj.m_lkneeAxisShankCovariance);
                H5EasyAdd(filename,'/CovarianceInfo/SacrumImuToRHipCtrCovariance',obj.m_sacrumImuToRHipCtrCovariance);
                H5EasyAdd(filename,'/CovarianceInfo/RThighImuToRHipCtrCovariance',obj.m_rthighImuToHipCtrCovariance);
                H5EasyAdd(filename,'/CovarianceInfo/RThighImuToRKneeCtrCovariance',obj.m_rthighImuToKneeCtrCovariance);
                H5EasyAdd(filename,'/CovarianceInfo/RShankImuToRKneeCtrCovariance',obj.m_rshankImuToKneeCtrCovariance);
                H5EasyAdd(filename,'/CovarianceInfo/RShankImuToRAnkleCtrCovariance',obj.m_rshankImuToAnkleCtrCovariance);
                H5EasyAdd(filename,'/CovarianceInfo/RFootImuToRAnkleCtrCovariance',obj.m_rfootImuToAnkleCtrCovariance);
                H5EasyAdd(filename,'/CovarianceInfo/SacrumImuToLHipCtrCovariance',obj.m_sacrumImuToLHipCtrCovariance);
                H5EasyAdd(filename,'/CovarianceInfo/LThighImuToLHipCtrCovariance',obj.m_lthighImuToHipCtrCovariance);
                H5EasyAdd(filename,'/CovarianceInfo/LThighImuToLKneeCtrCovariance',obj.m_lthighImuToKneeCtrCovariance);
                H5EasyAdd(filename,'/CovarianceInfo/LShankImuToLKneeCtrCovariance',obj.m_lshankImuToKneeCtrCovariance);
                H5EasyAdd(filename,'/CovarianceInfo/LShankImuToLAnkleCtrCovariance',obj.m_lshankImuToAnkleCtrCovariance);
                H5EasyAdd(filename,'/CovarianceInfo/LFootImuToLAnkleCtrCovariance',obj.m_lfootImuToAnkleCtrCovariance);
                H5EasyAdd(filename,'/CovarianceInfo/SacrumImuAngVelPrincipalVariance',obj.m_sacrumImuAngVelPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/RThighImuAngVelPrincipalVariance',obj.m_rthighImuAngVelPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/RShankImuAngVelPrincipalVariance',obj.m_rshankImuAngVelPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/RFootImuAngVelPrincipalVariance',obj.m_rfootImuAngVelPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/LThighImuAngVelPrincipalVariance',obj.m_lthighImuAngVelPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/LShankImuAngVelPrincipalVariance',obj.m_lshankImuAngVelPrincipalVariance);
                H5EasyAdd(filename,'/CovarianceInfo/LFootImuAngVelPrincipalVariance',obj.m_lfootImuAngVelPrincipalVariance);
            end
            % model settings
            % remember that strings can't be written with h5write(). continue to use the old hdf5write() for those.
            hdf5write(filename,'/ModelSettings/ModelType','LowerBodyPoseEstimator','writemode','append');
            % useful: make =@(s) strcat('H5EasyAdd(filename,''/ModelSettings/',s(3:end),''',obj.',s,',''WriteMode'',''append'');');
            H5EasyAdd(filename,'/ModelSettings/useRFemurLengthFactor',obj.m_useRFemurLengthFactor);
            H5EasyAdd(filename,'/ModelSettings/useRTibiaLengthFactor',obj.m_useRTibiaLengthFactor);
            H5EasyAdd(filename,'/ModelSettings/useLFemurLengthFactor',obj.m_useLFemurLengthFactor);
            H5EasyAdd(filename,'/ModelSettings/useLTibiaLengthFactor',obj.m_useLTibiaLengthFactor);
            H5EasyAdd(filename,'/ModelSettings/usePelvicWidthFactor',obj.m_usePelvicWidthFactor);
            H5EasyAdd(filename,'/ModelSettings/useMaxAnthroConstraint',obj.m_useMaxAnthroConstraint);
            H5EasyAdd(filename,'/ModelSettings/useMinAnthroConstraint',obj.m_useMinAnthroConstraint);
            H5EasyAdd(filename,'/ModelSettings/useJointVelFactor',obj.m_useJointVelFactor);
            H5EasyAdd(filename,'/ModelSettings/useFemurLengthDiscrepancyFactor',obj.m_useFemurLengthDiscrepancyFactor);
            H5EasyAdd(filename,'/ModelSettings/useTibiaLengthDiscrepancyFactor',obj.m_useTibiaLengthDiscrepancyFactor);
            H5EasyAdd(filename,'/ModelSettings/useKneeAxisTibiaOrthogonalityFactor',obj.m_useKneeAxisTibiaOrthogonalityFactor);
            H5EasyAdd(filename,'/ModelSettings/useKneeAxisFemurOrthogonalityFactor',obj.m_useKneeAxisFemurOrthogonalityFactor);
            hdf5write(filename,'/ModelSettings/gender',obj.m_gender,'writemode','append');
            H5EasyAdd(filename,'/ModelSettings/rfemurLength',obj.m_rfemurLength);
            H5EasyAdd(filename,'/ModelSettings/rtibiaLength',obj.m_rtibiaLength);
            H5EasyAdd(filename,'/ModelSettings/lfemurLength',obj.m_lfemurLength);
            H5EasyAdd(filename,'/ModelSettings/ltibiaLength',obj.m_ltibiaLength);
            H5EasyAdd(filename,'/ModelSettings/pelvicWidth',obj.m_pelvicWidth);
            H5EasyAdd(filename,'/ModelSettings/rfemurLengthSigma',obj.m_rfemurLengthSigma);
            H5EasyAdd(filename,'/ModelSettings/rtibiaLengthSigma',obj.m_rtibiaLengthSigma);
            H5EasyAdd(filename,'/ModelSettings/lfemurLengthSigma',obj.m_lfemurLengthSigma);
            H5EasyAdd(filename,'/ModelSettings/ltibiaLengthSigma',obj.m_ltibiaLengthSigma);
            H5EasyAdd(filename,'/ModelSettings/pelvicWidthSigma',obj.m_pelvicWidthSigma);
            H5EasyAdd(filename,'/ModelSettings/angBwKneeAxisAndSegmentMeanRFemur',obj.m_angBwKneeAxisAndSegmentMeanRFemur);
            H5EasyAdd(filename,'/ModelSettings/angBwKneeAxisAndSegmentStdFemur',obj.m_m_angBwKneeAxisAndSegmentStdFemur);
            H5EasyAdd(filename,'/ModelSettings/angBwKneeAxisAndSegmentMeanLFemur',obj.m_angBwKneeAxisAndSegmentMeanLFemur);
            H5EasyAdd(filename,'/ModelSettings/angBwKneeAxisAndSegmentMeanRTibia',obj.m_angBwKneeAxisAndSegmentMeanRTibia);
            H5EasyAdd(filename,'/ModelSettings/angBwKneeAxisAndSegmentStdTibia',obj.m_angBwKneeAxisAndSegmentStdTibia);
            H5EasyAdd(filename,'/ModelSettings/angBwKneeAxisAndSegmentMeanLTibia',obj.m_angBwKneeAxisAndSegmentMeanLTibia);
            H5EasyAdd(filename,'/ModelSettings/kneeJointCtrConnectionNoise',obj.m_kneeJointCtrConnectionNoise);
            H5EasyAdd(filename,'/ModelSettings/hipJointCtrConnectionNoise',obj.m_hipJointCtrConnectionNoise);
            H5EasyAdd(filename,'/ModelSettings/ankleJointCtrConnectionNoise',obj.m_ankleJointCtrConnectionNoise);
            H5EasyAdd(filename,'/ModelSettings/kneeHingeAxisNoise',obj.m_kneeHingeAxisNoise);
            H5EasyAdd(filename,'/ModelSettings/hipHingeAxisNoise',obj.m_hipHingeAxisNoise);
            H5EasyAdd(filename,'/ModelSettings/ankleHingeAxisNoise',obj.m_ankleHingeAxisNoise);
            H5EasyAdd(filename,'/ModelSettings/jointCtrVelConnectionNoise',obj.m_jointCtrVelConnectionNoise);
            try H5EasyAdd(filename,'/ModelSettings/numPreintegratedImuMeasurements',obj.m_sacrumImuPoseProblem.m_numMeasToPreint); catch; end
            H5EasyAdd(filename,'/ModelSettings/femurLengthDiscrepancyNoise',obj.m_femurLengthDiscrepancyNoise);
            H5EasyAdd(filename,'/ModelSettings/tibiaLengthDiscrepancyNoise',obj.m_tibiaLengthDiscrepancyNoise);
            H5EasyAdd(filename,'/ModelSettings/assumeHipHinge',obj.m_assumeHipHinge);
            % solver options
            H5EasyAdd(filename,'/SolverSettings/optimizerType',obj.m_optimizerType);
            H5EasyAdd(filename,'/SolverSettings/absErrDecreaseLimit',obj.m_absErrDecreaseLimit);
            H5EasyAdd(filename,'/SolverSettings/relErrDecreaseLimit',obj.m_relErrDecreaseLimit);
            H5EasyAdd(filename,'/SolverSettings/maxIterations',obj.m_maxIterations);
            % general stuff
            hdf5write(filename,'/General/GeneratingModelType','matlab.lowerBodyPoseEstimator','writemode','append');
            H5EasyAdd(filename,'/General/nKeyframes',length(obj.m_sacrumImuOrientation));
            hdf5write(filename,'/General/ID',obj.id,'writemode','append');
            % optional debugging notes:
            % view file with: h5disp(filename,'/','min');
            % can read data with, for example, x=h5read(filename,'/Estimated/R_Sacrum_to_N');
            % () double check on way out: make sure you can populate info
            info=h5info(filename);
            fprintf('done. (%.2f seconds)\n',toc(startTic));
        end
        
        %+++++++++++++++++++++++++++++++++++++++++++++++++%
        %---- Methods for testing/debugging/asserting ----%
        %+++++++++++++++++++++++++++++++++++++++++++++++++%
        function jointVelocityCalcConsistencyCheck(obj)
            % debug method: make sure the analytical calculation of joint velocity you make is equivalent to the discrete derivative of joint position
            errorTol=1e-5;
            dt=(obj.m_time(2)-obj.m_time(1));
            % () first: get baseline integration error for position/velocity of an imu
            rthighImuPos=Point3ArrayToMatrix(obj.m_rthighImuPosition);
            rthighImuVelDiff=diff(rthighImuPos)./dt;
            rthighImuVelError=obj.m_rthighImuVelocity(1:end-1,:)-rthighImuVelDiff;
            fprintf('mean norm error of rthigh Imu velocity: %.4f m/s\n',mean( sqrt(sum(rthighImuVelError.^2,2))));
            % starting with RHip
            for k=1:length(obj.m_rthighImuPosition)
                rhipPosRThighImu(k,:)=obj.m_rthighImuPosition(k).vector()'+(obj.m_rthighImuOrientation(k).matrix()*obj.m_rthighImuToHipCtr.vector())';
                rhipLinVelRThighImu(k,:)=obj.m_rthighImuVelocity(k,:);
                rhipTangentialVelocityRThighImu(k,:)=(obj.m_rthighImuOrientation(k).matrix()*cross(obj.m_rthighImuAngVel(k,:)',obj.m_rthighImuToHipCtr.vector()))';
                rhipVelRThighImu(k,:)=rhipLinVelRThighImu(k,:)+rhipTangentialVelocityRThighImu(k,:); % total velocity = linear + tangential component
            end
            rhipVelRThighImuDiff=diff(rhipPosRThighImu)./dt;
            rhipVelRThighImuError=rhipVelRThighImu(2:end,:)-rhipVelRThighImuDiff;
        end
        
        function assertHeadingRotationConsistency(obj)
            % given that there are no magnetometer factors here, you should be able to rotate all of the values by the same rotation in heading and produce the same error
            % () remove any mag factors
            graph=imuPoseEstimator.copyGraphRemoveMagnetometerFactors(obj.m_graph);
            % () remove the priors on the pose and velocity
            graph=imuPoseEstimator.copyGraphRemovePriors(graph);
            origError=graph.error(obj.m_estimate);
            rotationsToTest=linspace(0,2*pi,8); testErr=zeros(length(rotationsToTest),1);
            for k=1:length(rotationsToTest)
                newvals=imuPoseEstimator.copyValuesRotateByHeadingAngle(obj.m_estimate,rotationsToTest(k),obj.m_thighImuPoseProblem.accG,'a','b');
                testErr(k)=graph.error(newvals);
            end
            assert(all( abs(testErr-origError)<1e-5 ),'some of your heading rotations produced different errors...');
        end
        
        function setDerivedJointAnglesFromEstimatedStates(obj)
            % from the already-assumed-set estimated states in this model, set all derived joint angles and segment coordinate systems
            t=tic; fprintf('    setting derived joint angles and skeletal coordinate systems...');
            [obj.m_rhipFlexExAng,obj.m_rhipIntExtRotAng,obj.m_rhipAbAdAng,obj.m_R_Pelvis_to_N,~]=bruteForceRHipAngleCalc(obj);
            [obj.m_rkneeFlexExAng,obj.m_rkneeIntExtRotAng,obj.m_rkneeAbAdAng,obj.m_R_RFemur_to_N,obj.m_R_RTibia_to_N]=bruteForceRKneeAngleCalc(obj);
            [obj.m_lhipFlexExAng,obj.m_lhipIntExtRotAng,obj.m_lhipAbAdAng,~,~]=bruteForceLHipAngleCalc(obj);
            [obj.m_lkneeFlexExAng,obj.m_lkneeIntExtRotAng,obj.m_lkneeAbAdAng,obj.m_R_LFemur_to_N,obj.m_R_LTibia_to_N]=bruteForceLKneeAngleCalc(obj);
            fprintf(' done! (%.2f sec)\n',toc(t));
        end
        
        function setMemberStatesFromValues(obj,vals)
            % update Mar 9, 2020: you have all of the relevant keys stored in member variables. use those!
            % use stored keys to loop through values and set arrays of estimated values
            % stored as arrays of: orientation: gtsam.Rot3, position: gtsam.Point3, velocity, imu gyro & accel bias: native matlab matrix
            fprintf('setting member states from optimized solution... '); startTic=tic;
            % () preallocate all member variables
            N=obj.getNumKeyframes();
            obj.m_sacrumImuPose=repmat(gtsam.Pose3,[N 1]);
            obj.m_sacrumImuOrientation=repmat(gtsam.Rot3,[N 1]);
            obj.m_sacrumImuPosition=repmat(gtsam.Point3,[N 1]);
            obj.m_sacrumImuVelocity=zeros(N,3);
            obj.m_sacrumImuGyroBias=zeros(N,3);
            obj.m_sacrumImuAccelBias=zeros(N,3);
            obj.m_rthighImuPose=repmat(gtsam.Pose3,[N 1]);
            obj.m_rthighImuOrientation=repmat(gtsam.Rot3,[N 1]);
            obj.m_rthighImuPosition=repmat(gtsam.Point3,[N 1]);
            obj.m_rthighImuVelocity=zeros(N,3);
            obj.m_rthighImuGyroBias=zeros(N,3);
            obj.m_rthighImuAccelBias=zeros(N,3);
            obj.m_rshankImuPose=repmat(gtsam.Pose3,[N 1]);
            obj.m_rshankImuOrientation=repmat(gtsam.Rot3,[N 1]);
            obj.m_rshankImuPosition=repmat(gtsam.Point3,[N 1]);
            obj.m_rshankImuVelocity=zeros(N,3);
            obj.m_rshankImuGyroBias=zeros(N,3);
            obj.m_rshankImuAccelBias=zeros(N,3);
            obj.m_rfootImuPose=repmat(gtsam.Pose3,[N 1]);
            obj.m_rfootImuOrientation=repmat(gtsam.Rot3,[N 1]);
            obj.m_rfootImuPosition=repmat(gtsam.Point3,[N 1]);
            obj.m_rfootImuVelocity=zeros(N,3);
            obj.m_rfootImuGyroBias=zeros(N,3);
            obj.m_rfootImuAccelBias=zeros(N,3);
            obj.m_lthighImuPose=repmat(gtsam.Pose3,[N 1]);
            obj.m_lthighImuOrientation=repmat(gtsam.Rot3,[N 1]);
            obj.m_lthighImuPosition=repmat(gtsam.Point3,[N 1]);
            obj.m_lthighImuVelocity=zeros(N,3);
            obj.m_lthighImuGyroBias=zeros(N,3);
            obj.m_lthighImuAccelBias=zeros(N,3);
            obj.m_lshankImuPose=repmat(gtsam.Pose3,[N 1]);
            obj.m_lshankImuOrientation=repmat(gtsam.Rot3,[N 1]);
            obj.m_lshankImuPosition=repmat(gtsam.Point3,[N 1]);
            obj.m_lshankImuVelocity=zeros(N,3);
            obj.m_lshankImuGyroBias=zeros(N,3);
            obj.m_lshankImuAccelBias=zeros(N,3);
            obj.m_lfootImuPose=repmat(gtsam.Pose3,[N 1]);
            obj.m_lfootImuOrientation=repmat(gtsam.Rot3,[N 1]);
            obj.m_lfootImuPosition=repmat(gtsam.Point3,[N 1]);
            obj.m_lfootImuVelocity=zeros(N,3);
            obj.m_lfootImuGyroBias=zeros(N,3);
            obj.m_lfootImuAccelBias=zeros(N,3);
            % () in loop, populate all IMU values
            for k=1:N
                % sacrum IMU
                obj.m_sacrumImuPose(k)=obj.m_estimate.atPose3(obj.m_sacrumImuPoseProblem.m_poseKeys(k));
                obj.m_sacrumImuOrientation(k)=obj.m_sacrumImuPose(k).rotation();
                obj.m_sacrumImuPosition(k)=obj.m_sacrumImuPose(k).translation();
                obj.m_sacrumImuVelocity(k,1:3)=obj.m_estimate.atVector(obj.m_sacrumImuPoseProblem.m_velKeys(k))';
                currentImuBias=obj.m_estimate.atConstantBias(obj.m_sacrumImuPoseProblem.m_imuBiasKeys(k));
                obj.m_sacrumImuGyroBias(k,1:3)=currentImuBias.gyroscope()';
                obj.m_sacrumImuAccelBias(k,1:3)=currentImuBias.accelerometer()';
                % rthigh IMU
                obj.m_rthighImuPose(k)=obj.m_estimate.atPose3(obj.m_rthighImuPoseProblem.m_poseKeys(k));
                obj.m_rthighImuOrientation(k)=obj.m_rthighImuPose(k).rotation();
                obj.m_rthighImuPosition(k)=obj.m_rthighImuPose(k).translation();
                obj.m_rthighImuVelocity(k,1:3)=obj.m_estimate.atVector(obj.m_rthighImuPoseProblem.m_velKeys(k))';
                currentImuBias=obj.m_estimate.atConstantBias(obj.m_rthighImuPoseProblem.m_imuBiasKeys(k));
                obj.m_rthighImuGyroBias(k,1:3)=currentImuBias.gyroscope()';
                obj.m_rthighImuAccelBias(k,1:3)=currentImuBias.accelerometer()';
                % rshank IMU
                obj.m_rshankImuPose(k)=obj.m_estimate.atPose3(obj.m_rshankImuPoseProblem.m_poseKeys(k));
                obj.m_rshankImuOrientation(k)=obj.m_rshankImuPose(k).rotation();
                obj.m_rshankImuPosition(k)=obj.m_rshankImuPose(k).translation();
                obj.m_rshankImuVelocity(k,1:3)=obj.m_estimate.atVector(obj.m_rshankImuPoseProblem.m_velKeys(k))';
                currentImuBias=obj.m_estimate.atConstantBias(obj.m_rshankImuPoseProblem.m_imuBiasKeys(k));
                obj.m_rshankImuGyroBias(k,1:3)=currentImuBias.gyroscope()';
                obj.m_rshankImuAccelBias(k,1:3)=currentImuBias.accelerometer()';
                % rfoot IMU
                obj.m_rfootImuPose(k)=obj.m_estimate.atPose3(obj.m_rfootImuPoseProblem.m_poseKeys(k));
                obj.m_rfootImuOrientation(k)=obj.m_rfootImuPose(k).rotation();
                obj.m_rfootImuPosition(k)=obj.m_rfootImuPose(k).translation();
                obj.m_rfootImuVelocity(k,1:3)=obj.m_estimate.atVector(obj.m_rfootImuPoseProblem.m_velKeys(k))';
                currentImuBias=obj.m_estimate.atConstantBias(obj.m_rfootImuPoseProblem.m_imuBiasKeys(k));
                obj.m_rfootImuGyroBias(k,1:3)=currentImuBias.gyroscope()';
                obj.m_rfootImuAccelBias(k,1:3)=currentImuBias.accelerometer()';
                % lthigh IMU
                obj.m_lthighImuPose(k)=obj.m_estimate.atPose3(obj.m_lthighImuPoseProblem.m_poseKeys(k));
                obj.m_lthighImuOrientation(k)=obj.m_lthighImuPose(k).rotation();
                obj.m_lthighImuPosition(k)=obj.m_lthighImuPose(k).translation();
                obj.m_lthighImuVelocity(k,1:3)=obj.m_estimate.atVector(obj.m_lthighImuPoseProblem.m_velKeys(k))';
                currentImuBias=obj.m_estimate.atConstantBias(obj.m_lthighImuPoseProblem.m_imuBiasKeys(k));
                obj.m_lthighImuGyroBias(k,1:3)=currentImuBias.gyroscope()';
                obj.m_lthighImuAccelBias(k,1:3)=currentImuBias.accelerometer()';
                % lshank IMU
                obj.m_lshankImuPose(k)=obj.m_estimate.atPose3(obj.m_lshankImuPoseProblem.m_poseKeys(k));
                obj.m_lshankImuOrientation(k)=obj.m_lshankImuPose(k).rotation();
                obj.m_lshankImuPosition(k)=obj.m_lshankImuPose(k).translation();
                obj.m_lshankImuVelocity(k,1:3)=obj.m_estimate.atVector(obj.m_lshankImuPoseProblem.m_velKeys(k))';
                currentImuBias=obj.m_estimate.atConstantBias(obj.m_lshankImuPoseProblem.m_imuBiasKeys(k));
                obj.m_lshankImuGyroBias(k,1:3)=currentImuBias.gyroscope()';
                obj.m_lshankImuAccelBias(k,1:3)=currentImuBias.accelerometer()';
                % lfoot IMU
                obj.m_lfootImuPose(k)=obj.m_estimate.atPose3(obj.m_lfootImuPoseProblem.m_poseKeys(k));
                obj.m_lfootImuOrientation(k)=obj.m_lfootImuPose(k).rotation();
                obj.m_lfootImuPosition(k)=obj.m_lfootImuPose(k).translation();
                obj.m_lfootImuVelocity(k,1:3)=obj.m_estimate.atVector(obj.m_lfootImuPoseProblem.m_velKeys(k))';
                currentImuBias=obj.m_estimate.atConstantBias(obj.m_lfootImuPoseProblem.m_imuBiasKeys(k));
                obj.m_lfootImuGyroBias(k,1:3)=currentImuBias.gyroscope()';
                obj.m_lfootImuAccelBias(k,1:3)=currentImuBias.accelerometer()';
            end
            obj.m_rkneeAxisThigh=vals.atUnit3(obj.m_rkneeAxisThighKey);
            obj.m_rkneeAxisShank=vals.atUnit3(obj.m_rkneeAxisShankKey);
            obj.m_lkneeAxisThigh=vals.atUnit3(obj.m_lkneeAxisThighKey);
            obj.m_lkneeAxisShank=vals.atUnit3(obj.m_lkneeAxisShankKey);
            if obj.m_assumeHipHinge
                obj.m_hipAxisSacrum = vals.atUnit3(obj.m_hipAxisSacrumKey);
            end
            if obj.m_assumeAnkleHinge
                obj.m_rankleAxisFoot = vals.atUnit3(obj.m_rankleAxisFootKey);
                obj.m_lankleAxisFoot = vals.atUnit3(obj.m_lankleAxisFootKey);
            end
            obj.m_sacrumImuToRHipCtr=obj.m_estimate.atPoint3(obj.m_sacrumImuToRHipCtrKey);
            obj.m_sacrumImuToLHipCtr=obj.m_estimate.atPoint3(obj.m_sacrumImuToLHipCtrKey);
            obj.m_rthighImuToKneeCtr=obj.m_estimate.atPoint3(obj.m_rthighImuToKneeCtrKey);
            obj.m_rthighImuToHipCtr=obj.m_estimate.atPoint3(obj.m_rthighImuToHipCtrKey);
            obj.m_rshankImuToKneeCtr=obj.m_estimate.atPoint3(obj.m_rshankImuToKneeCtrKey);
            obj.m_rshankImuToAnkleCtr=obj.m_estimate.atPoint3(obj.m_rshankImuToAnkleCtrKey);
            obj.m_rfootImuToAnkleCtr=obj.m_estimate.atPoint3(obj.m_rfootImuToAnkleCtrKey);
            obj.m_lthighImuToKneeCtr=obj.m_estimate.atPoint3(obj.m_lthighImuToKneeCtrKey);
            obj.m_lthighImuToHipCtr=obj.m_estimate.atPoint3(obj.m_lthighImuToHipCtrKey);
            obj.m_lshankImuToKneeCtr=obj.m_estimate.atPoint3(obj.m_lshankImuToKneeCtrKey);
            obj.m_lshankImuToAnkleCtr=obj.m_estimate.atPoint3(obj.m_lshankImuToAnkleCtrKey);
            obj.m_lfootImuToAnkleCtr=obj.m_estimate.atPoint3(obj.m_lfootImuToAnkleCtrKey);
            if obj.m_sternumImuAdded
                obj.m_sternumImuToSacrum=obj.m_estimate.atPoint3(obj.m_sternumImuToSacrumKey);
                obj.m_sacrumImuToSternum=obj.m_estimate.atPoint3(obj.m_sacrumImuToSternumKey);
            end
            obj.m_sacrumImuAngVel=zeros(length(obj.m_sacrumImuAngVelKeys),3);
            obj.m_rthighImuAngVel=zeros(length(obj.m_rthighImuAngVelKeys),3);
            obj.m_rshankImuAngVel=zeros(length(obj.m_rshankImuAngVelKeys),3);
            obj.m_rfootImuAngVel=zeros(length(obj.m_rfootImuAngVelKeys),3);
            obj.m_lthighImuAngVel=zeros(length(obj.m_lthighImuAngVelKeys),3);
            obj.m_lshankImuAngVel=zeros(length(obj.m_lshankImuAngVelKeys),3);
            obj.m_lfootImuAngVel=zeros(length(obj.m_lfootImuAngVelKeys),3);
            for k=1:length(obj.m_sacrumImuAngVelKeys)
                obj.m_sacrumImuAngVel(k,1:3)=obj.m_estimate.atVector(obj.m_sacrumImuAngVelKeys(k))';
                obj.m_rthighImuAngVel(k,1:3)=obj.m_estimate.atVector(obj.m_rthighImuAngVelKeys(k))';
                obj.m_rshankImuAngVel(k,1:3)=obj.m_estimate.atVector(obj.m_rshankImuAngVelKeys(k))';
                obj.m_rfootImuAngVel(k,1:3)=obj.m_estimate.atVector(obj.m_rfootImuAngVelKeys(k))';
                obj.m_lthighImuAngVel(k,1:3)=obj.m_estimate.atVector(obj.m_lthighImuAngVelKeys(k))';
                obj.m_lshankImuAngVel(k,1:3)=obj.m_estimate.atVector(obj.m_lshankImuAngVelKeys(k))';
                obj.m_lfootImuAngVel(k,1:3)=obj.m_estimate.atVector(obj.m_lfootImuAngVelKeys(k))';
            end
            fprintf('complete! (%.4f seconds)\n',toc(startTic));
        end
        
        function [factorErrorsRaw,factorErrorsWeighted]=getJointConnectionErrorResiduals(graph,vals,filterKeys1,filterKeys2)
            fv=NonlinearFactorGraphToFactorVector(graph);
            staticJointCtrOffsetFactors=cell(0,0);
            filteringKeys=vertcat(filterKeys1(:),filterKeys2(:));
            for k=1:length(fv)
                if isa(fv{k},'bioslam.ConstrainedJointCenterPositionFactor')
                    fvkeys=KeyVectorToUintKeyArray(fv{k}.keys);
                    if all(ismember(filteringKeys,fvkeys)) % fvkeys contains all of filteringKeys--you've found a factor which includes the keys you want
                        staticJointCtrOffsetFactors{end+1}=fv{k};
                    end
                end
            end
            % () now pull out errors for each
            factorErrorsRaw=zeros(length(staticJointCtrOffsetFactors),3); % [xyz errors for each factor]
            factorErrorsWeighted=zeros(length(staticJointCtrOffsetFactors),1);
            for i=1:length(staticJointCtrOffsetFactors)
                myKeyVector=staticJointCtrOffsetFactors{i}.keys;
                xA=vals.atPose3(myKeyVector.at(0));
                xB=vals.atPose3(myKeyVector.at(1));
                vecA=vals.atPoint3(myKeyVector.at(2));
                vecB=vals.atPoint3(myKeyVector.at(3));
                factorErrorsRaw(i,1:3)=staticJointCtrOffsetFactors{i}.evaluateError(xA,xB,vecA,vecB)';
                factorErrorsWeighted(i)=staticJointCtrOffsetFactors{i}.error(vals);
            end
            % remember that factorErrorsWeighted is: exp(-0.5*[rawError]*SigmasMatrix*[rawError]')
            % so can compute weighted error w by uncommenting the following code:
            %             e=staticJointCtrOffsetFactors{i}.evaluateError(xA,xB,vecA,vecB);
            %             s=obj.m_kneeJointCtrConnectionNoiseModel.sigmas;
            %             w=0.5*abs(e')*inv(m)*abs(e); % or equivalently 0.5*sumsqr(e./s) for diagonal noise models
            % NOTE: they do this instead of putting it in the exp() because exp() is a monotonic function: so optimizing exp(stuff) and (stuff) have the same argmin of (stuff)!
        end
        
        function setImuOrientationsBasedOnJointAngleLimits(obj)
            % start at the sacrum and work down both legs.
            % based on initial values for imu poses and expected joint angle limits
            % algorithm: (repeat for each joint as you move down the leg)
            % for k=1:N
            %    calculate joint angles at k
            %    if all joint angles in bounds, do nothing
            %    else
            %        find what rotation deltaR, applied to distal imu, would bring all joint angles in bounds
            %        apply deltaR to all distal imu orientations k:N
            %    (optional) loop back through joint angles k:N and make sure all are in bounds now
            startTic=tic; fprintf('\tfor each joint in model, adjusting distal IMU so angles are in bounds:');
            % () set boundaries on angles
            % positive angles for right knee/hip are: extension, adduction, internal rot
            kneeFlexExMaxMin=[deg2rad(12.1) -deg2rad(162.9)];
            kneeAbAdMaxMin=[deg2rad(25) -deg2rad(20)];
            kneeIntExtRotMaxMin=[deg2rad(20) -deg2rad(25)];
            hipFlexExMaxMin=deg2rad([140 -30]);
            rhipAbAdMaxMin=deg2rad([80 -80]);
            rhipIntExtRotMaxMin=deg2rad([80 -80]);
            lhipAbAdMaxMin=deg2rad([80 -80]);
            lhipIntExtRotMaxMin=deg2rad([80 -80]);
            % () for convenience, pull out static vectors into variables
            sacrumImuToRHipCtr=obj.m_initialValues.atPoint3(obj.m_sacrumImuToRHipCtrKey);
            rthighImuToHipCtr=obj.m_initialValues.atPoint3(obj.m_rthighImuToHipCtrKey);
            rthighImuToKneeCtr=obj.m_initialValues.atPoint3(obj.m_rthighImuToKneeCtrKey);
            rshankImuToKneeCtr=obj.m_initialValues.atPoint3(obj.m_rshankImuToKneeCtrKey);
            rshankImuToAnkleCtr=obj.m_initialValues.atPoint3(obj.m_rshankImuToAnkleCtrKey);
            sacrumImuToLHipCtr=obj.m_initialValues.atPoint3(obj.m_sacrumImuToLHipCtrKey);
            lthighImuToHipCtr=obj.m_initialValues.atPoint3(obj.m_lthighImuToHipCtrKey);
            lthighImuToKneeCtr=obj.m_initialValues.atPoint3(obj.m_lthighImuToKneeCtrKey);
            lshankImuToKneeCtr=obj.m_initialValues.atPoint3(obj.m_lshankImuToKneeCtrKey);
            lshankImuToAnkleCtr=obj.m_initialValues.atPoint3(obj.m_lshankImuToAnkleCtrKey);
            rkneeAxisThigh=obj.m_initialValues.atUnit3(obj.m_rkneeAxisThighKey);
            rkneeAxisShank=obj.m_initialValues.atUnit3(obj.m_rkneeAxisShankKey);
            lkneeAxisThigh=obj.m_initialValues.atUnit3(obj.m_lkneeAxisThighKey);
            lkneeAxisShank=obj.m_initialValues.atUnit3(obj.m_lkneeAxisShankKey);
            % () get sacrum imu pose (won't change), as well as other pose trajectories which will be edited here.
            sacrumImuPose=imuPoseEstimator.vectorizePoses(obj.m_initialValues,obj.m_sacrumImuPoseProblem.m_poseKeys);
            rthighImuPose=imuPoseEstimator.vectorizePoses(obj.m_initialValues,obj.m_rthighImuPoseProblem.m_poseKeys);
            rthighImuVel=imuPoseEstimator.vectorizeVelocities(obj.m_initialValues,obj.m_rthighImuPoseProblem.m_velKeys);
            rshankImuPose=imuPoseEstimator.vectorizePoses(obj.m_initialValues,obj.m_rshankImuPoseProblem.m_poseKeys);
            rshankImuVel=imuPoseEstimator.vectorizeVelocities(obj.m_initialValues,obj.m_rshankImuPoseProblem.m_velKeys);
            lthighImuPose=imuPoseEstimator.vectorizePoses(obj.m_initialValues,obj.m_lthighImuPoseProblem.m_poseKeys);
            lthighImuVel=imuPoseEstimator.vectorizeVelocities(obj.m_initialValues,obj.m_lthighImuPoseProblem.m_velKeys);
            lshankImuPose=imuPoseEstimator.vectorizePoses(obj.m_initialValues,obj.m_lshankImuPoseProblem.m_poseKeys);
            lshankImuVel=imuPoseEstimator.vectorizeVelocities(obj.m_initialValues,obj.m_lshankImuPoseProblem.m_velKeys);
            % compute R_Pelvis_to_N
            R_Pelvis_to_N = get_R_Pelvis_to_N(sacrumImuPose, [-1 0 0], sacrumImuToRHipCtr, sacrumImuToLHipCtr,true);
            pelvisRightAxisUnit3=gtsam.Unit3(gtsam.Point3(sacrumImuToRHipCtr.vector()-sacrumImuToLHipCtr.vector()));
            % RHip
            t=tic; fprintf(' RHip...');
            [rthighImuPose,rthighImuVel] = adjustDistalImuTrajectoryForInboundJointAngles(R_Pelvis_to_N,sacrumImuPose,pelvisRightAxisUnit3,rthighImuPose,rthighImuVel,rthighImuToHipCtr,rthighImuToKneeCtr,rkneeAxisThigh,hipFlexExMaxMin,rhipAbAdMaxMin,rhipIntExtRotMaxMin);
            fprintf('\b\b\b (%.2f sec)',toc(t));
            % RKnee
            t=tic; fprintf(', RKnee...');
            R_RFemur_to_N = get_R_Segment_to_N(rthighImuPose, rkneeAxisThigh, rthighImuToHipCtr, rthighImuToKneeCtr,false);
            [rshankImuPose,rshankImuVel] = adjustDistalImuTrajectoryForInboundJointAngles(R_RFemur_to_N,rthighImuPose,rkneeAxisThigh,rshankImuPose,rshankImuVel,rshankImuToKneeCtr,rshankImuToAnkleCtr,rkneeAxisShank,kneeFlexExMaxMin,kneeAbAdMaxMin,kneeIntExtRotMaxMin);
            fprintf('\b\b\b (%.2f sec)',toc(t));
            % LHip
            t=tic; fprintf(', LHip...');
            [lthighImuPose,lthighImuVel] = adjustDistalImuTrajectoryForInboundJointAngles(R_Pelvis_to_N,sacrumImuPose,pelvisRightAxisUnit3,lthighImuPose,lthighImuVel,lthighImuToHipCtr,lthighImuToKneeCtr,lkneeAxisThigh,hipFlexExMaxMin,lhipAbAdMaxMin,lhipIntExtRotMaxMin);
            fprintf('\b\b\b (%.2f sec)',toc(t));
            % LKnee
            t=tic; fprintf(', LKnee...');
            R_LFemur_to_N = get_R_Segment_to_N(lthighImuPose, lkneeAxisThigh, lthighImuToHipCtr, lthighImuToKneeCtr,false);
            [lshankImuPose,lshankImuVel] = adjustDistalImuTrajectoryForInboundJointAngles(R_LFemur_to_N,lthighImuPose,lkneeAxisThigh,lshankImuPose,lshankImuVel,lshankImuToKneeCtr,lshankImuToAnkleCtr,lkneeAxisShank,kneeFlexExMaxMin,kneeAbAdMaxMin,kneeIntExtRotMaxMin);
            fprintf('\b\b\b (%.2f sec)',toc(t));
            % () put these new poses and velocities into the values
            t=tic; fprintf(', updating values...');
            for k=1:length(obj.m_sacrumImuPoseProblem.m_poseKeys)
                obj.m_initialValues.update(obj.m_rthighImuPoseProblem.m_poseKeys(k),rthighImuPose(k));
                obj.m_initialValues.update(obj.m_rthighImuPoseProblem.m_velKeys(k),rthighImuVel(k,1:3)');
                obj.m_initialValues.update(obj.m_rshankImuPoseProblem.m_poseKeys(k),rshankImuPose(k));
                obj.m_initialValues.update(obj.m_rshankImuPoseProblem.m_velKeys(k),rshankImuVel(k,1:3)');
                obj.m_initialValues.update(obj.m_lthighImuPoseProblem.m_poseKeys(k),lthighImuPose(k));
                obj.m_initialValues.update(obj.m_lthighImuPoseProblem.m_velKeys(k),lthighImuVel(k,1:3)');
                obj.m_initialValues.update(obj.m_lshankImuPoseProblem.m_poseKeys(k),lshankImuPose(k));
                obj.m_initialValues.update(obj.m_lshankImuPoseProblem.m_velKeys(k),lshankImuVel(k,1:3)');
            end
            fprintf('\b\b\b (%.2f sec). complete: total time = %.3f sec\n',toc(t),toc(startTic));
        end
        
        function setImuPositionsBasedOnConsistentInitialStaticVecsToJointCtrs(obj)
            % start at the sacrum and work down both legs.
            % based on initial values for the static vectors to joint centers, adjust all positions to be consistent
            startTic=tic; fprintf('\tfor each joint in model, adjusting distal IMU so joint centers are consistent:');
            % for convenience, pull out static vectors into variables
            sacrumImuToRHipCtr=obj.m_initialValues.atPoint3(obj.m_sacrumImuToRHipCtrKey);
            rthighImuToHipCtr=obj.m_initialValues.atPoint3(obj.m_rthighImuToHipCtrKey);
            rthighImuToKneeCtr=obj.m_initialValues.atPoint3(obj.m_rthighImuToKneeCtrKey);
            rshankImuToKneeCtr=obj.m_initialValues.atPoint3(obj.m_rshankImuToKneeCtrKey);
            rshankImuToAnkleCtr=obj.m_initialValues.atPoint3(obj.m_rshankImuToAnkleCtrKey);
            rfootImuToAnkleCtr=obj.m_initialValues.atPoint3(obj.m_rfootImuToAnkleCtrKey);
            sacrumImuToLHipCtr=obj.m_initialValues.atPoint3(obj.m_sacrumImuToLHipCtrKey);
            lthighImuToHipCtr=obj.m_initialValues.atPoint3(obj.m_lthighImuToHipCtrKey);
            lthighImuToKneeCtr=obj.m_initialValues.atPoint3(obj.m_lthighImuToKneeCtrKey);
            lshankImuToKneeCtr=obj.m_initialValues.atPoint3(obj.m_lshankImuToKneeCtrKey);
            lshankImuToAnkleCtr=obj.m_initialValues.atPoint3(obj.m_lshankImuToAnkleCtrKey);
            lfootImuToAnkleCtr=obj.m_initialValues.atPoint3(obj.m_lfootImuToAnkleCtrKey);
            % get sacrum imu pose, it won't be changing
            sacrumImuPose=imuPoseEstimator.vectorizePoses(obj.m_initialValues,obj.m_sacrumImuPoseProblem.m_poseKeys);
            % RHip
            t=tic; fprintf(' RHip...');
            rthighImuPose=imuPoseEstimator.vectorizePoses(obj.m_initialValues,obj.m_rthighImuPoseProblem.m_poseKeys);
            rthighImuPose=lowerBodyPoseEstimator.adjustDistalImuPosBasedOnStaticVecsToJointCtr(sacrumImuPose,rthighImuPose,sacrumImuToRHipCtr,rthighImuToHipCtr);
            fprintf('\b\b\b (%.2f sec)',toc(t));
            % RKnee
            t=tic; fprintf(' RKnee...');
            rshankImuPose=imuPoseEstimator.vectorizePoses(obj.m_initialValues,obj.m_rshankImuPoseProblem.m_poseKeys);
            rshankImuPose=lowerBodyPoseEstimator.adjustDistalImuPosBasedOnStaticVecsToJointCtr(rthighImuPose,rshankImuPose,rthighImuToKneeCtr,rshankImuToKneeCtr);
            fprintf('\b\b\b (%.2f sec)',toc(t));
            % RAnkle
            t=tic; fprintf(' RAnkle...');
            rfootImuPose=imuPoseEstimator.vectorizePoses(obj.m_initialValues,obj.m_rfootImuPoseProblem.m_poseKeys);
            rfootImuPose=lowerBodyPoseEstimator.adjustDistalImuPosBasedOnStaticVecsToJointCtr(rshankImuPose,rfootImuPose,rshankImuToAnkleCtr,rfootImuToAnkleCtr);
            fprintf('\b\b\b (%.2f sec)',toc(t));
            % LHip
            t=tic; fprintf(' LHip...');
            lthighImuPose=imuPoseEstimator.vectorizePoses(obj.m_initialValues,obj.m_lthighImuPoseProblem.m_poseKeys);
            lthighImuPose=lowerBodyPoseEstimator.adjustDistalImuPosBasedOnStaticVecsToJointCtr(sacrumImuPose,lthighImuPose,sacrumImuToLHipCtr,lthighImuToHipCtr);
            fprintf('\b\b\b (%.2f sec)',toc(t));
            % LKnee
            t=tic; fprintf(' LKnee...');
            lshankImuPose=imuPoseEstimator.vectorizePoses(obj.m_initialValues,obj.m_lshankImuPoseProblem.m_poseKeys);
            lshankImuPose=lowerBodyPoseEstimator.adjustDistalImuPosBasedOnStaticVecsToJointCtr(lthighImuPose,lshankImuPose,lthighImuToKneeCtr,lshankImuToKneeCtr);
            fprintf('\b\b\b (%.2f sec)',toc(t));
            % LAnkle
            t=tic; fprintf(' LAnkle...');
            lfootImuPose=imuPoseEstimator.vectorizePoses(obj.m_initialValues,obj.m_lfootImuPoseProblem.m_poseKeys);
            lfootImuPose=lowerBodyPoseEstimator.adjustDistalImuPosBasedOnStaticVecsToJointCtr(lshankImuPose,lfootImuPose,lshankImuToAnkleCtr,lfootImuToAnkleCtr);
            fprintf('\b\b\b (%.2f sec)',toc(t));
            % () put all of these poses back in the initial values
            t=tic; fprintf(', updating values...');
            for k=1:length(obj.m_sacrumImuPoseProblem.m_poseKeys)
                obj.m_initialValues.update(obj.m_rthighImuPoseProblem.m_poseKeys(k),rthighImuPose(k));
                obj.m_initialValues.update(obj.m_rshankImuPoseProblem.m_poseKeys(k),rshankImuPose(k));
                obj.m_initialValues.update(obj.m_rfootImuPoseProblem.m_poseKeys(k),rfootImuPose(k));
                obj.m_initialValues.update(obj.m_lthighImuPoseProblem.m_poseKeys(k),lthighImuPose(k));
                obj.m_initialValues.update(obj.m_lshankImuPoseProblem.m_poseKeys(k),lshankImuPose(k));
                obj.m_initialValues.update(obj.m_lfootImuPoseProblem.m_poseKeys(k),lfootImuPose(k));
            end
            fprintf('\b\b\b (%.2f sec). complete: total time = %.3f sec\n',toc(t),toc(startTic));
            % () double check that error in graph due to ConstrainedJointCenterPositionFactor is zero
            %    could easily print with printErrorsInGraphByFactorType(obj.m_graph,obj.m_initialValues);
            %    for now, it looks like zero!
        end
        
        % --- DESTRUCTOR ---
        function delete(obj) % destructor
            % delete all gtsam.Values objects using clear()
            if ~isempty(obj.m_initialValues); obj.m_initialValues.clear(); end
            if ~isempty(obj.m_estimate); obj.m_estimate.clear(); end
        end
        % -----------------
    end
    methods (Static)
        function [distalImuPose]=adjustDistalImuPosBasedOnStaticVecsToJointCtr(proximalImuPose,distalImuPose,proximalImuToJointCtr,distalImuToJointCtr)
            for k=1:length(proximalImuPose)
                % calculate rhip ctr in world frame
                jointCtrPerProximalImu=proximalImuPose(k).translation().vector()+proximalImuPose(k).rotation().matrix()*proximalImuToJointCtr.vector(); % world frame
                % now find what the distal imu pos must be given its orientation and joint ctr
                distalImuPosFixed=jointCtrPerProximalImu-distalImuPose(k).rotation().matrix()*distalImuToJointCtr.vector();
                % now put this new position into the array
                distalImuPose(k)=gtsam.Pose3(distalImuPose(k).rotation(),gtsam.Point3(distalImuPosFixed));
                % now double check your math
                jointCtrPerDistalImu=distalImuPose(k).translation().vector()+distalImuPose(k).rotation().matrix()*distalImuToJointCtr.vector(); % world frame
                assert(all(abs(jointCtrPerProximalImu-jointCtrPerDistalImu)<1.0e-13));
            end
        end
        function [ImuOrientationRot3Array,ImuPositionPoint3Array,ImuVelocity,ImuGyroBias,ImuAccelBias]=readImuStatesFromH5ResultsFileByString(file,str)
            % this function assumes a good bit about how imu data is stored in the results .h5 file
            ImuOrientation=h5read(file,strcat('/Estimated/R_',str,'Imu_to_N'));
            if size(ImuOrientation,1)==9 % it was flatted by the cpp version, put it as a 3x3xN matrix
                ImuOrientation = flattenedRotationMatrixTo3x3xNMatrix(ImuOrientation);
            end
            ImuPosition=h5read(file,strcat('/Estimated/p_',str,'Imu'));
            if size(ImuPosition,2)>size(ImuPosition,1) % flip to make row indexed
                ImuPosition=ImuPosition';
            end
            ImuVelocity=h5read(file,strcat('/Estimated/v_',str,'Imu'));
            if size(ImuVelocity,2)>size(ImuVelocity,1) % flip to make row indexed
                ImuVelocity=ImuVelocity';
            end
            ImuGyroBias=h5read(file,strcat('/Estimated/gyrobias_',str,'Imu'));
            if size(ImuGyroBias,2)>size(ImuGyroBias,1) % flip to make row indexed
                ImuGyroBias=ImuGyroBias';
            end
            ImuAccelBias=h5read(file,strcat('/Estimated/accelbias_',str,'Imu'));
            if size(ImuAccelBias,2)>size(ImuAccelBias,1) % flip to make row indexed
                ImuAccelBias=ImuAccelBias';
            end
            % () now, since member variables for orientaiton are a Rot3 array and position is a point3 array, put it in that form before returning
            for k=1:length(ImuOrientation)
                ImuOrientationRot3Array(k)=gtsam.Rot3(ImuOrientation(:,:,k));
                ImuPositionPoint3Array(k)=gtsam.Point3(ImuPosition(k,:)');
            end
        end
        function isValidFile=isValidResultsH5File(file)
            % a function which returns a boolean of if this input file is a valid .h5 file representing results from a lowerBodyPoseEstimator
            isValidFile=true;
            % () check: is it a file which exists?
            if exist(file,'file')~=2
                isValidFile=false; return
            end
            % () check: is the extension h5?
            if ~strcmp(file(end-2:end),'.h5')
                isValidFile=false; return
            end
            % () check: can you read this file without error?
            try
                h5info(file);
            catch % some error occured, not a valid file
                isValidFile=false;
            end
            % () check: does it contain results you'd expect from an h5 file generated by lowerbodyposeestimator?
            try
                %                 h5read(file,'/Time'); % <- add this back once you've rerun the cpp versions with Time added.
                h5read(file,'/Estimated/R_RThighImu_to_N');
            catch % some error occured, not a valid file
                isValidFile=false;
            end
        end
        function [factorError,factorErrorWeighted]=getSegmentLengthFactorErrors(graph,vals,imuToProximalJointCtrKey,imuToDistalJointCtrKey)
            fv=NonlinearFactorGraphToFactorVector(graph);
            for k=1:length(fv)
                if isa(fv{k},'bioslam.SegmentLengthMagnitudeFactor')
                    fvkeys=KeyVectorToUintKeyArray(fv{k}.keys);
                    if all(ismember([imuToProximalJointCtrKey imuToDistalJointCtrKey],fvkeys)) % fvkeys contains all of filteringKeys--you've found a factor which includes the keys you want
                        fac=fv{k};
                        break;
                    end
                end
            end
            % () now pull out errors for each
            myKeyVector=fac.keys;
            vec1=vals.atPoint3(myKeyVector.at(0));
            vec2=vals.atPoint3(myKeyVector.at(1));
            factorError=fac.evaluateError(vec1,vec2);
            factorErrorWeighted=fac.error(vals);
        end
        function [sacrum, rthigh, rshank, rfoot, lthigh, lshank, lfoot]=standardAnatomicalPosePriorImuPositions()
            % using foot IMU as 0 0 0. Z up world frame
            rfoot=[0 0 0];
            rshank=[0 0 0.2];
            rthigh=[0 0 .6];
            sacrum=[0 0 .9];
            lfoot=[0 0 0];
            lshank=[0 0 0.2];
            lthigh=[0 0 .6];
        end
        function err=jointAlignmentError(R_A_to_N,pA,sA,R_B_to_N,pB,sB)
            R_A_to_N_mat=Rot3ArrayToMatrices(R_A_to_N);
            R_B_to_N_mat=Rot3ArrayToMatrices(R_B_to_N);
            pA=Point3ArrayToMatrix(pA);
            pB=Point3ArrayToMatrix(pB);
            sA=sA.vector(); sB=sB.vector();
            err=zeros(length(R_A_to_N_mat),3);
            for k=1:length(err)
                err(k,:)=[R_A_to_N_mat(:,:,k)*sA]'+pA(k,:) - ([R_B_to_N_mat(:,:,k)*sB]'+pB(k,:));
            end
        end
        function err=jointVelocityError(R_A_to_N,linVelA,angVelA,sA,R_B_to_N,linVelB,angVelB,sB)
            % compute the joint velocity error via the velocity definition
            jointVelA=lowerBodyPoseEstimator.jointVelocity(R_A_to_N,linVelA,angVelA,sA);
            jointVelB=lowerBodyPoseEstimator.jointVelocity(R_B_to_N,linVelB,angVelB,sB);
            err=jointVelA-jointVelB;
        end
        function jointVel=jointVelocity(R_A_to_N,linVelA,angVelA,sA)
            assert(length(R_A_to_N)==length(angVelA));
            jointVel=zeros(length(angVelA),3);
            for k=1:length(R_A_to_N)
                jointVel(k,1:3)=linVelA(k,1:3)+(R_A_to_N(k).matrix()*(cross(angVelA(k,1:3)',sA.vector())))';
            end
        end
        function err=jointVelocityErrorViaJointPosDiff(R_A_to_N,pA,sA,R_B_to_N,pB,sB,dt)
            R_A_to_N_mat=Rot3ArrayToMatrices(R_A_to_N);
            R_B_to_N_mat=Rot3ArrayToMatrices(R_B_to_N);
            pA=Point3ArrayToMatrix(pA);
            pB=Point3ArrayToMatrix(pB);
            sA=sA.vector(); sB=sB.vector();
            jointPosA=zeros(length(R_A_to_N_mat),3);
            jointPosB=zeros(length(R_A_to_N_mat),3);
            for k=1:length(R_A_to_N_mat)
                jointPosA(k,:)=[R_A_to_N_mat(:,:,k)*sA]'+pA(k,:);
                jointPosB(k,:)=[R_B_to_N_mat(:,:,k)*sB]'+pB(k,:);
            end
            % now take diffs to find velocities
            jointVelA=diff(jointPosA)./dt; jointVelB=diff(jointPosB)./dt; % will have nRows-1
            err=jointVelA-jointVelB;
        end
        function newVals=alignImuValuesInHeadingAtIdx(goodVals,offVals,gN,idx)
            % () convert both sets of initial orientation to ypr()
            newVals=gtsam.Values(offVals);
            [goodValsPose3,keysOfPose3sFoundUintGoodVals]=getAllPose3FromValues(goodVals);
            [offValsPose3,keysOfPose3sFoundUintOffVals]=getAllPose3FromValues(offVals);
            goodValsRot3=Pose3ArrayToRot3Array(goodValsPose3);
            offValsRot3=Pose3ArrayToRot3Array(offValsPose3);
            goodValsYpr=zeros(length(goodValsRot3),3);
            offValsYpr=goodValsYpr;
            for k=1:length(goodValsYpr)
                goodValsYpr(k,:)=goodValsRot3(k).ypr()';
                offValsYpr(k,:)=offValsRot3(k).ypr()';
            end
            % () set the shank's YPR to the thigh's, but keep the offset at idx
            goodValsHeading=goodValsYpr(:,1); offValsHeading=offValsYpr(:,1);
            goodValsHeadingUnwrapped=unwrap(goodValsHeading);
            offValsHeadingUnwrapped=unwrap(offValsHeading);
            headingOffsetAtIdx=goodValsHeadingUnwrapped(idx)-offValsHeadingUnwrapped(idx);
            adjustedHeadingUnwrapped=goodValsHeadingUnwrapped-headingOffsetAtIdx;
            % () reconstruct the relevant Pose3s
            adjustedRot3=gtsamRot3YprConstructor(wrapToPi(adjustedHeadingUnwrapped),wrapToPi(offValsYpr(:,2)),wrapToPi(offValsYpr(:,3)));
            debugThis=1;
            if debugThis
                % check to make sure the new ypr() from adjustedRot3 matches what you expect
                for k=1:length(adjustedRot3)
                    adjustedYpr(k,1:3)=adjustedRot3(k).ypr()';
                    expectedYpr(k,1:3)=[wrapToPi(adjustedHeadingUnwrapped(k)),offValsYpr(k,2),offValsYpr(k,3)];
                end
                assert(all(all(abs(adjustedYpr-expectedYpr)<1e-10)),'didnt construct the Rot3s correctly');
            end
            for k=1:length(goodValsHeading)
                adjustedPose3(k)=gtsam.Pose3(adjustedRot3(k),offValsPose3(k).translation());
            end
            % () now set these new Pose3's to the Values
            for k=1:length(offValsPose3)
                newVals.update(keysOfPose3sFoundUintOffVals(k),adjustedPose3(k));
            end
        end
        function kneePos=kneeCtrAccordingToImu(Rimu,Pimu,vec)
            % inputs: IMU orientation Rimu, IMU position Pimu, IMU->knee rotation vector vec (in IMU frame)
            % can't do this math in GTSAM's primitives!
            imuPos=Point3ArrayToMatrix(Pimu);
            vecOffset=[vec.x() vec.y() vec.z()];
            kneePos=zeros(length(Rimu),3);
            for k=1:length(Rimu)
                kneePos(k,:)=imuPos(k,:)+[Rimu(k).matrix()*vecOffset']'; % imuPos[N] + R[B->N]*vec[B]
            end
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [errRmse,axisErrDeg]=hingeAxesNavFrameComparisonRmse(thighAxis,shankAxis,Rot3_A_to_N,Rot3_B_to_N)
thighAxis=thighAxis(:)';
shankAxis=shankAxis(:)';
r_A_to_N=Rot3ArrayToMatrices(Rot3_A_to_N);
r_B_to_N=Rot3ArrayToMatrices(Rot3_B_to_N);
axisErrDeg=zeros(length(r_A_to_N),1);
for k=1:length(r_A_to_N)
    thighAxisN=[r_A_to_N(:,:,k)*thighAxis']';
    shankAxisN=[r_B_to_N(:,:,k)*shankAxis']';
    axisErrDeg(k)=rad2deg(unsignedAngle(thighAxisN,shankAxisN));
end
errRmse=sqrt(mean(axisErrDeg.^2));
end