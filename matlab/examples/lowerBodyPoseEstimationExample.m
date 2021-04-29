% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

% example of the MATLAB implementation of the lowerBodyPoseEstimator class
clc; clear; close all;

% --- settings --- %
mag_G=[19.5563,5.0972,-47.9409];
acc_G=[0.0,0.0,-9.81];
% ---------------- %

% clear char map
VarStrToCharMap.clear();

% get test data directory
[filepath,name,ext] = fileparts(matlab.desktop.editor.getActiveFilename);
testDataDir=fullfile(filepath,'..','..','test','data');

% construct a data file
imus=ImuData(fullfile(testDataDir,'20170411-154746-Y1_TUG_6.h5'));
rightThighImu=imus(strcmp('Right Thigh',{imus.label}));
rightShankImu=imus(strcmp('Right Tibia',{imus.label}));
sacrumImu=imus(strcmp('Sacrum',{imus.label}));
rightFootImu=imus(strcmp('Right Foot',{imus.label}));
leftThighImu=imus(strcmp('Left Thigh',{imus.label}));
leftShankImu=imus(strcmp('Left Tibia',{imus.label}));
leftFootImu=imus(strcmp('Left Foot',{imus.label}));

% construct imuposeestimators
sacrumIpe=imuPoseEstimator(sacrumImu,1,'sacrum');
sacrumIpe.m_usePositionPrior=1; sacrumIpe.m_useVelocityPrior=1; sacrumIpe.m_useImuBiasPrior=0; sacrumIpe.m_useCompassPrior=1;
sacrumIpe.setup();
sacrumIpe.setInitialValuesToOptimizedValues();
rThighIpe=imuPoseEstimator(rightThighImu,1,'rightThigh');
rThighIpe.m_usePositionPrior=0; rThighIpe.m_useVelocityPrior=0; rThighIpe.m_useImuBiasPrior=0; rThighIpe.m_useCompassPrior=0;
rThighIpe.setup();
rThighIpe.setInitialValuesToOptimizedValues();
rShankIpe=imuPoseEstimator(rightShankImu,1,'rightShank');
rShankIpe.m_usePositionPrior=0; rShankIpe.m_useVelocityPrior=0; rShankIpe.m_useImuBiasPrior=0; rShankIpe.m_useCompassPrior=0;
rShankIpe.setup();
rShankIpe.setInitialValuesToOptimizedValues();
rFootIpe=imuPoseEstimator(rightFootImu,1,'rightFoot');
rFootIpe.m_usePositionPrior=0; rFootIpe.m_useVelocityPrior=0; rFootIpe.m_useImuBiasPrior=0; rFootIpe.m_useCompassPrior=0;
rFootIpe.setup();
rFootIpe.setInitialValuesToOptimizedValues();
lThighIpe=imuPoseEstimator(leftThighImu,1,'leftThigh');
lThighIpe.m_usePositionPrior=0; lThighIpe.m_useVelocityPrior=0; lThighIpe.m_useImuBiasPrior=0; lThighIpe.m_useCompassPrior=0;
lThighIpe.setup();
lThighIpe.setInitialValuesToOptimizedValues();
lShankIpe=imuPoseEstimator(leftShankImu,1,'leftShank');
lShankIpe.m_usePositionPrior=0; lShankIpe.m_useVelocityPrior=0; lShankIpe.m_useImuBiasPrior=0; lShankIpe.m_useCompassPrior=0;
lShankIpe.setup();
lShankIpe.setInitialValuesToOptimizedValues();
lFootIpe=imuPoseEstimator(leftFootImu,1,'leftFoot');
lFootIpe.m_usePositionPrior=0; lFootIpe.m_useVelocityPrior=0; lFootIpe.m_useImuBiasPrior=0; lFootIpe.m_useCompassPrior=0;
lFootIpe.setup();
lFootIpe.setInitialValuesToOptimizedValues();

% construct 
lbpe=lowerBodyPoseEstimator(sacrumIpe,rThighIpe,rShankIpe,rFootIpe,lThighIpe,lShankIpe,lFootIpe,'lowerbody');
lbpe.setup();
% lbpe.setAllNoisesHighExceptJointVelocityConstraint();
lbpe.fastOptimize();
h5file=fullfile(pwd,'exampleResults.h5');
lbpe.saveResultsToH5(h5file); % <- can save all the results to an .h5 file
delete(h5file); % <- delete that file

lbpe.jointVelocityCalcConsistencyCheck();

% hingeFactorErrorSurface(lbpe.m_rthighImuOrientation,lbpe.m_rthighImuAngVel,lbpe.m_rshankImuOrientation,lbpe.m_rshankImuAngVel,'A',1);

% some plots:
% plotOptimizerErrorOverTime(lbpe);
% plotRKneeAngles(lbpe);
% plotLKneeAngles(lbpe);
% plotRHipAngles(lbpe);
% plotLHipAngles(lbpe);
% plotEstimatedAngularVelocity(lbpe);
% plotJointConnectionVelocityErrors(lbpe);
% plotJointConnectionPositionErrors(lbpe);
% plotEstimatedImuStates(lbpe);
% plotRelativeEstimatedAngularVelocityVectorsRThighFrame(lbpe);
% plotKneeHingeAxesInNavFrame(lbpe);
% fighJointCtrError=lbpe.plotJointConnectionErrors();
% fighanim=lbpe.animate('animation.gif');
lbpe.plotRKneeRelativeAngularVelocityVectors();
return;

delete('animation.gif');

lbpe.plotEstimatedValues(1);
lbpe.plotKneeCenterAccordingToImus();
lbpe.plotKneeAxisFactorError();
lbpe.plotRelativeAngularVelocityVectorsThighFrame();
lbpe.plotImuHeadings();

% plot single imu accel debug
figh_thighImuAccelDebugPostKnee=plotSingleImuAccelDebug(lbpe.m_thighImuPoseProblem.m_time,lbpe.m_thighImuOrientation,lbpe.m_thighImuPoseProblem.m_imu.time,...
    [lbpe.m_thighImuPoseProblem.m_imu.ax lbpe.m_thighImuPoseProblem.m_imu.ay lbpe.m_thighImuPoseProblem.m_imu.az],[0 0 -9.81]);
figh_thighImuAccelDebugPostKnee.Name='thighImu accel plots, post-knee inference';
figh_shankImuAccelDebugPoseKnee=plotSingleImuAccelDebug(lbpe.m_shankImuPoseProblem.m_time,lbpe.m_shankImuOrientation,lbpe.m_shankImuPoseProblem.m_imu.time,...
    [lbpe.m_shankImuPoseProblem.m_imu.ax lbpe.m_shankImuPoseProblem.m_imu.ay lbpe.m_shankImuPoseProblem.m_imu.az],[0 0 -9.81]);
figh_shankImuAccelDebugPoseKnee.Name='shankImu accel plots, post-knee inference';
