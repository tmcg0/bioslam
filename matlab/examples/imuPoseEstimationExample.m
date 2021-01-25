% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

% example usage of the MATLAB implementation of the imuPoseEstimator class
clc; clear; close all;

addpath('/home/tmcgrath/bioslam/matlab/src');
addpath(genpath('/home/tmcgrath/bioslam/matlab/utils'));

VarStrToCharMap.clear();

% get test data directory
[filepath,name,ext] = fileparts(matlab.desktop.editor.getActiveFilename);
testDataDir=fullfile(filepath,'..','..','test','data');

% construct a data file
imus=OpalIMUData(fullfile(testDataDir,'20170412-162455-Y2_TUG_5.h5'));
% imus=OpalIMUData(fullfile(dropboxpath,'Research','treadmill_study','data','imu','20190923-124627-S330MinWalk.h5'));
% imus=cutOpalIMU(imus,400,451);
% rightThighImu=imus(strcmp('Right Thigh',{imus.label}));
% rightShankImu=imus(strcmp('Right Tibia',{imus.label}));
myImu=imus(strcmp('Sternum',{imus.label}));
figure('units','normalized','position',[0.1300 0.5500 0.250 0.250]);
quatplot(myImu.qAPDM); title('APDM Quaternion'); drawnow;

% construct imuposeestimator
doOnlinePlot=1;

ipe=imuPoseEstimator(myImu,1,'test');
% --- add settings ---
ipe.accG=[0 0 -9.81];
ipe.m_useMagnetometer=0;
ipe.m_usePositionPrior=1;
ipe.m_useVelocityPrior=1;
ipe.m_useImuBiasPrior=1;
ipe.m_useCompassPrior=1;
ipe.m_numMeasToPreint=10;
ipe.m_optimizerType=1; % 0 for GN, 1 for LM
% ipe.m_priorRotMode='manual';
ipe.m_priorRotNoise=[1e-2 1e3 1e3];
ipe.m_magnetometerNoise=[1e3 1e3 1e3];
% --------------------
ipe.setup();
% ipe.robustOptimize(doOnlinePlot);
ipe.fastOptimize();
% ipe.defaultOptimize();
% ipe.plotImuFactorErrors();
ipe.plotEstimatedValues();
ipe.plotAccelDebug();

% try animating
% ipe.animate();

% test printing data to files
testfilename=fullfile(pwd,'test');
ipe.printYprToFile(strcat(testfilename,'.ypr'));
ipe.printQuatToFile(strcat(testfilename,'.quat'));
ipe.printAccelBiasToFile(strcat(testfilename,'.accelbias'));
ipe.printGyroBiasToFile(strcat(testfilename,'.gyrobias'));
ipe.printDcmToFile(strcat(testfilename,'.dcm'));
ipe.printPosToFile(strcat(testfilename,'.pos'));
ipe.printVelToFile(strcat(testfilename,'.vel'));
alltestfilenames=fullfile(pwd,strcat('test',{'.ypr','.quat','.pos','.dcm','.accelbias','.gyrobias','.vel'}));
delete(alltestfilenames{:});
