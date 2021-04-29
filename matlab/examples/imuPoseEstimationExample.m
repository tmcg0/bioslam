% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

% example usage of the MATLAB implementation of the imuPoseEstimator class

clc; clear; close all;

addpath(fullfile(fileparts(matlab.desktop.editor.getActiveFilename),'..','..','matlab','src')) % add src/ directory
addpath(fullfile(fileparts(matlab.desktop.editor.getActiveFilename),'..','..','matlab','utils')) % add utils/ directory

VarStrToCharMap.clear();

% get test data directory
[filepath,name,ext] = fileparts(matlab.desktop.editor.getActiveFilename);
testDataDir=fullfile(filepath,'..','..','test','data');

% construct a data file
imus=ImuData(fullfile(testDataDir,'20170411-154746-Y1_TUG_6.h5'));
% imus=ImuData(fullfile(dropboxpath,'Research','treadmill_study','data','imu','20190923-124627-S330MinWalk.h5'));
myImu=imus(strcmp('Sternum',{imus.label}));
figure('units','normalized','position',[0.1300 0.5500 0.250 0.250]);
% plot quaternion from manufacturer's onboard filter:
figure;
lh=plot(repmat(myImu.time,[1 4]),myImu.qAPDM);
set(lh,{'color'},{[0 0 0]; [1 0 0]; [0 1 0]; [0 0 1]});
legend('q_s','q_x','q_y','q_z');
grid on; xlabel('time (sec)'); ylabel('quaternion component');

% construct imuposeestimator
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
% ipe.robustOptimize(1);
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
