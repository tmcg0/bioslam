% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

% unit test the MATLAB implementation of the imuPoseEstimator class
clc; clear; close all;

addpath(fullfile(fileparts(matlab.desktop.editor.getActiveFilename),'..','..','matlab','src')) % add src/ directory
addpath(fullfile(fileparts(matlab.desktop.editor.getActiveFilename),'..','..','matlab','utils')) % add utils/ directory

VarStrToCharMap.clear();

% get test data directory
[filepath,name,ext] = fileparts(matlab.desktop.editor.getActiveFilename);
testDataDir=fullfile(filepath,'..','..','test','data');

% construct a data file
imus=ImuData(fullfile(testDataDir,'20170411-154746-Y1_TUG_6.h5'));
myImu=imus(strcmp('Right Thigh',{imus.label}));

% plot quaternion from manufacturer's onboard filter:
figure('units','normalized','position',[0.1300 0.5500 0.250 0.250]);
lh=plot(repmat(myImu.time,[1 4]),myImu.qAPDM);
set(lh,{'color'},{[0 0 0]; [1 0 0]; [0 1 0]; [0 0 1]});
legend('q_s','q_x','q_y','q_z');
grid on; xlabel('time (sec)'); ylabel('quaternion component');

% construct imuposeestimator
doOnlinePlot=1;

% ipe=imuPoseEstimator(myImu,1,'test');
accG=[0 0 -9.81];

% ()
ipe=testIpeStandard(myImu,accG,0,0);

% ()
testIpeStrongPriorsForMarginals(myImu,accG);

% Philosophy: expected behaviors of this IMU estimation routine
% 1) it should run under the "standard" conditions--initialize orientation from the forward/backward EKF
%   Note: this should be your closest to real results (near zero-ish biases, though you could init those too if you were able)
% 2) Now, set the init to something wrong, like [1 0 0 0] with a medium prior (1e-1). If your estimator is including the accelerometers as it should,
%   then it will move away from this prior and your global error will stay higher due to disagreement between prior and optimized solution (the latter including the accel).
%   Note: if it stays at [1 0 0 0], this means it isn't including the acclerometers and is just integrating the gyros! Bad!
% 3) Now, set keep it at the wrong init ([1 0 0 0]) but force it with a strong prior (1e-5). Now it should stay there, but you should get an awful solution!
%   This is because your strong prior is holding it there, but the accels say it's wrong. So the estimator will drive biases to weird things in order to compensate!


function ipe=testIpeStandard(myImu,accG,useMagnetometer,useMaxVelocity)
VarStrToCharMap.clear();
ipe=imuPoseEstimator(myImu,1,'test');
ipe.accG=accG;
% [q,P]=simpleForwardBackwardEkf([myImu.gx myImu.gy myImu.gz],[myImu.ax myImu.ay myImu.az],1/myImu.sampleRate,4);
% set init rot3 to this
% fprintf('\tinitializing first Rot3 to APDM estimate: [%.2f %.2f %.2f %.2f]\n',myImu.qAPDM(1,:));
% ipe.m_initRot3=gtsam.Rot3.Quaternion(myImu.qAPDM(1,1),myImu.qAPDM(1,2),myImu.qAPDM(1,3),myImu.qAPDM(1,4));
% ipe.m_initRot3=gtsam.Rot3.Quaternion(1,0,0,0);
% ipe.m_initRot3=gtsam.Rot3.Quaternion(q(1,1),q(1,2),q(1,3),q(1,4));
% ipe.m_priorRotNoise=1e-5*[1 1 1];
% ipe.m_priorRotMode='FBEKF';
ipe.setup();
ipe.fastOptimize();
ipe.plotEstimatedValues;
ipe.plotAccelDebug;
end

function ipe=testIpeStrongPriorsForMarginals(myImu,accG)
VarStrToCharMap.clear();
ipe=imuPoseEstimator(myImu,1,'testStrongPriors');
ipe.accG=accG;
noiseLevel=1e-10;
ipe.m_priorRotNoise=ones(1,3)*noiseLevel;
ipe.m_priorPosNoise=ones(1,3)*noiseLevel;
ipe.m_priorVelNoise=ones(1,3)*noiseLevel;
ipe.m_priorRotNoise=ones(1,3)*noiseLevel;
ipe.m_priorAccelBiasNoise=ones(1,3)*noiseLevel;
ipe.m_priorGyroBiasNoise=ones(1,3)*noiseLevel;
ipe.setup();
ipe.fastOptimize();
if isempty(ipe.m_marginals); warning('marginals unset. you used a small noise--this shouldn''t happen!'); end
ipe.plotEstimatedValues();
% ipe.plotAccelDebug();
testPrintingToFile(ipe,1);
end

function testPrintingToFile(ipe,deleteFilesWhenDone)
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
if deleteFilesWhenDone
    delete(alltestfilenames{:});
end
end
