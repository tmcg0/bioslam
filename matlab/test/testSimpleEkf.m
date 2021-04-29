% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

% unit test the MATLAB implementation the simple EKF
clc; clear; close all;

addpath(fullfile(fileparts(matlab.desktop.editor.getActiveFilename),'..','..','matlab','src')) % add src/ directory
addpath(fullfile(fileparts(matlab.desktop.editor.getActiveFilename),'..','..','matlab','utils')) % add utils/ directory

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
title('manufacturer filter');

% now run the EKF
imu=myImu;
[x,P]=simpleForwardBackwardEkf([imu.gx imu.gy imu.gz],[imu.ax imu.ay imu.az],1/imu.sampleRate,4);

% test: is x similar to APDM's x?
quatError=quatmultiply(imu.qAPDM,quatinv(x));
[r1,r2,r3]=quat2angle(quatError);
r1e=rmse(r1); r2e=rmse(r2); r3e=rmse(r3);
fprintf('error relative to APDM (euler angles, rad): [%.4f %.4f %.4f]\n',[r1e r2e r3e]);

% plot bioslam's EKF implementation:
figure('units','normalized','position',[0.1300 0.5500 0.250 0.250]);
lh=plot(repmat(myImu.time,[1 4]),x);
set(lh,{'color'},{[0 0 0]; [1 0 0]; [0 1 0]; [0 0 1]});
legend('q_s','q_x','q_y','q_z');
grid on; xlabel('time (sec)'); ylabel('quaternion component');
title('EKF results');

function armse=rmse(a)
    armse=sqrt(mean(a.^2));
end