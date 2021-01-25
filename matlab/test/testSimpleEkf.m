% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

% unit test the MATLAB implementation the simple EKF
clc; clear; close all;

addpath('/home/tmcgrath/bioslam/matlab/src');
addpath(genpath('/home/tmcgrath/bioslam/matlab/utils'));

testDataDir=fullfile(strcat(filesep,'home'),'tmcgrath','bioslam','test','data');

% construct a data file
imus=OpalIMUData(fullfile(testDataDir,'20170411-154746-Y1_TUG_6.h5'));
myImu=imus(strcmp('Right Thigh',{imus.label}));

% 
figure('units','normalized','position',[0.1300 0.5500 0.250 0.250]);
quatplot(myImu.qAPDM); drawnow;

% now run the EKF
imu=myImu;
% [x,P]=simpleImuOrientationForwardEkf([imu.gx imu.gy imu.gz],[imu.ax imu.ay imu.az],1/imu.sampleRate,[],[]);
% figure; quatplot(x);

[x,P]=simpleForwardBackwardEkf([imu.gx imu.gy imu.gz],[imu.ax imu.ay imu.az],1/imu.sampleRate,4);

% test: is x similar to APDM's x?
quatError=quatmultiply(imu.qAPDM,quatinv(x));
[r1,r2,r3]=quat2angle(quatError);
r1e=rmse(r1); r2e=rmse(r2); r3e=rmse(r3);
fprintf('error relative to APDM (euler angles, rad): [%.4f %.4f %.4f]\n',[r1e r2e r3e]);


function armse=rmse(a)
armse=sqrt(mean(a.^2));
end