% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function figh=plotSingleImuAccelDebug(estTime,est_R_B_to_N,estAccelBias,measTime,measAccel,gN)
% if bias was static, use repmat to expand it
if(size(estAccelBias,1)==1)
    estAccelBias=repmat(estAccelBias,[length(est_R_B_to_N) 1]);
end
gN=gN(:)'; % rowize
% plot debug plots with accelerometers
figh=figure('units','normalized','position',[.05 .05 .8 .8]);
green=[.1 .5 .1];
subplot(2,3,1); hold on; % plot raw accel measurements
plot(measTime,measAccel(:,1),'linewidth',2,'color','r');
plot(measTime,measAccel(:,2),'linewidth',2,'color',green);
plot(measTime,measAccel(:,3),'linewidth',2,'color','b');
grid on; title('raw accel meas');
subplot(2,3,2); hold on; % plot accel meas rotated into global frame
for k=1:length(est_R_B_to_N)-1
%     idx=getNearestIdx(measTime,estTime(k));
    idxs=measTime>=estTime(k) & measTime<estTime(k+1);
    intervalMeanAccel=mean([measAccel(idxs,1) measAccel(idxs,2) measAccel(idxs,3)],1);
    accelG(k,1:3)=[est_R_B_to_N(k).matrix()*(intervalMeanAccel-estAccelBias(k,:))']';
end
plot(estTime(1:end-1),accelG(:,1),'linewidth',2,'color','r');
plot(estTime(1:end-1),accelG(:,2),'linewidth',2,'color',green);
plot(estTime(1:end-1),accelG(:,3),'linewidth',2,'color','b');
grid on; title('meas. accel rotated into nav frame');
subplot(2,3,3); hold on;  % gravity vector rotated into local frame
for k=1:length(est_R_B_to_N)-1
    gB(k,1:3)=[est_R_B_to_N(k).inverse().matrix()*gN']';
end
plot(estTime(1:end-1),gB(:,1),'linewidth',2,'color','r');
plot(estTime(1:end-1),gB(:,2),'linewidth',2,'color',green);
plot(estTime(1:end-1),gB(:,3),'linewidth',2,'color','b');
grid on; title(sprintf('gN=[%.2f %.2f %.2f] rotated into body frame',gN(1),gN(2),gN(3)));
subplot(2,3,4); hold on; % imu body accel
linearAccelN=accelG+gN;
plot(estTime(1:end-1),linearAccelN(:,1),'linewidth',2,'color','r');
plot(estTime(1:end-1),linearAccelN(:,2),'linewidth',2,'color',green);
plot(estTime(1:end-1),linearAccelN(:,3),'linewidth',2,'color','b');
grid on; title('IMU linear accel in Nav frame: meas. accel + gN');

pos=[0 0 0]; vel=[0 0 0];
for k=2:length(est_R_B_to_N)-1
    dt=estTime(k)-estTime(k-1);
    vel(k,:)=vel(k-1,:)+dt*linearAccelN(k-1,:);
    pos(k,:)=pos(k-1,:)+dt*vel(k-1,:)+0.5*dt^2*linearAccelN(k-1,:);
end
subplot(2,3,5); hold on; % velocity
plot(estTime(1:end-1),vel(:,1),'linewidth',2,'color','r');
plot(estTime(1:end-1),vel(:,2),'linewidth',2,'color',green);
plot(estTime(1:end-1),vel(:,3),'linewidth',2,'color','b');
grid on; title('integration of body accel (velocity)');
subplot(2,3,6); hold on; % double integration of linear accel
plot(estTime(1:end-1),pos(:,1),'linewidth',2,'color','r');
plot(estTime(1:end-1),pos(:,2),'linewidth',2,'color',green);
plot(estTime(1:end-1),pos(:,3),'linewidth',2,'color','b');
grid on; title('double integration of linear accel in Nav frame (=>position)');
end