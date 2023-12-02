% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

% unit test the MATLAB implementation of the imuPoseEstimator class
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

% first clear varstrtocharmap
VarStrToCharMap.clear();

% () first test a toy problem
measToUse=1:50;
debugSingleImuFactor([myImu.ax(measToUse) myImu.ay(measToUse) myImu.az(measToUse)],[myImu.gx(measToUse) myImu.gy(measToUse) myImu.gz(measToUse)],...
    1/128,[0 0 -9.81],myImu.qAPDM(measToUse,:));


% construct imuposeestimator
ipe=imuPoseEstimator(myImu,1,'test');

ipe.setupImuFactorStaticBias();

% () now that it's setup, you can inspect the error of the imufactor at the initial condition
getFirstImuFactor(ipe.m_graph,ipe.m_initialValues);

ipe.fastOptimize();
% ipe.plotEstimatedValues();
% ipe.plotAccelDebug();

function debugSingleImuFactor(acc,gyros,dt,gN,qTrue)
% setup a toy problem. single imu factor from:
% Nx3 accel measurements
% Nx3 gyro measurements
% dt
% global frame gravity definition
fprintf('Testing toy problem with %d IMU measurements (%.3f sec @ 128Hz)\n',length(acc),length(acc)/128);
%             initRot3=gtsam.Rot3.Quaternion(0.7916,0.5074,0.3045,0.1522);
            initRot3=gtsam.Rot3;
            initPos=gtsam.Point3(0,0,0);
            initPose=gtsam.Pose3(initRot3,initPos);
            initVelocity=[0 0 0]';
            initBias=gtsam.imuBias.ConstantBias;
%             % noise models
            priorPoseModel=gtsam.noiseModel.Diagonal.Sigmas(1e1*[1 1 1 0.005, 0.005, 0.005]'); % rad, rad, rad, m, m, m
            priorVelModel=gtsam.noiseModel.Diagonal.Sigmas(1e-2*[0.05 0.05 0.05]');
            priorBiasModel=gtsam.noiseModel.Diagonal.Sigmas(1e-2*[0.05 0.05 0.05 0.05 0.05 0.05]');
            % setup graph
            m_graph=gtsam.NonlinearFactorGraph;
            % add priors
            m_graph.push_back(gtsam.PriorFactorPose3(gtsam.symbol('a',0),initPose,priorPoseModel));
            m_graph.push_back(gtsam.PriorFactorVector(gtsam.symbol('b',0),initVelocity,priorVelModel));
            m_graph.push_back(gtsam.PriorFactorConstantBias(gtsam.symbol('c',0),initBias,priorBiasModel));
            % add inital values for first state here
            m_initialValues=gtsam.Values;
            m_initialValues.insert(gtsam.symbol('a',0),initPose);
            m_initialValues.insert(gtsam.symbol('b',0),[0 0 0]');
            m_initialValues.insert(gtsam.symbol('c',0),gtsam.imuBias.ConstantBias);
            % setup noise model
            %             accel_noise_sigma=.003924; % default
            %             gyro_noise_sigma = 0.000205689024915;
            %             accel_bias_rw_sigma = 0.004905;
            %             gyro_bias_rw_sigma = 0.000001454441as043;
            % it seems like gtsam likes if my accel noise is high relative to gyro noise. i.e., trust gyro a lot.
            accel_noise_sigma=.002924;
            gyro_noise_sigma = 0.000205689024915;
%             accel_bias_rw_sigma = 0.004905;
%             gyro_bias_rw_sigma = 0.000001454441043;
            
            measured_acc_cov = eye(3) * accel_noise_sigma^2;
            measured_omega_cov = eye(3) * gyro_noise_sigma^2;
            integration_error_cov = eye(3)*1e-8; % error committed in integrating position from velocities
%             bias_acc_cov = eye(3) * accel_bias_rw_sigma^2;
%             bias_omega_cov = eye(3) * gyro_bias_rw_sigma^2;
%             bias_acc_omega_int = eye(6)*1e-5; % error in the bias used for preintegration
            % setup preintegration params
            p=gtsam.PreintegrationParams(gN');
            p.setAccelerometerCovariance(measured_acc_cov);
            p.setGyroscopeCovariance(measured_omega_cov);
            p.setIntegrationCovariance(integration_error_cov);
            imu_preintegrated=gtsam.PreintegratedImuMeasurements(p,gtsam.imuBias.ConstantBias);
            % () now loop to make preint imu
            for k=1:length(acc)
                imu_preintegrated.integrateMeasurement([acc(k,:)]',[gyros(k,:)]',dt);
            end
            % () now add ImuFactor
            imu_factor=gtsam.ImuFactor(gtsam.symbol('a',0),gtsam.symbol('b',0),gtsam.symbol('a',1),gtsam.symbol('b',1),gtsam.symbol('c',0),imu_preintegrated);
            m_graph.push_back(imu_factor);
            % () add some crap Values to second values
            m_initialValues.insert(gtsam.symbol('a',1),initPose); % insert current pose3
            m_initialValues.insert(gtsam.symbol('b',1),initVelocity); % insert current pose3
            % () graph should be fully setup now. time to optimize.
            params=gtsam.LevenbergMarquardtParams; params.setVerbosityLM('TRYLAMBDA'); params.setVerbosity('TRYLAMBDA');
            optimizer=gtsam.LevenbergMarquardtOptimizer(m_graph, m_initialValues, params);
            optimizer.optimize();
            totalErrorInit=optimizer.error();
            absErrDecreaseLimit=1e-10; relErrDecreaseLimit=1e-10; maxIterations=500;
            absErrorDecrease=9.0e9; relErrorDecrease=9.0e9;
            currentError=999999;
            while absErrorDecrease>absErrDecreaseLimit && relErrorDecrease>relErrDecreaseLimit && optimizer.iterations()<maxIterations
%             for asdf=1:50
                previousError=currentError; % set previous iteration's info
                iterationStart=tic;
                currentLinearGraph=optimizer.iterate(); % : returns gtsam::GaussianFactorGraph % iterate
                iterationTime=toc(iterationStart);
                currentError=optimizer.error(); % get current info
                absErrorDecrease=previousError-currentError; relErrorDecrease=absErrorDecrease/previousError; % compute change in errors
%                 fprintf('iteration %d: (%.4f sec)\n',optimizer.iterations(),iterationTime); % print
%                 fprintf('    error=%.4g (decrease= %.4e || %.4f %%)\n',currentError,absErrorDecrease,relErrorDecrease*100);
            end % main opt loop
            % () now, inspect your results.
            m_estimate=optimizer.values();
            totalErrorOpt=optimizer.error();
            imuFacErrorInit=imu_factor.error(m_initialValues);
            imuFacErrorOpt=imu_factor.error(m_estimate);
            %
            [q0init,q1init,p0init,p1init,v0init,v1init,gyroBiasInit,accelBiasInit]=getValuesToyProblem(m_initialValues);
            [q0opt,q1opt,p0opt,p1opt,v0opt,v1opt,gyroBiasOpt,accelBiasOpt]=getValuesToyProblem(m_estimate);
            % make a Values that has the APDM orientation in it
            trueValues=gtsam.Values(m_estimate);
            trueValues.update(gtsam.symbol('a',0),gtsam.Pose3(gtsam.Rot3.Quaternion(qTrue(1,1),qTrue(1,2),qTrue(1,3),qTrue(1,4)),gtsam.Point3(p0opt(1),p0opt(2),p0opt(3))));
            trueValues.update(gtsam.symbol('a',1),gtsam.Pose3(gtsam.Rot3.Quaternion(qTrue(end,1),qTrue(end,2),qTrue(end,3),qTrue(end,4)),gtsam.Point3(p1opt(1),p1opt(2),p1opt(3))));
            [q0ref,q1ref,p0ref,p1ref,v0ref,v1ref,gyroBiasRef,accelBiasRef]=getValuesToyProblem(trueValues);
            imuFacErrorTrue=imu_factor.error(trueValues);
            fprintf('************************************\n');
            fprintf('initial => optimized values:\n\tpose0: q=[%.4f %.4f %.4f %.4f] => q=[%.4f %.4f %.4f %.4f]\n',q0init,q0opt);
            fprintf('\t\tp=[%.4f %.4f %.4f] => p=[%.4f %.4f %.4f]\n',p0init,p0opt);
            fprintf('\tpose1: q=[%.4f %.4f %.4f %.4f] => q=[%.4f %.4f %.4f %.4f]\n',q1init,q1opt);
            fprintf('\t\tp=[%.4f %.4f %.4f] => p=[%.4f %.4f %.4f]\n',p1init,p1opt);
            fprintf('\tvel0: v=[%.4f %.4f %.4f] => v=[%.4f %.4f %.4f]\n',v0init,v0opt);
            fprintf('\tvel1: v=[%.4f %.4f %.4f] => v=[%.4f %.4f %.4f]\n',v1init,v1opt);
            fprintf('\tgyroBias: gb=[%.4f %.4f %.4f] => gb=[%.4f %.4f %.4f]\n',gyroBiasInit,gyroBiasOpt);
            fprintf('\taccelBias: ab=[%.4f %.4f %.4f] => ab=[%.4f %.4f %.4f]\n',accelBiasInit,accelBiasOpt);
            fprintf('\timu factor error: %.6f => %.6g\n',imuFacErrorInit,imuFacErrorOpt);
            fprintf('\ttotal error: %.6f => %.6g\n',totalErrorInit,totalErrorOpt);
            fprintf('Reference: optimized values but with "true" (APDM) orientations:\n');
            fprintf('\tq0=[%.4f %.4f %.4f %.4f], p0=[%.4f %.4f %.4f]\n',q0ref,p0ref);
            fprintf('\tq1=[%.4f %.4f %.4f %.4f], p1=[%.4f %.4f %.4f]\n',q1ref,p1ref);
            fprintf('\tv0=[%.4f %.4f %.4f]\n',v0ref);
            fprintf('\tv1=[%.4f %.4f %.4f]\n',v1ref);
            fprintf('\tgyroBias: gb=[%.4f %.4f %.4f]\n',gyroBiasRef);
            fprintf('\taccelBias: ab=[%.4f %.4f %.4f]\n',accelBiasRef);
            fprintf('\timu factor error: %.6f\n',imuFacErrorTrue);
            fprintf('************************************\n');
            
end

function [q0,q1,p0,p1,v0,v1,gyroBias,accelBias]=getValuesToyProblem(vals)
pose0=vals.atPose3(gtsam.symbol('a',0));
q0=pose0.rotation().quaternion(); p0=pose0.translation().vector();
pose1=vals.atPose3(gtsam.symbol('a',1));
q1=pose1.rotation().quaternion(); p1=pose1.translation().vector();
v0=vals.atVector(gtsam.symbol('b',0));
v1=vals.atVector(gtsam.symbol('b',1));
imuBias=vals.atConstantBias(gtsam.symbol('c',0));
gyroBias=imuBias.gyroscope();
accelBias=imuBias.accelerometer();
end

function getFirstImuFactor(graph,vals)

allFactors=NonlinearFactorGraphToFactorVector(graph);

end