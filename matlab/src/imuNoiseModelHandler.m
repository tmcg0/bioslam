% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

classdef imuNoiseModelHandler < handle
    %imuNoiseModelHandler holds noise model parameters for IMU time-series pose estimation
    % There are 6 relevant noise parameters here. The first three
    % correspond to the static-bias case, while all 6 correspond to the
    % dynamic-bias (random walk of biases) case
    % (1) accelerometer noise sigma
    % (2) gyroscope noise sigma
    % (3) integration error sigma:
    %   - see line 81 of ImuFactor.cpp: preintMeasCov_.block<3, 3>(3, 3).noalias() += iCov * dt;
    %   - so it adds at every integrateMeasurement() to preintMeasCov_(4:6,4:6) (MATLAB indexing)
    %   - I believe this corresponds to the position preintegration covariance. ImuFactor.h line 78 describes preintMeasCov_ as: Matrix9 preintMeasCov_; ///< COVARIANCE OF: [PreintROTATION PreintPOSITION PreintVELOCITY] (first-order propagation from *measurementCovariance*).
    % (4)
    % (5)
    % (6)
    % Reminder: noise parameters in [1]
    % - b^g: bias in gyroscope
    % - eta^(gd): discrete-time noise on gyroscope measurement (related to continuous-time noise eta^(g) by Cov(eta^(gd))=1/dt*Cov(eta^(g))
    % General IMU model assumptions to remember:
    % - IMU states are propogated through Euler integration, which is exact assuming that accel and gyro measurements are constant in interval i->j [1]
    % References:
    % [1] Forster et al. "IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation"
    properties
        imuNoiseModel % a struct for holding noise parameters
        isSet=0;
    end
    
    methods
        function obj = imuNoiseModelHandler(varargin)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            
        end
        
        function setAsOpalv2Default(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            clearNoiseModel(obj);
            obj.imuNoiseModel.accelNoiseSigma=0.04905; % from datasheet for v2: [acc1, acc2] noise density = [150, 5000] ug/sqrt(Hz) (converted 5000 ug/sqrt(Hz) to m/s^2/sqrt(Hz)
            obj.imuNoiseModel.accelBiasRandomWalkSigma=0.008905/2; % no info on datasheet for acc1 or acc2 bias stability. this is a placeholder number
            obj.imuNoiseModel.gyroNoiseSigma=0.000157079633; % from datasheet for v2: gyro noise density = 9 mdps/sqrt(Hz) (converted to rad/s/sqrt(Hz)
            obj.imuNoiseModel.gyroBiasRandomWalkSigma=1.93925e-5; % from datasheet for v2: gyro bias stability = 4 deg/hr (converted to rad/s^2/sqrt(Hz)
            obj.imuNoiseModel.integrationErrSigma=1.0e-8; % from PreintegrationParams.h: "continuous-time "Covariance" describing integration uncertainty"
            obj.imuNoiseModel.preintBiasSigma=1.0e-5;
            obj.isSet=1;
        end
        function setAsGtsamDefault(obj) % bias model that I'm calling 'default' to gtsam. was found in a random example script online. you should use a model that has been determined for your sensor.
            clearNoiseModel(obj);
            obj.imuNoiseModel.accelNoiseSigma=0.03924;
            obj.imuNoiseModel.accelBiasRandomWalkSigma=0.004905;
            obj.imuNoiseModel.gyroNoiseSigma=0.00205689024915;
            obj.imuNoiseModel.gyroBiasRandomWalkSigma=0.00001454441043;
            obj.imuNoiseModel.integrationErrSigma=1.0e-8;
            obj.imuNoiseModel.preintBiasSigma=1.0e-5;
            obj.isSet=1;
        end
        function setAsDebug(obj) % debug parameters. for now choosing the ones that worked in my MATLAB implementation.
            clearNoiseModel(obj);
            obj.imuNoiseModel.accelNoiseSigma=.009924;
            obj.imuNoiseModel.accelBiasRandomWalkSigma=0.004905;
            obj.imuNoiseModel.gyroNoiseSigma=0.00205689024915;
            obj.imuNoiseModel.gyroBiasRandomWalkSigma=0.00001454441043;
            obj.imuNoiseModel.integrationErrSigma=1.0e-8;
            obj.imuNoiseModel.preintBiasSigma=1.0e-5;
            obj.isSet=1;
        end
        function applyModelToPreintMeasParams(obj,p)
            if ~obj.isSet; error('imubiasmodel has not been set!'); end
            assert(isa(p,'gtsam.CombinedPreintegrationParams') || isa(p,'gtsam.PreintegrationParams'));
            if isa(p,'gtsam.PreintegrationParams') || isa(p,'gtsam.CombinedPreintegrationParams')
                % apply the imuBiasModel to the boost::shared_ptr for the PreintegratedCombinedMeasurements
                p.setAccelerometerCovariance(eye(3,3) * obj.imuNoiseModel.accelNoiseSigma^2); % acc white noise in continuous
                p.setIntegrationCovariance(eye(3,3) * obj.imuNoiseModel.integrationErrSigma); % integration uncertainty continuous
                p.setGyroscopeCovariance(eye(3,3) * obj.imuNoiseModel.gyroNoiseSigma^2); % gyro white noise in continuous
            end
            if isa(p,'gtsam.CombinedPreintegrationParams')
                % these are for using combinedImuFactor:
                p.setBiasAccCovariance(eye(3,3) * obj.imuNoiseModel.accelBiasRandomWalkSigma^2); % unset default is eye(3)
                p.setBiasOmegaCovariance(eye(3,3) * obj.imuNoiseModel.gyroBiasRandomWalkSigma^2); % unset default is eye(3)
                p.setBiasAccOmegaInt(eye(6,6)*obj.imuNoiseModel.preintBiasSigma); % variance of bias used for pre-integration
            end
        end
    end
    methods (Access=private)
        function clearNoiseModel(obj)
            % obj.imuNoiseModel = structfun(@(x) [], obj.imuNoiseModel, 'UniformOutput', false); % why doesn't this work?
            obj.imuNoiseModel=[]; obj.isSet=0;
        end
    end
end

