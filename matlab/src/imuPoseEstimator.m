% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

classdef imuPoseEstimator < handle
    %IMUPOSEESTIMATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% ------------ options for modeling and solving ------------- %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % model structure options
        m_usePositionPrior=1, m_useVelocityPrior=1, m_useImuBiasPrior=1, m_useCompassPrior=1; % use priors on the first keyframe?
        m_imuBiasModelMode=0;
        %         m_predictWithNavState=0; % Possible bug with GTSAM: if you enable this, the optimizer will laughably start with zero error and not iterate. as if it refuses to use accelerometer reference.w
        m_imuReturnsToSamePosition=0; % does imu come back to its original position?
        m_imuReturnsToSameVelocity=0; % does imu come back to its original velocity?
        m_imuReturnsToSameOrientation=0; % does imu come back to its original orientation?
        m_useMagnetometer=0;
        % model parameters, some are optional
        m_magnetometerNoise=[100 100 100];
        m_numMeasToPreint=10;
        m_priorRotMode='manual'; % manual for whatever is set here (can be changed by user). Or 'FBEKF' for FBEKF with a noise of 1e-4
        % prior means and noises
        m_priorGyroBias=[0 0 0];
        m_priorAccelBias=[0 0 0];
        m_initRot3=gtsam.Rot3.Quaternion(1,0,0,0);
        m_initPos=[0 0 0];
        m_initVel=[0 0 0];
        m_priorCompassAngle=0;
        m_priorRotNoise=[1e-2 1e3 1e3];
        m_priorPosNoise=1e-3*[1 1 1];
        m_priorVelNoise=1e-3*[1 1 1];
        m_priorAccelBiasNoise=1e-1*[1 1 1];
        m_priorGyroBiasNoise=1e-1*[1 1 1];
        m_priorCompassAngleNoise=1.0e-3;
        % compass system settings
        m_refVecLocal=[1.0,0.0,0.0]'; m_refVecNav=[1.0,0.0,0.0]';
        % --- solver settings --- %
        m_optimizerType=1; % 0 for GN, 1 for LM
        % ----------------------- %
        % the rest of the stuff
        id
        m_imu
        m_time % estimated time domain
        m_graph % NonlinearFactorGraph
        m_initialValues % gtsam.Values
        m_estimate % gtsam.Values
        m_orientation % Rot3 array, think it's R[B->N]
        m_position % Point3 array
        m_velocity % vector array of estimated velocities
        m_accelBias % accelerometer bias, estimated
        m_gyroBias % gyro bias, estimated
        m_effectiveGyroPreints % array of effective gyroscope rates
        m_deltaPij
        m_nKeyframes
        m_imuNoiseModel % imuNoiseModelHandler
        % environmental params
        accG=[0 0 -9.81];
        magG=[30.5, 14.91, -56.3];
        % opt params
        m_optType=1
        m_maxIterations=5000
        m_relErrDecreaseLimit=1e-5;
        m_absErrDecreaseLimit=1e-8;
        % variable string and characters
        m_poseVarStr, m_velVarStr, m_imuBiasVarStr
        m_poseVarChar, m_velVarChar, m_imuBiasVarChar
        % key arrays
        m_poseKeys, m_velKeys, m_imuBiasKeys
        % marginals and principal variances after estimations
        m_marginals % GTSAM::Marginals object
        m_orientationPrincipalVariance, m_positionPrincipalVariance, m_velocityPrincipalVariance, m_gyroBiasPrincipalVariance, m_accelBiasPrincipalVariance
        % initial state. if these are empty, they'll be zero.
        m_initialOrientation=[], m_initialVelocity=[], m_initialPosition=[]
        % optimization results data
        m_optimizationTotalError, m_optimizationTotalTime
    end
    
    methods
        function obj = imuPoseEstimator(varargin)
            %IMUPOSEESTIMATOR Construct an instance of this class
            %   imuPoseEstimator(imu,id)
            if nargin==0
                % constructs empty copy of class
                return
            elseif nargin==3 % normal usage
                if isa(varargin{1},'ImuData') && isa(varargin{2},'numeric') && isa(varargin{3},'char')
                    obj.m_imu=varargin{1}; obj.m_imuBiasModelMode=varargin{2}; obj.id = varargin{3};
                    %m_magnetometerNoise=gtsam::noiseModel::Isotropic::Sigmas(magNoiseVec);
                    % now, from m_strId, set the variable names and add them to the VarStrToCharMap
                    obj.m_poseVarStr=strcat(obj.id,'_ImuPose');
                    obj.m_velVarStr=strcat(obj.id,'_ImuVel');
                    obj.m_imuBiasVarStr=strcat(obj.id,'_ImuBias');
                    %m_magBiasVarStr=m_strId+"_MagBias";
                    VarStrToCharMap.insert(obj.m_poseVarStr);
                    VarStrToCharMap.insert(obj.m_velVarStr);
                    VarStrToCharMap.insert(obj.m_imuBiasVarStr);
                    %VarStrToCharMap::insert(m_magBiasVarStr); // disable wasting a VarStrCharMap variable on this for now, you aren't estimating it.
                    obj.m_poseVarChar=VarStrToCharMap.getChar(obj.m_poseVarStr);
                    obj.m_velVarChar=VarStrToCharMap.getChar(obj.m_velVarStr);
                    obj.m_imuBiasVarChar=VarStrToCharMap.getChar(obj.m_imuBiasVarStr);
                else; error('No matching constructor for imuPoseEstimator');
                end
            else; error('No matching constructor for imuPoseEstimator');
            end
        end
        function delete(obj) % destructor
            % delete all gtsam.Values objects using clear()
            if ~isempty(obj.m_initialValues); obj.m_initialValues.clear(); end
            if ~isempty(obj.m_estimate); obj.m_estimate.clear(); end
        end
        function setup(obj)
            setupTic=tic;
            if obj.m_imuBiasModelMode==0 % static bias
                fprintf('setting up ImuPoseEstimator (''%s'') with static bias (gtsam::ImuFactor) ... \n',obj.id);
                setupImuFactorStaticBias(obj); fprintf('completed! %.4f sec\n',toc(setupTic));
            elseif obj.m_imuBiasModelMode==1 % dynamic bias
                fprintf('setting up ImuPoseEstimator (''%s'') with dynamic bias (gtsam::CombinedImuFactor) ... \n',obj.id);
                setupCombinedImuFactorDynamicBias(obj); fprintf('completed! %.4f sec\n',toc(setupTic));
            else; error('unrecognized imu bias mode');
            end
        end
        function imuIdxArray=getImuIndecesCorrespondingToKeyframes(obj)
            % return an array of the indeces in the imu j which go with keyframes k
            imuIdxArray=1:obj.m_numMeasToPreint:obj.m_imu.numSamples;
            assert(length(imuIdxArray)==length(obj.m_poseKeys));
        end
        function setAnyRequestedPriors(obj)
            % function to implement the request priors scheme from options
            if obj.m_usePositionPrior
                priorPosModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_priorPosNoise(:));
                obj.m_graph.add(bioslam.Pose3TranslationPrior(obj.m_poseKeys(1),obj.m_initPos(:),priorPosModel));
            end
            if obj.m_useVelocityPrior
                priorVelModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_priorVelNoise(:));
                obj.m_graph.add(gtsam.PriorFactorVector(obj.m_velKeys(1),obj.m_initVel(:),priorVelModel));
            end
            if obj.m_useImuBiasPrior
                initBias=gtsam.imuBias.ConstantBias(obj.m_priorAccelBias(:),obj.m_priorGyroBias(:));
                priorBiasModel=gtsam.noiseModel.Diagonal.Sigmas(vertcat(obj.m_priorGyroBiasNoise(:),obj.m_priorAccelBiasNoise(:)));
                obj.m_graph.add(gtsam.PriorFactorConstantBias(obj.m_imuBiasKeys(1),initBias,priorBiasModel));
            end
            if obj.m_useCompassPrior
                priorCompassNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_priorCompassAngleNoise);
                obj.m_graph.add(bioslam.Pose3CompassPrior(obj.m_poseKeys(1),obj.m_refVecLocal,obj.m_refVecNav,-1.0*(obj.accG(:))/sqrt(sum(obj.accG(:).^2,1)),obj.m_priorCompassAngle,priorCompassNoiseModel));
            end
        end
        function setupCombinedImuFactorDynamicBias(obj)
            warning('bug: using dynamic bias estimation is currently broken functionality, using static bias for now. See issue on GitHub at https://github.com/tmcg0/bioslam/issues/4');
            setupImuFactorStaticBias(obj);
            return
            %setupCombinedImuFactorDynamicBias Setup factor graph with dynamic bias (gtsam's CombinedImuFactor)
            %   Detailed explanation goes here
            dt=obj.m_imu.time(2)-obj.m_imu.time(1);
            magDirN=gtsam.Unit3(gtsam.Point3(obj.magG(1),obj.magG(2),obj.magG(3)));
            magScaleN=sqrt(sum(obj.magG(:)'.^2,2));
            magBias=gtsam.Point3(0,0,0);
            %             initRot3=gtsam.Rot3.Quaternion(0.7916,0.5074,0.3045,0.1522);
            if mod(obj.m_imu.numSamples,obj.m_numMeasToPreint)==0
                obj.m_nKeyframes=floor(obj.m_imu.numSamples/obj.m_numMeasToPreint); % see bottom of this file for explanation
            else
                obj.m_nKeyframes=floor(obj.m_imu.numSamples/obj.m_numMeasToPreint)+1;
            end
            % update: this previous equation was wrong when the floor() argument above is directly divisible by obj.m_numMeasToPreint
            % now that you know the number of keyframes, make arrays for the Keys using gtsam::Symbol
            obj.m_poseKeys=zeros(obj.m_nKeyframes,1,'uint64'); obj.m_velKeys=zeros(obj.m_nKeyframes,1,'uint64'); obj.m_imuBiasKeys=zeros(obj.m_nKeyframes,1,'uint64');
            for c=1:obj.m_nKeyframes
                obj.m_poseKeys(c,1)=gtsam.symbol(obj.m_poseVarChar,c-1); % remember GTSAM is zero-indexed, hence k-1
                obj.m_velKeys(c,1)=gtsam.symbol(obj.m_velVarChar,c-1);
                obj.m_imuBiasKeys(c,1)=gtsam.symbol(obj.m_imuBiasVarChar,c-1);
            end
            % set the initial Rotation prior by the selected mode
            switch obj.m_priorRotMode
                case 'manual' % default, set to whatever is already there. May be changed by user after setup
                    % do nothing
                    fprintf('\tsetting prior rot mode manual: default qinit=[%.3f %.3f %.3f %.3f]\n',obj.m_initRot3.quaternion()');
                case 'FBEKF'
                    [qInit,~]=simpleForwardBackwardEkf([obj.m_imu.gx obj.m_imu.gy obj.m_imu.gz],[obj.m_imu.ax obj.m_imu.ay obj.m_imu.az],1/obj.m_imu.sampleRate,4);
                    obj.m_initRot3=gtsam.Rot3.Quaternion(qInit(1,1),qInit(1,2),qInit(1,3),qInit(1,4)); % obj.m_priorRotNoise=1e-1*[1 1 1];
                    fprintf('\tsetting prior on orientation from FBEKF: qinit=[%.3f %.3f %.3f %.3f] and noise=[%.3f %.3f %.3f]\n',qInit(1,:),obj.m_priorRotNoise);
                otherwise error('unrecognized priorRotMode! Select ''manual'' or ''FBEKF''');
            end
            if strcmp(obj.m_priorRotMode,'FBEKF') && obj.m_useMagnetometer==1
                warning('youve selected to use both magneteometer and FBEKF. This will lead to an inconsistent prior: FBEKF right now does not include magnetometer information for heading!');
            end
            initPose=gtsam.Pose3(obj.m_initRot3,gtsam.Point3(obj.m_initPos(:)));
            initBias=gtsam.imuBias.ConstantBias(obj.m_priorAccelBias(:),obj.m_priorGyroBias(:));
            maxAccelBiasNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(1.0e-3);
            % noise models
            magnetometerNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_magnetometerNoise(:));
            % setup graph
            obj.m_graph=gtsam.NonlinearFactorGraph;
            % add any requested priors
            setAnyRequestedPriors(obj);
            % add inital values for first state here
            obj.m_initialValues=gtsam.Values;
            obj.m_initialValues.insert(obj.m_poseKeys(1),initPose);
            obj.m_initialValues.insert(obj.m_velKeys(1),obj.m_initVel(:));
            obj.m_initialValues.insert(obj.m_imuBiasKeys(1),initBias);
            % setup preintegration params
            p=gtsam.CombinedPreintegrationParams(obj.accG(:));
            % setup imunoisemodel and apply it to the preintegration params
            obj.m_imuNoiseModel=imuNoiseModelHandler; obj.m_imuNoiseModel.setAsOpalv2Default();
            obj.m_imuNoiseModel.applyModelToPreintMeasParams(p);
            % setup the preintegrated imu
            imu_preintegrated=gtsam.PreintegratedCombinedMeasurements(p,gtsam.imuBias.ConstantBias);
            % begin loop
            j=1; % indeces of measurements in IMU
            nPreintInLoop=0; % preintegration counter
            obj.m_time(1)=obj.m_imu.time(1);
            gyros=[obj.m_imu.gx obj.m_imu.gy obj.m_imu.gz]; % gyroscope measurements
            accels=[obj.m_imu.ax obj.m_imu.ay obj.m_imu.az]; % accelerometer measurements
            for k=1:obj.m_nKeyframes-1 % k = indeces of keyframe states in graph. remember you got to K-1 because you're setting up factors and adding initial values at k+1 in the loop below.
                %                 fprintf('keyframe k=%d, preintegrating IMU measurements j=[',k); % debug print statement. uncomment to see relationship between keyframes and IMU measurements.
                while nPreintInLoop<obj.m_numMeasToPreint % todo: make this a for loop
                    %                     if nPreintInLoop<obj.m_numMeasToPreint-1; fprintf('%d, ',j); elseif nPreintInLoop==obj.m_numMeasToPreint-1; fprintf(']\n'); end % debug print statement. uncomment to see relationship between keyframes and IMU measurements.
                    imu_preintegrated.integrateMeasurement(accels(j,:)',gyros(j,:)',dt);
                    j=j+1;
                    nPreintInLoop=nPreintInLoop+1;
                end
                obj.m_deltaPij(k,:)=imu_preintegrated.deltaPij()';
                obj.m_effectiveGyroPreints(end+1,1:3)=(imu_preintegrated.deltaRij().xyz())./(dt*nPreintInLoop)';
                % mean(gyros((j-obj.m_numMeasToPreint:j)',:),1) % run this to see that it's similar
                % now you're done with preintegration. move on.
                % () make ImuFactor
                combined_imu_factor=gtsam.CombinedImuFactor(obj.m_poseKeys(k),obj.m_velKeys(k),obj.m_poseKeys(k+1),obj.m_velKeys(k+1),obj.m_imuBiasKeys(k),obj.m_imuBiasKeys(k+1),imu_preintegrated);
                obj.m_graph.add(combined_imu_factor);
                if obj.m_useMagnetometer % add mag factor
                    magMeas=gtsam.Point3(obj.m_imu.mx(j),obj.m_imu.my(j),obj.m_imu.mz(j));
                    obj.m_graph.add(bioslam.MagPose3Factor(gtsam.symbol(obj.m_poseVarChar,k),magMeas,magScaleN,magDirN,magBias,magnetometerNoiseModel));
                end
                % add states to initial values at k+1 (remember you did values for k=1 before loop)
                if isempty(obj.m_initialOrientation); initO=[1 0 0 0]; else; initO=obj.m_initialOrientation(k,:); end
                if isempty(obj.m_initialVelocity); initV=[0 0 0]; else; initV=obj.m_initialVelocity(k,:); end
                if isempty(obj.m_initialPosition); initP=[0 0 0]; else; initP=obj.m_initialPosition(k,:); end
                obj.m_initialValues.insert(obj.m_poseKeys(k+1), gtsam.Pose3( Rot3QuaternionConstructor(initO) , gtsam.Point3(initP(:)))  ); % insert current pose3
                obj.m_initialValues.insert(obj.m_velKeys(k+1),initV(:)); % insert current vel
                obj.m_initialValues.insert(obj.m_imuBiasKeys(k+1),initBias);
                % add current time
                obj.m_time(k+1)=obj.m_time(k)+imu_preintegrated.deltaTij; % should be equivalent to: time(k+1)=time(k)+nPreint*imuMeasDeltaT;
                % reset preintegration
                imu_preintegrated.resetIntegration();
                nPreintInLoop=0;
            end
            % graph should be fully setup now. time to optimize.
            if obj.m_imuReturnsToSamePosition % constrain position to be similar between first and last index
                mynoisemodel=gtsam.noiseModel.Diagonal.Sigmas([1e3 1e3 1e3 .01 .01 .01]');
                obj.m_graph.add(gtsam.BetweenFactorPose3(obj.m_poseKeys(1),obj.m_poseKeys(end),gtsam.Pose3(),mynoisemodel));
            end
            if obj.m_imuReturnsToSameVelocity % constrain velocity to be similar between first and last index
                error('fix this'); % it doesn't look like BetweenFactor<Vector3> exists in the MATLAB wrapper right now
                mynoisemodel=gtsam.noiseModel.Diagonal.Sigmas([.1 .1 .1]');
                obj.m_graph.add(gtsam.PriorFactorVector(gtsam.symbol(obj.m_velVarChar,k-1),[0 0 0]',mynoisemodel));
            end
            if obj.m_imuReturnsToSameOrientation % constrain first and last orientations to be similar
                mynoisemodel=gtsam.noiseModel.Diagonal.Sigmas([1e0 1e0 1e0 1e4 1e4 1e4]');
                obj.m_graph.add(gtsam.BetweenFactorPose3(obj.m_poseKeys(1),obj.m_poseKeys(end),gtsam.Pose3(),mynoisemodel));
            end
        end
        
        function setupImuFactorStaticBias(obj)
            %setupImuFactorStaticBias Setup factor graph with static bias (gtsam's ImuFactor)
            %   Detailed explanation goes here
            dt=obj.m_imu.time(2)-obj.m_imu.time(1);
            magDirN=gtsam.Unit3(gtsam.Point3(obj.magG(1),obj.magG(2),obj.magG(3)));
            magScaleN=sqrt(sum(obj.magG(:)'.^2,2));
            magBias=gtsam.Point3(0,0,0);
            %             initRot3=gtsam.Rot3.Quaternion(0.7916,0.5074,0.3045,0.1522);
            if mod(obj.m_imu.numSamples,obj.m_numMeasToPreint)==0
                obj.m_nKeyframes=floor(obj.m_imu.numSamples/obj.m_numMeasToPreint); % see bottom of this file for explanation
            else
                obj.m_nKeyframes=floor(obj.m_imu.numSamples/obj.m_numMeasToPreint)+1;
            end
            % update: this previous equation was wrong when the floor() argument above is directly divisible by obj.m_numMeasToPreint
            % now that you know the number of keyframes, make arrays for the Keys using gtsam::Symbol
            obj.m_poseKeys=zeros(obj.m_nKeyframes,1,'uint64'); obj.m_velKeys=zeros(obj.m_nKeyframes,1,'uint64'); obj.m_imuBiasKeys=zeros(1,1,'uint64');
            for c=1:obj.m_nKeyframes
                obj.m_poseKeys(c,1)=gtsam.symbol(obj.m_poseVarChar,c-1); % remember GTSAM is zero-indexed, hence k-1
                obj.m_velKeys(c,1)=gtsam.symbol(obj.m_velVarChar,c-1);
            end
            obj.m_imuBiasKeys(1)=gtsam.symbol(obj.m_imuBiasVarChar,0);
            % set the initial Rotation prior by the selected mode
            switch obj.m_priorRotMode
                case 'manual' % default, set to whatever is already there. May be changed by user after setup
                    % do nothing
                    fprintf('\tsetting prior rot mode manual: default qinit=[%.3f %.3f %.3f %.3f]\n',obj.m_initRot3.quaternion()');
                case 'FBEKF'
                    [qInit,~]=simpleForwardBackwardEkf([obj.m_imu.gx obj.m_imu.gy obj.m_imu.gz],[obj.m_imu.ax obj.m_imu.ay obj.m_imu.az],1/obj.m_imu.sampleRate,4);
                    obj.m_initRot3=gtsam.Rot3.Quaternion(qInit(1,1),qInit(1,2),qInit(1,3),qInit(1,4)); % obj.m_priorRotNoise=1e-1*[1 1 1];
                    fprintf('\tsetting prior on orientation from FBEKF: qinit=[%.3f %.3f %.3f %.3f] and noise=[%.3f %.3f %.3f]\n',qInit(1,:),obj.m_priorRotNoise);
                otherwise error('unrecognized priorRotMode! Select ''manual'' or ''FBEKF''');
            end
            if strcmp(obj.m_priorRotMode,'FBEKF') && obj.m_useMagnetometer==1
                warning('youve selected to use both magneteometer and FBEKF. This will lead to an inconsistent prior: FBEKF right now does not include magnetometer information for heading!');
            end
            initPose=gtsam.Pose3(obj.m_initRot3,gtsam.Point3(obj.m_initPos(:)));
            %             initBias=zeros(6,1);
            initBias=gtsam.imuBias.ConstantBias(obj.m_priorAccelBias(:),obj.m_priorGyroBias(:));
            maxAccelBiasNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(1.0e-3);
            % noise models
            magnetometerNoiseModel=gtsam.noiseModel.Diagonal.Sigmas(obj.m_magnetometerNoise(:));
            % setup graph
            obj.m_graph=gtsam.NonlinearFactorGraph;
            % add any requested priors
            setAnyRequestedPriors(obj);
            % add inital values for first state here
            obj.m_initialValues=gtsam.Values;
            obj.m_initialValues.insert(obj.m_poseKeys(1),initPose);
            obj.m_initialValues.insert(obj.m_velKeys(1),obj.m_initVel(:));
            obj.m_initialValues.insert(obj.m_imuBiasKeys(1),initBias);
            % setup preintegration params
            p=gtsam.PreintegrationParams(obj.accG(:));
            % setup imunoisemodel and apply it to the preintegration params
            obj.m_imuNoiseModel=imuNoiseModelHandler; obj.m_imuNoiseModel.setAsOpalv2Default();
            obj.m_imuNoiseModel.applyModelToPreintMeasParams(p);
            % setup the preintegrated imu
            imu_preintegrated=gtsam.PreintegratedImuMeasurements(p,gtsam.imuBias.ConstantBias);
            % begin loop
            j=1; % indeces of measurements in IMU
            nPreintInLoop=0; % preintegration counter
            obj.m_time(1)=0;
            gyros=[obj.m_imu.gx obj.m_imu.gy obj.m_imu.gz]; % gyroscope measurements
            accels=[obj.m_imu.ax obj.m_imu.ay obj.m_imu.az]; % accelerometer measurements
            for k=1:obj.m_nKeyframes-1 % k = indeces of keyframe states in graph. remember you got to K-1 because you're setting up factors and adding initial values at k+1 in the loop below.
                %                 fprintf('keyframe k=%d, preintegrating IMU measurements j=[',k); % debug print statement. uncomment to see relationship between keyframes and IMU measurements.
                while nPreintInLoop<obj.m_numMeasToPreint % todo: make this a for loop
                    %                     if nPreintInLoop<obj.m_numMeasToPreint-1; fprintf('%d, ',j); elseif nPreintInLoop==obj.m_numMeasToPreint-1; fprintf(']\n'); end % debug print statement. uncomment to see relationship between keyframes and IMU measurements.
                    imu_preintegrated.integrateMeasurement(accels(j,:)',gyros(j,:)',dt);
                    j=j+1;
                    nPreintInLoop=nPreintInLoop+1;
                end
                obj.m_deltaPij(k,:)=imu_preintegrated.deltaPij()';
                obj.m_effectiveGyroPreints(end+1,1:3)=(imu_preintegrated.deltaRij().xyz())./(dt*nPreintInLoop)';
                % mean(gyros((j-obj.m_numMeasToPreint:j)',:),1) % run this to see that it's similar
                % now you're done with preintegration. move on.
                % () make ImuFactor
                imu_factor=gtsam.ImuFactor(obj.m_poseKeys(k),obj.m_velKeys(k),obj.m_poseKeys(k+1),obj.m_velKeys(k+1),obj.m_imuBiasKeys(1),imu_preintegrated);
                obj.m_graph.push_back(imu_factor);
                if obj.m_useMagnetometer % add mag factor
                    magMeas=gtsam.Point3(obj.m_imu.mx(j),obj.m_imu.my(j),obj.m_imu.mz(j));
                    obj.m_graph.add(bioslam.MagPose3Factor(gtsam.symbol(obj.m_poseVarChar,k),magMeas,magScaleN,magDirN,magBias,magnetometerNoiseModel));
                end
                % add states to initial values at k+1 (remember you did values for k=1 before loop)
                if isempty(obj.m_initialOrientation); initO=[1 0 0 0]; else; initO=obj.m_initialOrientation(k,:); end
                if isempty(obj.m_initialVelocity); initV=[0 0 0]; else; initV=obj.m_initialVelocity(k,:); end
                if isempty(obj.m_initialPosition); initP=[0 0 0]; else; initP=obj.m_initialPosition(k,:); end
                obj.m_initialValues.insert(obj.m_poseKeys(k+1), gtsam.Pose3( Rot3QuaternionConstructor(initO) , gtsam.Point3(initP(:)))  ); % insert current pose3
                obj.m_initialValues.insert(obj.m_velKeys(k+1),initV(:)); % insert current vel
                % add current time
                obj.m_time(k+1)=obj.m_time(k)+imu_preintegrated.deltaTij; % should be equivalent to: time(k+1)=time(k)+nPreint*imuMeasDeltaT;
                % reset preintegration
                imu_preintegrated.resetIntegration();
                nPreintInLoop=0;
            end
            % graph should be fully setup now. time to optimize.
            if obj.m_imuReturnsToSamePosition % constrain position to be similar between first and last index
                mynoisemodel=gtsam.noiseModel.Diagonal.Sigmas([1e3 1e3 1e3 .01 .01 .01]');
                obj.m_graph.add(gtsam.BetweenFactorPose3(obj.m_poseKeys(1),obj.m_poseKeys(end),gtsam.Pose3(),mynoisemodel));
            end
            if obj.m_imuReturnsToSameVelocity % constrain velocity to be similar between first and last index
                error('fix this'); % it doesn't look like BetweenFactor<Vector3> exists in the MATLAB wrapper right now
                mynoisemodel=gtsam.noiseModel.Diagonal.Sigmas([.1 .1 .1]');
                obj.m_graph.add(gtsam.PriorFactorVector(gtsam.symbol(obj.m_velVarChar,k-1),[0 0 0]',mynoisemodel));
            end
            if obj.m_imuReturnsToSameOrientation % constrain first and last orientations to be similar
                mynoisemodel=gtsam.noiseModel.Diagonal.Sigmas([1e0 1e0 1e0 1e4 1e4 1e4]');
                obj.m_graph.add(gtsam.BetweenFactorPose3(obj.m_poseKeys(1),obj.m_poseKeys(end),gtsam.Pose3(),mynoisemodel));
            end
        end
        
        function setInitialOrientation(obj,r)
            % assuming you haven't optimized yet.
            assert(size(r,2)==4); % a quaternion
            obj.m_initialOrientation=r;
        end
        
        function setInitialPosition(obj,p)
            % assuming you haven't optimized yet.
            assert(size(p,2)==3);
            obj.m_initialPosition=p;
        end
        
        function defaultOptimize(obj)
            optParams=gtsam.LevenbergMarquardtParams;
            optParams.setMaxIterations(obj.m_maxIterations);
            optimizer=gtsam.LevenbergMarquardtOptimizer(obj.m_graph, obj.m_initialValues, optParams);
            fprintf('running defaultOptimize()... '); tich=tic;
            obj.m_estimate=optimizer.optimize();
            fprintf('done! (%.4f sec)\n',toc(tich));
            [obj.m_orientation,obj.m_position,obj.m_velocity,obj.m_accelBias,obj.m_gyroBias] = parseImuValues(obj.m_estimate);
            % () if possible, pull out variances
            setMarginals(obj);
        end
        
        function fastOptimize(obj)
            %%%%%% fastoptimize %%%%%
            if obj.m_optimizerType==0 % Gauss-Newton
                params=gtsam.GaussNewtonParams; params.setVerbosity('DELTA');
                optimizer=gtsam.GaussNewtonOptimizer(obj.m_graph, obj.m_initialValues, params);
            elseif obj.m_optimizerType==1 % Levenberg-Marquardt
                params=gtsam.LevenbergMarquardtParams; verbosity='SUMMARY'; % options are: SUMMARY, TERMINATION, LAMBDA, TRYLAMBDA, TRYCONFIG, DAMPED, TRYDELTA
                params.setlambdaInitial(1e-8); params.setlambdaFactor(10); params.setlambdaUpperBound(1e10); params.setlambdaLowerBound(1e-10);
                params.setRelativeErrorTol(1.0e-20); params.setAbsoluteErrorTol(1.0e-20); % <-set abs and rel params to an extremely small number since you're manually gonna manage the convergence
                params.setVerbosityLM(verbosity); params.setVerbosity(verbosity);
                optimizer=gtsam.LevenbergMarquardtOptimizer(obj.m_graph, obj.m_initialValues, params);
            else; error('unknown optimization type. choose 0 for Gauss-Newton or 1 for Levenberg-Marquardt');
            end
            currentError=optimizer.error(); nIterations=optimizer.iterations();
            fprintf('***** imuPoseEstimator::fastOptimize() *****\n');
            printErrorsInGraphByFactorType(obj.m_graph,obj.m_initialValues);
            fprintf('\tinitial error: %.8g\n',currentError);
            optBeginTic=tic;
            absErrorDecrease=9.0e9; relErrorDecrease=9.0e9;
            % also create a small gui to stop this loop execution
            figh_kill=figure('units','normalized','position',[.4 .45 .2 .1]); ax=axes('Visible','off'); text(0,0,'Close this window to kill optimization routine'); drawnow;
            % optimization stats
            obj.m_optimizationTotalError(nIterations+1)=optimizer.error(); obj.m_optimizationTotalTime(nIterations+1)=0; % gotta add 1 b/c nIterations starts at zero
            while absErrorDecrease>obj.m_absErrDecreaseLimit && relErrorDecrease>obj.m_relErrDecreaseLimit && optimizer.iterations()<obj.m_maxIterations
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
                % print?
                fprintf('  --> end iteration %d (%.4f sec): error=%.6g (decrease= %.4e || %.4f %%)\n',optimizer.iterations()-1,iterationTime,currentError,absErrorDecrease,relErrorDecrease*100);
                if ~ishandle(figh_kill)
                    fprintf('optimization killed be user.\n');
                    break;
                end
                obj.m_optimizationTotalError(nIterations+1)=optimizer.error(); obj.m_optimizationTotalTime(nIterations+1)=toc(optBeginTic); % gotta add 1 b/c nIterations starts at zero
                pause(1.0e-10);
            end
            if ishandle(figh_kill); close(figh_kill); end
            fprintf('---- complete! total time=%.4f sec, %d iterations, final error=%.8g (avg. %.8g per keyframe) ----\n',toc(optBeginTic),optimizer.iterations(),optimizer.error(),optimizer.error()/obj.m_nKeyframes);
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
            if optimizer.error()/obj.m_nKeyframes>10
                warning('this average error per keyframe (%.5f) is higher than typically desired (%d or less). Solution may not be good.',optimizer.error()/obj.m_nKeyframes,10);
            end
            fprintf('*********************************************\n');
            % converged!
            obj.m_estimate=optimizer.values();
            % print errors in graph
            printErrorsInGraphByFactorType(obj.m_graph,obj.m_estimate);
            % pull out values
            [obj.m_orientation,obj.m_position,obj.m_velocity,obj.m_accelBias,obj.m_gyroBias] = parseImuValues(obj.m_estimate);
            % () if possible, pull out variances
            setMarginals(obj);
            % () if no magnetometer, you should be able to rotate these values in the heading plane with little change in global error.
            %             assertHeadingRotationConsistency(obj);
        end
        
        function robustOptimize(obj,doOnlinePlot)
            %%%%%% robustoptimize %%%%%
            absErrDecreaseLimit=1e-8; relErrDecreaseLimit=1e-8;
            if obj.m_optType==0; optimizer=gtsam.GaussNewtonOptimizer(obj.m_graph, obj.m_initialValues);
            elseif obj.m_optType==1; optimizer=gtsam.LevenbergMarquardtOptimizer(obj.m_graph, obj.m_initialValues);
            else error('unknown optimization type. choose 0 or 1');
            end
            
            currentError=optimizer.error();
            if doOnlinePlot % create figure to plot results
                currentVals=optimizer.values();
                q=Rot3ArrayToQuaternions(Pose3ArrayToRot3Array(getAllPose3FromValues(currentVals)));
                p=Point3ArrayToMatrix(Pose3ArrayToPoint3Array(getAllPose3FromValues(currentVals)));
                v=getAllVectorFromValues(currentVals,3);
                [gyroBias,accelBias]=ConstantBiasArrayToNativeTypes(getAllConstantBiasFromValues(currentVals));
                q=quat_unwrap(q);
                myTime=1:length(q);
                errorPerIteration(1)=optimizer.error();
                timePerIteration(1)=0;
                figh=onlinePlot([],myTime,q,p,v,gyroBias,accelBias,errorPerIteration,timePerIteration);
            end
            %             m_estimate=optimizer.optimize();
            optBeginTic=tic;
            ordering=obj.m_graph.orderingCOLAMD;
            absErrorDecrease=9.0e9; relErrorDecrease=9.0e9;
            while absErrorDecrease>absErrDecreaseLimit && relErrorDecrease>relErrDecreaseLimit && optimizer.iterations()<obj.m_maxIterations
                % set previous iteration's info
                previousError=currentError;
                % iterate
                iterationStart=tic;
                currentLinearGraph=optimizer.iterate(); % : returns gtsam::GaussianFactorGraph
                iterationTime=toc(iterationStart);
                % get current info
                currentError=optimizer.error();
                % compute change in errors
                absErrorDecrease=previousError-currentError; relErrorDecrease=absErrorDecrease/previousError;
                % get error statistics for the imu factors
                currentValues=optimizer.values();
                [imuFacErrors,imuFacErrorsWeighted]=imuPoseEstimator.getCombinedImuFactorErrorResiduals(obj.m_graph,currentValues,[]);
                [priorPoseError,priorVelError,priorImuBiasError]=imuPoseEstimator.getPriorFactorResiduals(obj.m_graph,currentValues);
                % CONCLUSION: global error (i.e. optimizer.error()) is the sum of the weighted errors for every factor (i.e., factor.error(vals))
                % print
                fprintf('iteration %d: (%.4f sec)\n',optimizer.iterations(),iterationTime);
                fprintf('    error=%.6g (decrease= %.4e || %.4f %%)\n',currentError,absErrorDecrease,relErrorDecrease*100);
                fprintf('    %s error: mean=%.5f, sumsqr=%.5g, sum=%.4f, max/min=%.4f/%.4f\n','CombinedImuFactor',mean(imuFacErrors(:)),sum(imuFacErrors(:).^2),sum(imuFacErrors(:)),max(imuFacErrors(:)),min(imuFacErrors(:)));
                fprintf('        weighted: sum=%.4g (%.3f%% of total error), sumsqr=%.4g\n',sum(imuFacErrorsWeighted),sum(imuFacErrorsWeighted)*100/currentError,sum(imuFacErrorsWeighted.^2));
                if obj.m_useMagnetometer % collect and print magnetometer eror statistics
                    [magFacErrors,magFacErrorsWeighted]=imuPoseEstimator.getMagFactorPoes3ErrorResiduals(obj.m_graph,currentValues,[]);
                    fprintf('    magnetometer factor error weighted: sum=%.4g (%.3f%% of total error)\n',sum(magFacErrorsWeighted),sum(magFacErrorsWeighted)*100/currentError);
                end
                fprintf('    prior errors, weighted (%% of total error): pose=%.4f (%.3f%%), vel=%.4f (%.3f%%), bias=%.4f (%.3f%%)\n',priorPoseError,priorPoseError*100/currentError,priorVelError,priorVelError*100/currentError,priorImuBiasError,priorImuBiasError*100/currentError);
                if doOnlinePlot
                    currentVals=optimizer.values();
                    q=Rot3ArrayToQuaternions(Pose3ArrayToRot3Array(getAllPose3FromValues(currentVals)));
                    q=quat_unwrap(q);
                    v=getAllVectorFromValues(currentVals,3);
                    p=Point3ArrayToMatrix(Pose3ArrayToPoint3Array(getAllPose3FromValues(currentVals)));
                    [gyroBias,accelBias]=ConstantBiasArrayToNativeTypes(getAllConstantBiasFromValues(currentVals));
                    errorPerIteration(end+1)=currentError;
                    timePerIteration(end+1)=toc(optBeginTic);
                    onlinePlot(figh,myTime,q,p,v,gyroBias,accelBias,errorPerIteration,timePerIteration);
                end
                % Indefinite system: This condition occurs when the system Hessian is indefinite, i.e. non-positive-semidefinite
                %                                 jac=currentLinearGraph.jacobian(ordering); hes=currentLinearGraph.hessian(ordering);
                %                 eigenvalsJac=eig(jac); eigenvalsHes=eig(hes);
                %                 fprintf('    Jacobian size=[%d x %d], rank=%d, det=%.5g, min,max eigenval=%.4f,%.4f => condition num=%.8f\n',size(jac,1),size(jac,2),rank(jac),det(jac),min(eig(jac)),max(eig(jac)),max(eig(jac))/min(eig(jac)));
                %                 fprintf('    Hessian size=[%d x %d], rank=%d, det=%.5g, min,max eigenval=%.4f,%.4f => condition num=%.8f\n',size(hes,1),size(hes,2),rank(hes),det(hes),min(eig(hes)),max(eig(hes)),max(eig(hes))/min(eig(hes)));
                
                %                                 instrumentedDebugJacobian(jac,currentLinearGraph,obj.m_graph,optimizer.values());
                %                                 instrumentedDebugHessian(hes,currentLinearGraph,obj.m_graph,optimizer.values());
                %                                 close all;
            end % main opt loop
            % converged!
            obj.m_estimate=optimizer.values();
            [obj.m_orientation,obj.m_position,obj.m_velocity,obj.m_accelBias,obj.m_gyroBias] = parseImuValues(obj.m_estimate);
            % () if possible, pull out variances
            setMarginals(obj);
        end
        
        function setInitialValuesToOptimizedValues(obj)
            % simple method to take your initial values and set them to the optimal solution... don't iterate
            obj.m_estimate=obj.m_initialValues;
            [obj.m_orientation,obj.m_position,obj.m_velocity,obj.m_accelBias,obj.m_gyroBias] = parseImuValues(obj.m_estimate);
            % () if possible, pull out variances
            %             setMarginals(obj);
        end
        
        function figh=plotEstimatedValues(obj)
            % turned this into a wrapper for a static method so it had access for other classes. called through:
            % imuPoseEstimator.plotSingleImu(time,orientation,velocity,position,gyrobias,accelbias);
            % where:
            %   time = Nx1 array of times (seconds)
            %   orientation = 3x3xN DCM array in SO(3)
            %   velcocity, position, gyrobias, accelbias = Nx3 array in R(3)
            figh=imuPoseEstimator.plotSingleImu(obj.m_time,obj.m_orientation,obj.m_velocity,obj.m_position,obj.m_gyroBias,obj.m_accelBias,...
                obj.m_orientationPrincipalVariance,obj.m_velocityPrincipalVariance,obj.m_positionPrincipalVariance,obj.m_gyroBiasPrincipalVariance,obj.m_accelBiasPrincipalVariance);
        end
        function figh=plotAccelDebug(obj)
            % just a wrapper for plotSingleImuAccelDebug(). parse data and call it.
            figh=plotSingleImuAccelDebug(obj.m_time,obj.m_orientation,obj.m_accelBias,obj.m_imu.time,[obj.m_imu.ax obj.m_imu.ay obj.m_imu.az],obj.accG);
        end
        function figh=plotImuFactorErrors(obj)
            warning('this is not fully implemented yet');
            % () get the errors
            if obj.m_imuBiasModelMode==0 % static
                [imuFactorErrors,imuFactorErrorsWeighted]=imuPoseEstimator.getImuFactorErrorResiduals(obj.m_graph,obj.m_estimate,[]);
                % imuFactorErrors is Nx9. I think it's (1:3)=>error due to rotation, (4:6)=>error due to position, (7:9)=>error due to velocity
                rotError=imuFactorErrors(:,1:3);
                posError=imuFactorErrors(:,4:6);
                velError=imuFactorErrors(:,7:9);
                figure; plot(rotError);
            elseif obj.m_imuBiasModelMode==1 % dynamic
                [imuFactorErrors,imuFactorErrorsWeighted]=imuPoseEstimator.getCombinedImuFactorErrorResiduals(obj.m_graph,obj.m_estimate,[]);
                % imuFactorErrors is Nx15. I think it's (1:3)=>error due to rotation, (4:6)=>error due to position, (7:9)=>error due to velocity, rest are biases
                rotError=imuFactorErrors(:,1:3);
                posError=imuFactorErrors(:,4:6);
                velError=imuFactorErrors(:,7:9);
                figure; plot(rotError); figure; plot(posError); title('pos error');
            end
            figh=figure;
            
        end
        function assertPositionIntegrationConsistency(obj)
            % assert that the double integration to position is consistent, i.e.,
            % that integrating the bias-subtracted, gravity removed acceleration twice results in the position
        end
        %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++%
        %---- Methods for setting marginals and principal variances ----%
        %+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++%
        function setMarginals(obj)
            assert(~isempty(obj.m_orientation),'estimated orientation unset. have you run an optimizer?');
            fprintf('attempting to retrieve marginals: ');
            try
                obj.m_marginals=gtsam.Marginals(obj.m_graph,obj.m_estimate);
            catch mException
                if isIndeterminantLinearSystemException(mException)
                    fprintf('\texception in setMarginals(): indeterminant system, could not get marginals\n');
                else
                    error('what kind of error is this?');
                end
            end
            if ~isempty(obj.m_marginals) % you got the marginals, now set principal variances of estimated variables
                fprintf('\tsuccess! setting principal variances and exiting.\n');
                setPosePrincipalVariance(obj);
                setVelocityPrincipalVariance(obj);
                setBiasPrincipalVariance(obj);
            end
        end % setMarginals(obj)
        function setPosePrincipalVariance(obj)
            try
                orientationPrinVar=zeros(length(obj.m_orientation),3);
                positionPrinVar=orientationPrinVar;
                for k=1:length(obj.m_orientation)
                    m=obj.m_marginals.marginalCovariance(gtsam.symbol(obj.m_poseVarChar,k-1));
                    orientationPrinVar(k,1:3)=[m(1,1) m(2,2) m(3,3)];
                    positionPrinVar(k,1:3)=[m(4,4) m(5,5) m(6,6)];
                end
                obj.m_orientationPrincipalVariance=orientationPrinVar;
                obj.m_positionPrincipalVariance=positionPrinVar;
            catch mException
                if isIndeterminantLinearSystemException(mException)
                    fprintf('\texception in setPosePrincipalVariance(): indeterminant system, could not get covariances\n');
                else
                    error('what kind of error is this?');
                end
            end
        end % setTPosePrincipalVariance(obj)
        function setVelocityPrincipalVariance(obj)
            try
                velocityPrinVar=zeros(length(obj.m_velocity),3);
                for k=1:length(obj.m_velocity)
                    m=obj.m_marginals.marginalCovariance(gtsam.symbol(obj.m_velVarChar,k-1));
                    velocityPrinVar(k,1:3)=[m(1,1) m(2,2) m(3,3)];
                end
                obj.m_velocityPrincipalVariance=velocityPrinVar;
            catch mException
                if isIndeterminantLinearSystemException(mException)
                    fprintf('\texception in setVelocityPrincipalVariance(): indeterminant system, could not get covariances\n');
                else
                    error('what kind of error is this?');
                end
            end
        end % setVelocityPrincipalVariance(obj)
        function setBiasPrincipalVariance(obj)
            try
                imuBiasPrinVar=zeros(size(obj.m_gyroBias,1),6);
                for k=1:length(size(obj.m_gyroBias,1))
                    m=obj.m_marginals.marginalCovariance(gtsam.symbol(obj.m_imuBiasVarChar,k-1));
                    imuBiasPrinVar(k,1:6)=diag(m);
                end
                obj.m_accelBiasPrincipalVariance=imuBiasPrinVar(:,1:3);
                obj.m_gyroBiasPrincipalVariance=imuBiasPrinVar(:,4:6);
            catch mException
                if isIndeterminantLinearSystemException(mException)
                    fprintf('\texception in setBiasPrincipalVariance(): indeterminant system, could not get covariances\n');
                else
                    error('what kind of error is this?');
                end
            end
        end % setBiasPrincipalVariance(obj)
        %++++++++++++++++++++++++++++++++++++++++++++%
        %---- Methods for printing data to files ----%
        %++++++++++++++++++++++++++++++++++++++++++++%
        function printYprToFile(obj,filename)
            % determine ypr from GTSAM and print to file as [time, yaw, pitch, roll]
            ypr=zeros(length(obj.m_orientation),3);
            for k=1:length(obj.m_orientation)
                v=obj.m_orientation(k).ypr();
                ypr(k,1)=v(1); ypr(k,2)=v(2); ypr(k,3)=v(3);
            end
            printMatrixDataToCsvFile(filename,[obj.m_time(:) ypr]); % write
        end % printYprToFile
        function printQuatToFile(obj,filename)
            % determine quat from GTSAM and print to file as [time, qs, qx, qy, qz]
            q=Rot3ArrayToQuaternions(obj.m_orientation);
            printMatrixDataToCsvFile(filename,[obj.m_time(:) q]); % write
        end % printQuatToFile
        function printDcmToFile(obj,filename)
            % determine dcm from GTSAM and print to file as [time, m(1,1), m(1,2), m(1,3), m(2,1), ... m(3,3)]
            dcm=zeros(length(obj.m_orientation),9);
            for k=1:length(obj.m_orientation)
                m=obj.m_orientation(k).matrix();
                dcm(k,1)=m(1,1); dcm(k,2)=m(1,2); dcm(k,3)=m(1,3); dcm(k,4)=m(2,1); dcm(k,5)=m(2,2); dcm(k,6)=m(2,3); dcm(k,7)=m(3,1); dcm(k,8)=m(3,2); dcm(k,9)=m(3,3);
            end
            printMatrixDataToCsvFile(filename,[obj.m_time(:) dcm]); % write
        end % printDcmToFile()
        function printPosToFile(obj,filename)
            % determine pos from GTSAM and print to file as [time, x, y, z]
            pos=Point3ArrayToMatrix(obj.m_position);
            printMatrixDataToCsvFile(filename,[obj.m_time(:) pos]); % write
        end % printPosToFile()
        function printVelToFile(obj,filename)
            % determine pos from GTSAM and print to file as [time, x, y, z]
            printMatrixDataToCsvFile(filename,[obj.m_time(:) obj.m_velocity]);
        end % printVelToFile()
        function printAccelBiasToFile(obj,filename)
            % determine accel bias from GTSAM and print to file as [time, x, y, z] if dynamic, [x y z] if static
            assert(~isempty(obj.m_accelBias));
            if length(obj.m_accelBias)==3 % static
                printMatrixDataToCsvFile(filename,obj.m_accelBias(:)'); % write
            else % dynamic
                printMatrixDataToCsvFile(filename,[obj.m_time(:) obj.m_accelBias]); % write
            end
        end % printAccelBiasToFile()
        function printGyroBiasToFile(obj,filename)
            % determine gyro bias from GTSAM and print to file as [time, x, y, z] if dynamic, [x y z] if static
            assert(~isempty(obj.m_gyroBias));
            if length(obj.m_gyroBias)==3 % static
                printMatrixDataToCsvFile(filename,obj.m_gyroBias(:)'); % write
            else % dynamic
                printMatrixDataToCsvFile(filename,[obj.m_time(:) obj.m_gyroBias]); % write
            end
        end % printGyroBiasToFile()
        % data checking and debugging
        function assertHeadingRotationConsistency(obj)
            % () first, remove the priors on the pose and velocity
            graph=imuPoseEstimator.copyGraphRemovePriors(obj.m_graph);
            origError=graph.error(obj.m_estimate);
            rotationsToTest=linspace(0,2*pi,8); testErr=zeros(length(rotationsToTest),1);
            for k=1:length(rotationsToTest)
                newvals=imuPoseEstimator.copyValuesRotateByHeadingAngle(obj.m_estimate,rotationsToTest(k),obj.accG,'a','b');
                testErr(k)=graph.error(newvals);
            end
            assert(all( abs(testErr-origError)<1e-5 ),'some of your heading rotations produced different errors...');
        end
        function figh=animate(obj)
            p=Point3ArrayToMatrix(obj.m_position);
            q=Rot3ArrayToQuaternions(obj.m_orientation);
            figh=animateSingleImuPose(obj.m_time,p,q,'global','animation.gif');
        end
    end % methods
    
    
    methods (Static)
        function poses=vectorizePoses(vals,poseKeys)
            % put requested type in a matlab array
            poses=repmat(gtsam.Pose3,[length(poseKeys) 1]);
            for k=1:length(poseKeys)
                poses(k)=vals.atPose3(poseKeys(k));
            end
        end
        function vel=vectorizeVelocities(vals,velKeys)
            % return as an Nx3 matrix of velocities
            vel=zeros(length(velKeys),3);
            for k=1:length(velKeys)
               vel(k,1:3)=vals.atVector(velKeys(k))'; 
            end
        end
        function figh=plotSingleImu(time,orientation,velocity,position,gyrobias,accelbias,...
                orientationPrinVar,velocityPrinVar,positionPrinVar,gyroBiasPrinVar,accelBiasPrinVar)
            % imuPoseEstimator.plotSingleImu(time,orientation,velocity,position,gyrobias,accelbias);
            % where:
            %   time = Nx1 array of times (seconds)
            %   orientation = 3x3xN DCM array in SO(3)
            %   velcocity, position, gyrobias, accelbias = Nx3 array in R(3)
            % can optionally input the principal variances as well
            q=quat_unwrap(Rot3ArrayToQuaternions(orientation));
            p=Point3ArrayToMatrix(position);
            v=velocity;
            gyroBias=gyrobias;
            accelBias=accelbias;
            % plot
            figh=figure('units','normalized','position',[.05 .05 .8 .8]);
            subplot(2,3,1); % quaternion
            imuPoseEstimator.plotQuaternionOntoAxes(gca,time,q,orientationPrinVar,'Time (sec)','Quaternion','');
            subplot(2,3,2); % position
            imuPoseEstimator.plot3VectorDataOntoAxes(gca,time,p,positionPrinVar,'Time (sec)','Position (m)','');
            subplot(2,3,3); % velocity
            imuPoseEstimator.plot3VectorDataOntoAxes(gca,time,v,velocityPrinVar,'Time (sec)','Velocity (m/s)','');
            subplot(2,3,4); hold on; % gyro bias
            if size(gyroBias,1)==1 % static
                scatter([1 2 3],gyroBias,55,'filled','k'); grid on;
                xticks([1 2 3]); xticklabels({'x','y','z'});
                xlim([.5 3.5]);
                title('static gyro bias, rad/s');
            elseif size(gyroBias,1)>1 % dynamic
                imuPoseEstimator.plot3VectorDataOntoAxes(gca,time,gyroBias,gyroBiasPrinVar,'Time (sec)','gyro bias (rad/s)','');
            end
            subplot(2,3,5); hold on; % accel bias
            if size(accelBias,1)==1 % static
                scatter([1 2 3],accelBias,55,'filled','k'); grid on;
                xticks([1 2 3]); xticklabels({'x','y','z'});
                xlim([.5 3.5]);
                title('static accel bias, m/s^2');
            elseif size(accelBias,1)>1 % dynamic
                imuPoseEstimator.plot3VectorDataOntoAxes(gca,time,accelBias,accelBiasPrinVar,'Time (sec)','accel bias (m/s^2)','');
            end
        end
        function newgraph=copyGraphRemovePriors(graph)
            newgraph=gtsam.NonlinearFactorGraph(graph);
            for k=newgraph.size():-1:1
                try
                    if strfind(class(newgraph.at(k-1)),'PriorFactor') % note that at() is zero-indexed
                        newgraph.remove(k-1); % remove it from graph
                    end
                catch mException
                    if ~strfind(mException.message,'std::bad_typeid')
                        throw(mException);
                    end
                end
            end
        end
        function newgraph=copyGraphRemoveMagnetometerFactors(graph)
            newgraph=gtsam.NonlinearFactorGraph(graph);
            for k=newgraph.size():-1:1
                try
                    if strfind(class(newgraph.at(k-1)),'MagPose3Factor') % note that at() is zero-indexed
                        newgraph.remove(k-1); % remove it from graph
                    end
                catch mException
                    if ~strfind(mException.message,'std::bad_typeid')
                        throw(mException);
                    end
                end
            end
        end
        function newvals=copyValuesRotateByHeadingAngle(vals,ang,gN,poseChar,velChar)
            newvals=gtsam.Values(vals);
            [myPose3Array,keysOfPose3sFoundUint]=getAllPose3FromValues(newvals);
            [myVelArray,keysOfVelsFoundUint]=getAllVectorFromValues(newvals,3);
            assert(length(keysOfPose3sFoundUint)==length(keysOfVelsFoundUint),'didnt find the same number of Pose3s and Vector3s...');
            dRot=quat2dcm(axang2quat([gN(:)'./sqrt(sum(gN(:)',2).^2) ang])); % rotation about grav vector
            for k=1:length(keysOfPose3sFoundUint)
                % loop through, rotate each Pose3 and Vector3 and then reset to the same key in the values
                R=myPose3Array(k).rotation().matrix(); % R[B->N]
                newR=gtsam.Rot3(dRot*R); % R[originalN->newN]*R[B->N]
                p=[myPose3Array(k).translation().x() myPose3Array(k).translation().y() myPose3Array(k).translation().z() ];
                rotatedp=[dRot*p']';
                newP=gtsam.Point3(rotatedp(1),rotatedp(2),rotatedp(3));
                newPose=gtsam.Pose3(newR,newP);
                newvals.update(keysOfPose3sFoundUint(k),newplPose);
                % now do same for velocity
                v=myVelArray(k,:);
                rotatedv=[dRot*v']';
                newvals.update(keysOfVelsFoundUint(k),rotatedv(:));
            end
        end
        
        function [magFactorErrors,magFactorErrorsWeighted]=getMagFactorPoes3ErrorResiduals(graph,vals,filterKeys)
            if ~isempty(filterKeys); assert(isa(filterKeys,'gtsam.KeyVector')); filterKeys=KeyVectorToUintKeyArray(filterKeys);  end
            fv=NonlinearFactorGraphToFactorVector(graph);
            magFactors=cell(0,0);
            for k=1:length(fv)
                if isa(fv{k},'bioslam.MagPose3Factor')
                    fvkeys=KeyVectorToUintKeyArray(fv{k}.keys);
                    if ~isempty(filterKeys) % filter by keys
                        if all(ismember(fvkeys,filterKeys)) % all of this factor's keys are in your filtered key array--this is a magFactor from the set of keys you were searching for!
                            magFactors{end+1}=fv{k};
                        end
                    else % just pull out the magFactor normally
                        magFactors{end+1}=fv{k};
                    end
                end
            end
            % () now pull out errors for each
            magFactorErrors=zeros(length(magFactors),3);
            magFactorErrorsWeighted=zeros(length(magFactors),1);
            for i=1:length(magFactors)
                imuKeyVector=magFactors{i}.keys;
                x0=vals.atPose3(imuKeyVector.at(0));
                magFactorErrors(i,1:3)=magFactors{i}.evaluateError(x0)';
                magFactorErrorsWeighted(i)=magFactors{i}.error(vals);
            end
        end % getMagFactorPoes3ErrorResiduals()
        
        function [imuFactorErrors,imuFactorErrorsWeighted]=getCombinedImuFactorErrorResiduals(graph,vals,filterKeys)
            if ~isempty(filterKeys); assert(isa(filterKeys,'gtsam.KeyVector')); filterKeys=KeyVectorToUintKeyArray(filterKeys);  end
            fv=NonlinearFactorGraphToFactorVector(graph);
            imuFactors=cell(0,0);
            for k=1:length(fv)
                if isa(fv{k},'gtsam.CombinedImuFactor')
                    fvkeys=KeyVectorToUintKeyArray(fv{k}.keys);
                    if ~isempty(filterKeys) % filter by keys
                        if all(ismember(fvkeys,filterKeys)) % all of this factor's keys are in your filtered key array--this is a ImuFactor from the set of keys you were searching for!
                            imuFactors{end+1}=fv{k};
                        end
                    else % just pull out the imufactor normally
                        imuFactors{end+1}=fv{k};
                    end
                end
            end
            % () now pull out errors for each
            imuFactorErrors=zeros(length(imuFactors),15);
            imuFactorErrorsWeighted=zeros(length(imuFactors),1);
            for i=1:length(imuFactors)
                imuKeyVector=imuFactors{i}.keys;
                x0=vals.atPose3(imuKeyVector.at(0));
                v0=vals.atVector(imuKeyVector.at(1));
                x1=vals.atPose3(imuKeyVector.at(2));
                v1=vals.atVector(imuKeyVector.at(3));
                bias0=vals.atConstantBias(imuKeyVector.at(4));
                bias1=vals.atConstantBias(imuKeyVector.at(5));
                imuFactorErrors(i,1:15)=imuFactors{i}.evaluateError(x0,v0,x1,v1,bias0,bias1)';
                imuFactorErrorsWeighted(i)=imuFactors{i}.error(vals);
            end
        end % getCombinedImuFactorErrorResiduals()
        function [imuFactorErrors,imuFactorErrorsWeighted]=getImuFactorErrorResiduals(graph,vals,filterKeys)
            if ~isempty(filterKeys); assert(isa(filterKeys,'gtsam.KeyVector')); filterKeys=KeyVectorToUintKeyArray(filterKeys);  end
            fv=NonlinearFactorGraphToFactorVector(graph);
            imuFactors=cell(0,0);
            for k=1:length(fv)
                if isa(fv{k},'gtsam.ImuFactor')
                    fvkeys=KeyVectorToUintKeyArray(fv{k}.keys);
                    if ~isempty(filterKeys) % filter by keys
                        if all(ismember(fvkeys,filterKeys)) % all of this factor's keys are in your filtered key array--this is a ImuFactor from the set of keys you were searching for!
                            imuFactors{end+1}=fv{k};
                        end
                    else % just pull out the imufactor normally
                        imuFactors{end+1}=fv{k};
                    end
                end
            end
            % () now pull out errors for each
            imuFactorErrors=zeros(length(imuFactors),9);
            imuFactorErrorsWeighted=zeros(length(imuFactors),1);
            for i=1:length(imuFactors)
                imuKeyVector=imuFactors{i}.keys;
                x0=vals.atPose3(imuKeyVector.at(0));
                v0=vals.atVector(imuKeyVector.at(1));
                x1=vals.atPose3(imuKeyVector.at(2));
                v1=vals.atVector(imuKeyVector.at(3));
                bias0=vals.atConstantBias(imuKeyVector.at(4));
                imuFactorErrors(i,1:9)=imuFactors{i}.evaluateError(x0,v0,x1,v1,bias0)';
                imuFactorErrorsWeighted(i)=imuFactors{i}.error(vals);
            end
        end
        function [priorPoseError,priorVelError,priorImuBiasError]=getPriorFactorResiduals(graph,vals)
            fv=NonlinearFactorGraphToFactorVector(graph);
            priorPose3Factors=cell(0,0);
            priorVelFactors=cell(0,0);
            priorImuBiasFactors=cell(0,0);
            for k=1:length(fv)
                if isa(fv{k},'gtsam.PriorFactorPose3')
                    priorPose3Factors{end+1}=fv{k};
                elseif isa(fv{k},'gtsam.PriorFactorVector')
                    priorVelFactors{end+1}=fv{k};
                elseif isa(fv{k},'gtsam.PriorFactorConstantBias')
                    priorImuBiasFactors{end+1}=fv{k};
                end
            end
            assert(length(priorPose3Factors)==1,'assuming only one of these priors');
            assert(length(priorVelFactors)==1,'assuming only one of these priors');
            assert(length(priorImuBiasFactors)==1,'assuming only one of these priors');
            % () now get error data
            %             poseKeyVector=priorPose3Factors{1}.keys;
            %             pose0=vals.atPose3(poseKeyVector.at(0));
            priorPoseError=priorPose3Factors{1}.error(vals);
            priorVelError=priorVelFactors{1}.error(vals);
            priorImuBiasError=priorImuBiasFactors{1}.error(vals);
        end
        function plot3VectorDataOntoAxes(ax,time,data,dataVar,xlbl,ylbl,ttl)
            % takes in handle to axes to plot onto, time, 3-data, and optionally error bar to plot (dataVar)
            % by default plots in red, green, blue for x, y, z
            assert(size(data,2)==3,'must pass in Nx3 data!');
            hold on;
            if size(dataVar,1)==size(data,1) % you passed in variances!
                assert(size(dataVar,2)==3,'must pass in Nx3 variance!');
                hx=shadedErrorBar(time,data(:,1),dataVar(:,1)','lineprops',{'color','r','linewidth',2},'patchSaturation',0.04);
                hy=shadedErrorBar(time,data(:,2),dataVar(:,2)','lineprops',{'color',[0 .4 0],'linewidth',2},'patchSaturation',0.06);
                hz=shadedErrorBar(time,data(:,3),dataVar(:,3)','lineprops',{'color','b','linewidth',2},'patchSaturation',0.05);
                set(hx.edge,'linewidth',1,'linestyle',':'); set(hy.edge,'linewidth',1,'linestyle',':'); set(hz.edge,'linewidth',1,'linestyle',':');
            elseif isempty(dataVar) % plot normal data only
                plot(time,data(:,1),'linewidth',2,'color','r');
                plot(time,data(:,2),'linewidth',2,'color',[0 .4 0]);
                plot(time,data(:,3),'linewidth',2,'color','b');
            else; error('what condition could this be?');
            end
            ylabel(ylbl); xlabel(xlbl); title(ttl); grid on; hold off;
            minVecData=min(min(data)); maxVecData=max(max(data));
            ylim([minVecData-0.1*abs(minVecData) maxVecData+0.1*abs(maxVecData)]);
        end % plot3VectorDataOntoAxes()
        function plotQuaternionOntoAxes(ax,time,data,dataVar,xlbl,ylbl,ttl)
            % takes in handle to axes to plot onto, time, orientation, and optionally error bar to plot (dataVar)
            % by default plots in black, red, green, blue for s, x, y, z
            assert(size(data,2)==4,'must pass in Nx4 data!');
            hold on;
            if size(dataVar,1)==size(data,1) % you passed in variances!
                [highBar,lowBar]=imuPoseEstimator.quaternionErrorBarFromOrientationVariance(data,dataVar);
                hs=shadedErrorBar(time,data(:,1),[abs(highBar(:,1)-data(:,1)), abs(lowBar(:,1)-data(:,1))]','lineprops',{'color','k','linewidth',2},'patchSaturation',0.05);
                hx=shadedErrorBar(time,data(:,2),[abs(highBar(:,2)-data(:,2)), abs(lowBar(:,2)-data(:,2))]','lineprops',{'color','r','linewidth',2},'patchSaturation',0.04);
                hy=shadedErrorBar(time,data(:,3),[abs(highBar(:,3)-data(:,3)), abs(lowBar(:,3)-data(:,3))]','lineprops',{'color',[0 .4 0],'linewidth',2},'patchSaturation',0.06);
                hz=shadedErrorBar(time,data(:,4),[abs(highBar(:,4)-data(:,4)), abs(lowBar(:,4)-data(:,4))]','lineprops',{'color','b','linewidth',2},'patchSaturation',0.05);
                set(hx.edge,'linewidth',1,'linestyle',':'); set(hy.edge,'linewidth',1,'linestyle',':'); set(hz.edge,'linewidth',1,'linestyle',':'); set(hs.edge,'linewidth',1,'linestyle',':');
            elseif isempty(dataVar) % plot normal data only
                plot(time,data(:,1),'linewidth',2,'color','k');
                plot(time,data(:,2),'linewidth',2,'color','r');
                plot(time,data(:,3),'linewidth',2,'color',[0 .4 0]);
                plot(time,data(:,4),'linewidth',2,'color','b');
            else; error('what condition could this be?');
            end
            ylabel(ylbl); xlabel(xlbl); title(ttl); grid on; hold off; ylim([-1.01 1.01]);
        end % plot3QuaternionOntoAxes()
        function [highBar,lowBar]=quaternionErrorBarFromOrientationVariance(q,v)
            % takes in variance v and projects it as an infinitesimal bar from quaternion q
            assert(size(q,1)==size(v,1),'should be the same length!');
            N=size(q,1);
            if size(v,1)==N && size(v,2)==3 % passed in Nx3 Rodriguez rotation variances
                % () loop through variances, converting each to an infinitesimal quaternion
                mag=sqrt(sum(v.^2,2)); % magnitude of rotation
                dq=axang2quat([v./mag,mag]);
                % () compose this infeinitesimal quaternion with the estimated quaternion q
                qbar1=quatmultiply(q,dq);
                qbar2=quatmultiply(q,quatinv(dq));
                % () now sort these into the highbar and the lowbar
                tmp=zeros(N,4,2); tmp(:,:,1)=qbar1; tmp(:,:,2)=qbar2;
                highBar=max(tmp,[],3); lowBar=min(tmp,[],3);
            else; error('pass in the orientation variance in a standard form');
            end
        end % [highBar,lowBar]=quaternionErrorBarFromOrientationVariance(q,v)
    end
end % classdef

function figh=onlinePlot(figh,myTime,q,p,v,gyroBias,accelBias,errorPerIteration,timePerIteration)
% stores handles to plotted lines in figh.UserData for easy access
if isempty(figh) % create new figure
    figh=figure('units','normalized','position',[.05 .05 .8 .8]);
    subplot(2,3,1); % orientation plot
    figh.UserData.quath(1)=line('XData',myTime,'YData',q(:,1),'Color','k');
    figh.UserData.quath(2)=line('XData',myTime,'YData',q(:,2),'Color','r');
    figh.UserData.quath(3)=line('XData',myTime,'YData',q(:,3),'Color',[0 .4 0]);
    figh.UserData.quath(4)=line('XData',myTime,'YData',q(:,4),'Color','b'); grid on;
    ylim([-1.01 1.01]); ylabel('quaternion');
    subplot(2,3,2);
    figh.UserData.posh(1)=line('XData',myTime,'YData',p(:,1),'Color','r');
    figh.UserData.posh(2)=line('XData',myTime,'YData',p(:,2),'Color',[0 .4 0]);
    figh.UserData.posh(3)=line('XData',myTime,'YData',p(:,3),'Color','b'); grid on; ylabel('pos (m)');
    subplot(2,3,3);
    figh.UserData.velh(1)=line('XData',myTime,'YData',v(:,1),'Color','r');
    figh.UserData.velh(2)=line('XData',myTime,'YData',v(:,2),'Color',[0 .4 0]);
    figh.UserData.velh(3)=line('XData',myTime,'YData',v(:,3),'Color','b'); grid on; ylabel('vel (m/s)');
    subplot(2,3,4); hold on; % gyrobias
    if all(size(gyroBias)==[1 3]) % static bias
        figh.UserData.gyrobiash=scatter([1 2 3],gyroBias,55,'filled','k'); grid on;
        xticks([1 2 3]); xticklabels({'x','y','z'}); xlim([.5 3.5]); title('static gyro bias, rad/s');
    else % dynamic bias
        figh.UserData.gyrobiash(1)=line('XData',myTime,'YData',gyroBias(:,1),'Color','r');
        figh.UserData.gyrobiash(2)=line('XData',myTime,'YData',gyroBias(:,2),'Color',[0 .4 0]);
        figh.UserData.gyrobiash(3)=line('XData',myTime,'YData',gyroBias(:,3),'Color','b'); grid on; ylabel('gyro bias (rad/s)');
    end
    subplot(2,3,5); hold on; % accelbias
    if all(size(accelBias)==[1 3]) % static bias
        figh.UserData.accelbiash=scatter([1 2 3],accelBias,55,'filled','k'); grid on;
        xticks([1 2 3]); xticklabels({'x','y','z'}); xlim([.5 3.5]); title('static accel bias, rad/s');
    else % dynamic bias
        figh.UserData.accelbiash(1)=line('XData',myTime,'YData',accelBias(:,1),'Color','r');
        figh.UserData.accelbiash(2)=line('XData',myTime,'YData',accelBias(:,2),'Color',[0 .4 0]);
        figh.UserData.accelbiash(3)=line('XData',myTime,'YData',accelBias(:,3),'Color','b'); grid on; ylabel('accel bias (m/s^2)');
    end
    subplot(2,3,6); % error plot over iterations
    yyaxis left; set(gca,'ycolor','k');
    figh.UserData.errh=line('XData',[1:length(errorPerIteration)]-1,'YData',errorPerIteration);
    grid on; set(gca,'YScale','log'); xlabel('iterations'); ylabel('error');
    yyaxis right; ylabel('total time (sec)');
    figh.UserData.timeh=line('XData',[1:length(timePerIteration)]-1,'YData',timePerIteration,'Color',[ 0.9100 0.4100 0.1700],'LineStyle','--');
else % already had figure. update values now.
    figh.UserData.quath(1).YData=q(:,1); figh.UserData.quath(2).YData=q(:,2); figh.UserData.quath(3).YData=q(:,3); figh.UserData.quath(4).YData=q(:,4);
    figh.UserData.posh(1).YData=p(:,1); figh.UserData.posh(2).YData=p(:,2); figh.UserData.posh(3).YData=p(:,3);
    figh.UserData.velh(1).YData=v(:,1); figh.UserData.velh(2).YData=v(:,2); figh.UserData.velh(3).YData=v(:,3);
    figh.UserData.errh.XData=[1:length(errorPerIteration)]-1;
    figh.UserData.errh.YData=errorPerIteration;
    figh.UserData.timeh.XData=[1:length(timePerIteration)]-1;
    figh.UserData.timeh.YData=timePerIteration;
    if all(size(gyroBias)==[1 3]) % static bias
        figh.UserData.gyrobiash.YData=gyroBias;
    else
        figh.UserData.gyrobiash(1).YData=gyroBias(:,1); figh.UserData.gyrobiash(2).YData=gyroBias(:,2); figh.UserData.gyrobiash(3).YData=gyroBias(:,3);
    end
    if all(size(accelBias)==[1 3]) % static bias
        figh.UserData.accelbiash.YData=accelBias;
    else
        figh.UserData.accelbiash(1).YData=accelBias(:,1); figh.UserData.accelbiash(2).YData=accelBias(:,2); figh.UserData.accelbiash(3).YData=accelBias(:,3);
    end
end
drawnow;
end

function instrumentedDebugHessian(hes,currentLinGraph,nonlinGraph,vals)
[factorRows,factorDescrip,varCols,varCell]=parseFactorAndVariableOrderingInformation(currentLinGraph,nonlinGraph,vals);
visualizeColoredJacobian(hes,'Hessian',factorRows,factorDescrip,varCols,varCell);
end

function instrumentedDebugJacobian(jac,currentLinGraph,nonlinGraph,vals)
[factorRows,factorDescrip,varCols,varCell]=parseFactorAndVariableOrderingInformation(currentLinGraph,nonlinGraph,vals);
% ()
visualizeColoredJacobian(jac,'Jacobian',factorRows,factorDescrip,varCols,varCell);
end

function [factorRows,factorDescrip,varCols,varCell]=parseFactorAndVariableOrderingInformation(currentLinGraph,nonlinGraph,vals)
factors=graphToFactorVector(currentLinGraph); % array of JacobianFactors
for k=1:length(factors)
    factorRows(k)=factors(k).rows;
    factorCols(k)=factors(k).cols;
end
% () get the variable ordering and convert it to a char vector
vars=nonlinGraph.orderingCOLAMD;
varCell=orderingPrintToCellArray(vars);
determineVarTypeFromValuesAndKey(vals,varCell);
% () figure out how many columns each variable takes up (i.e., what is the size of each var)
%%%%% NOTE THIS IS TEMPORARY. for now I'm hardcoding this example.
% I think you could get these from nonlinGraph.at(k).dim, assuming that they're in the same order
varTypes=determineVarTypeFromValuesAndKey(vals,varCell);
varCols=varTypeToVarSize(varTypes);
% () now get description strings for each factor
factorDescrip=cell(length(factors),1);
for k=1:length(factors)
    factorDescrip{k}=parseFactorIntoDescriptionString(nonlinGraph.at(k-1));
end
end

function varCell=orderingPrintToCellArray(vars)
% parses an ordering object into a cell array of strings representing each variable, in order.
rawStr=evalc('vars');
r=regexp(rawStr,'([a-zA-Z]{1,1}\d{1,100})','tokenExtents');
varCell=cell(1,length(r));
for k=1:length(r)
    varCell{k}=rawStr(r{k}(1):r{k}(2));
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Some artwork describing how IMU measurement relate to keyframes.
% say you are preintegrating 3 IMU measurements with measurement dt=0.05 sec
%
%   time (seconds):   (0)   (0.05)   (0.1)   (0.15)  (0.2)   (0.25)  (0.3)   (0.35)
% IMU Measurements:   [1] --- [2] --- [3] --- [4] --- [5] --- [6] --- [7] --- [8]
% Keyframe number :   [1]-----(preint fac)----[2]-----(preint fac)----[3]
%
% and we ignore any measurement less than nPreint on the end of the measurement chain
% therefore:
%       number of keyframes in graph = floor(numMeas/nPreint)+1 // (because num/Meas counts the preintegration intervals. add 1 to get the num of keyframes)
%       keyframe dt = nPreint*IMU measurement dt
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%