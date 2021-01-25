% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function [distalImuPose,distalImuVel] = adjustDistalImuTrajectoryForInboundJointAngles(R_ProxSeg_to_N,proxImuPose,kneeAxisProx,distalImuPose,distalImuVel,distalImuToProxJointCtr,distalImuToDistalJointCtr,kneeAxisDistal,flexExMaxMin,abAdMaxMin,intExtRotMaxMin)
% --- settings ----
overcorrectionAngle=deg2rad(2); % how much to "overcorrect" by when moving an anatomical angle inbounds
doubleCheckAllAnglesInBounds=1; % at end, go back and make sure everything is in bounds?
zeroMedianCenterAbAd=1;
zeroMedianCenterIntExtRot=1;
debugPlots=0;
% -----------------
assert(isa(R_ProxSeg_to_N,'numeric')); assert(size(R_ProxSeg_to_N,1)==3); assert(size(R_ProxSeg_to_N,2)==3);
isAngleOutOfBounds=zeros(length(distalImuPose),1);
N=length(proxImuPose);
% () calculate original angles
R_DistalSeg_to_N = get_R_Segment_to_N(distalImuPose, kneeAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr,true);
[flexionAngleRadOrig,adductionAngleRadOrig,externalRotationAngleRadOrig]=consistent3DofJcsAngles(R_ProxSeg_to_N, R_DistalSeg_to_N);
% uncomment next line to plot
% figure; plot(flexionAngleRadOrig); hold on; plot(adductionAngleRadOrig); plot(externalRotationAngleRadOrig); grid on; legend('flex/ex','abad','ext rot');
% () apriori centering of median int/ext rotation angle to zero
if zeroMedianCenterIntExtRot
    medianIntExtRotAngle=median(externalRotationAngleRadOrig); % get median
    [~,medIdx]=min(abs(externalRotationAngleRadOrig-medianIntExtRotAngle)); % find which index in the angle is closest to this median angle
    % find R_B_to_B2 necessary to make the int/ext rot angle at medIndex = 0
    [~,R_B_to_B2]=adjustDistalImuOrientationGivenDesiredIntExtRotAngle(R_ProxSeg_to_N(:,:,medIdx),distalImuPose(medIdx),0, kneeAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr);
    % apply this rotation to all distal imu orientations
    for k=1:N % apply R[B->B2] to IMU trajectory
        R_newDistalImu_to_N=(R_B_to_B2*distalImuPose(k).rotation().matrix()')'; % R[B2->N]=(R[B->B2]*R[B->N]')' % new orientation R[B2->N]
        distalImuPose(k)=gtsam.Pose3(gtsam.Rot3(R_newDistalImu_to_N),distalImuPose(k).translation());
    end
    % assert that the new median is approximately zero
    R_DistalSeg_to_N = get_R_Segment_to_N(distalImuPose, kneeAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr,true);
    [~,~,externalRotationAngleRad2]=consistent3DofJcsAngles(R_ProxSeg_to_N, R_DistalSeg_to_N);
    if abs(median(externalRotationAngleRad2))>0.1
       warning('int/ext rot angle (after adjustment) is still out of bounds (median=%2f rad)\n',median(externalRotationAngleRad2));
    end
end
% uncomment for intermediate plot
% R_DistalSeg_to_N = get_R_Segment_to_N(distalImuPose, kneeAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr,true);
% [flexionAngleRadOrig2,adductionAngleRadOrig2,externalRotationAngleRadOrig2]=consistent3DofJcsAngles(R_ProxSeg_to_N, R_DistalSeg_to_N);
% figure; plot(flexionAngleRadOrig2); hold on; plot(adductionAngleRadOrig2); plot(externalRotationAngleRadOrig2); grid on; legend('flex/ex','abad','ext rot'); title('after int/ext rot median zero');
% () apriori centering of median ab/ad angle to zero
if zeroMedianCenterAbAd
    medianAbAdAngle=median(adductionAngleRadOrig); % get median
    [~,medIdx]=min(abs(adductionAngleRadOrig-medianAbAdAngle)); % find which index in the angle is closest to this median angle
    % find R_B_to_B2 necessary to make the ab/ad angle at medIndex = 0
    [~,R_B_to_B2]=adjustDistalImuOrientationGivenDesiredAbAdAngle(R_ProxSeg_to_N(:,:,medIdx),distalImuPose(medIdx),0, kneeAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr);
    % apply this rotation to all distal imu orientations
    for k=1:N % apply R[B->B2] to IMU trajectory
        R_newDistalImu_to_N=(R_B_to_B2*distalImuPose(k).rotation().matrix()')'; % R[B2->N]=(R[B->B2]*R[B->N]')' % new orientation R[B2->N]
        distalImuPose(k)=gtsam.Pose3(gtsam.Rot3(R_newDistalImu_to_N),distalImuPose(k).translation());
    end
    % assert that the new median is approximately zero
    R_DistalSeg_to_N = get_R_Segment_to_N(distalImuPose, kneeAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr,true);
    [~,abAdAng2,~]=consistent3DofJcsAngles(R_ProxSeg_to_N, R_DistalSeg_to_N);
    if abs(median(abAdAng2))>0.1
       warning('ab/ad angle (after adjustment) is still out of bounds (median=%2f rad)\n',median(abAdAng2));
    end
end
% uncomment for intermediate plot
% R_DistalSeg_to_N = get_R_Segment_to_N(distalImuPose, kneeAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr,true);
% [flexionAngleRadOrig3,adductionAngleRadOrig3,externalRotationAngleRadOrig3]=consistent3DofJcsAngles(R_ProxSeg_to_N, R_DistalSeg_to_N);
% figure; plot(flexionAngleRadOrig3); hold on; plot(adductionAngleRadOrig3); plot(externalRotationAngleRadOrig3); grid on; legend('flex/ex','abad','ext rot'); title('after int/ext rot and ab/ad median zero');
% go
for k=1:N
%     fprintf('outer iteration %d/%d\n',k,N);
    % () calculate R segment to N
    R_DistalSeg_to_N = get_R_Segment_to_N(distalImuPose(k).rotation(), kneeAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr,true);
    % () calculate joint angles
    [flexionAngleRad,adductionAngleRad,externalRotationAngleRad]=consistent3DofJcsAngles(R_ProxSeg_to_N(:,:,k), R_DistalSeg_to_N);
    % () if int/ext rot out of bounds, correct entire IMU trajectory k:N
    if externalRotationAngleRad>intExtRotMaxMin(1) % knee is too extended
        desiredIntExtRotAngle=intExtRotMaxMin(1)-overcorrectionAngle;
    elseif externalRotationAngleRad<intExtRotMaxMin(2) % knee is too flexed
        desiredIntExtRotAngle=intExtRotMaxMin(2)+overcorrectionAngle;
    end
    if externalRotationAngleRad>intExtRotMaxMin(1) || externalRotationAngleRad<intExtRotMaxMin(2)
        [~,R_B_to_B2]=adjustDistalImuOrientationGivenDesiredIntExtRotAngle(R_ProxSeg_to_N(:,:,k),distalImuPose(k),desiredIntExtRotAngle, kneeAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr);
        for j=k:N % apply R[B->B2] to IMU trajectory
            R_newDistalImu_to_N=distalImuPose(j).rotation().matrix()*R_B_to_B2'; % R[B2->N]=R[B->N]*R[B->B2]'
            distalImuPose(j)=gtsam.Pose3(gtsam.Rot3(R_newDistalImu_to_N),distalImuPose(j).translation());
        end
    end
    % () if flexion/extension out of bounds, correct entire IMU trajectory k:N
    if flexionAngleRad>flexExMaxMin(1) % knee is too extended
        desiredFlexExAngle=flexExMaxMin(1)-overcorrectionAngle;
    elseif flexionAngleRad<flexExMaxMin(2) % knee is too flexed
        desiredFlexExAngle=flexExMaxMin(2)+overcorrectionAngle;
    end
    if flexionAngleRad>flexExMaxMin(1) || flexionAngleRad<flexExMaxMin(2)
        [~,R_B_to_B2]=adjustDistalImuOrientationGivenDesiredFlexExAngle(R_ProxSeg_to_N(:,:,k),proxImuPose(k),distalImuPose(k),desiredFlexExAngle, kneeAxisProx,kneeAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr);
        for j=k:N % apply R[B->B2] to IMU trajectory
            R_newDistalImu_to_N=distalImuPose(j).rotation().matrix()*R_B_to_B2'; % R[B2->N]=R[B->N]*R[B->B2]'
            distalImuPose(j)=gtsam.Pose3(gtsam.Rot3(R_newDistalImu_to_N),distalImuPose(j).translation());
        end
    end
    % () if ab/ad out of bounds, correct entire IMU trajectory k:N
    if adductionAngleRad>abAdMaxMin(1) % too toe to left (ab/ad sense)
        desiredAbAdAngle=abAdMaxMin(1)-overcorrectionAngle;
    elseif adductionAngleRad<abAdMaxMin(2) % too toe to right (ab/ad sense)
        desiredAbAdAngle=abAdMaxMin(2)+overcorrectionAngle;
    end
    if adductionAngleRad>abAdMaxMin(1) || adductionAngleRad<abAdMaxMin(2)
        [~,R_B_to_B2]=adjustDistalImuOrientationGivenDesiredAbAdAngle(R_ProxSeg_to_N(:,:,k),distalImuPose(k),desiredAbAdAngle, kneeAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr);
        for j=k:N % apply R[B->B2] to IMU trajectory
            R_newDistalImu_to_N=distalImuPose(j).rotation().matrix()*R_B_to_B2'; % R[B2->N]=R[B->N]*R[B->B2]'
            distalImuPose(j)=gtsam.Pose3(gtsam.Rot3(R_newDistalImu_to_N),distalImuPose(j).translation());
        end
    end
end
% calculate new angles
% () calculate R segment to N
R_DistalSeg_to_N = get_R_Segment_to_N(distalImuPose, kneeAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr,true);
% () calculate joint angles
[flexionAngleRadNew,adductionAngleRadNew,externalRotationAngleRadNew]=consistent3DofJcsAngles(R_ProxSeg_to_N, R_DistalSeg_to_N);
% () optional: loop back through and check that all angles are in bounds
if doubleCheckAllAnglesInBounds
    for k=1:N
        if flexionAngleRadNew(k)>flexExMaxMin(1); error('angle out of bounds!'); end
        if flexionAngleRadNew(k)<flexExMaxMin(2); error('angle out of bounds!'); end
        if adductionAngleRadNew(k)>abAdMaxMin(1); error('angle out of bounds!'); end
        if adductionAngleRadNew(k)<abAdMaxMin(2); error('angle out of bounds!'); end
        if externalRotationAngleRadNew(k)>intExtRotMaxMin(1); error('angle out of bounds!'); end
        if externalRotationAngleRadNew(k)<intExtRotMaxMin(2); error('angle out of bounds!'); end
    end
end
% () optional: do debug plots
if debugPlots
    figh=figure('units','normalized','position',[.1 .1 .8 .8]);
    subplot(3,1,1); hold on;
    plot(rad2deg(flexionAngleRadNew),'k'); plot(rad2deg(flexionAngleRadOrig),'b--'); grid on;
    plot([1 length(flexionAngleRadNew)],rad2deg([flexExMaxMin(1) flexExMaxMin(1)]),'r--');
    plot([1 length(flexionAngleRadNew)],rad2deg([flexExMaxMin(2) flexExMaxMin(2)]),'r--');
    ylabel('flexion/extension');
    subplot(3,1,2); hold on;
    plot(rad2deg(adductionAngleRadNew),'k'); plot(rad2deg(adductionAngleRadOrig),'b--'); grid on;
    plot([1 length(flexionAngleRadNew)],rad2deg([abAdMaxMin(1) abAdMaxMin(1)]),'r--');
    plot([1 length(flexionAngleRadNew)],rad2deg([abAdMaxMin(2) abAdMaxMin(2)]),'r--');
    ylabel('ab/ad');
    subplot(3,1,3); hold on;
    plot(rad2deg(externalRotationAngleRadNew),'k'); plot(rad2deg(externalRotationAngleRadOrig),'b--'); grid on;
    plot([1 length(flexionAngleRadNew)],rad2deg([intExtRotMaxMin(1) intExtRotMaxMin(1)]),'r--');
    plot([1 length(flexionAngleRadNew)],rad2deg([intExtRotMaxMin(2) intExtRotMaxMin(2)]),'r--');
    ylabel('int/ext rot');
end
end

function unitTestAxisRotation(R_B_to_N,axisB)
% you have a set of orientations R_B_to_N
% you have some axis statically represented in the body frame you want to spin the coordinate system about
randSpinAngles=rand(length(R_B_to_N),1);
for k=1:length(R_B_to_N)
    origAxisN(k,:)=(R_B_to_N(k).matrix()*axisB(:))';
end
for k=1:length(R_B_to_N)
    axang=[axisB(:)',randSpinAngles(k)]; % R[B->B2]
    deltaRot=gtsam.Rot3(axang2rotm(axang)); % R[B->B2]
    R_B2_to_N(k)=gtsam.Rot3((deltaRot.matrix()*R_B_to_N(k).inverse().matrix())');
end
for k=1:length(R_B_to_N)
    newAxisN(k,:)=(R_B2_to_N(k).matrix()*axisB(:))';
end
assert(all(all((origAxisN-newAxisN)<1.0e-10)));
end

function unitTestRotateFlexionAngle(proxImuPose,proxImuToProxJointCtr,proxImuToDistalJointCtr,kneeAxisProx,distalImuPose,distalImuToProxJointCtr,distalImuToDistalJointCtr,kneeAxisDistal)
% ensure that you can change the distal imu orientation and it only affect one of the DOF of the knee
% () calculate original angles
% () calculate R segment to N
R_ProxSeg_to_N = get_R_Segment_to_N(proxImuPose.rotation(), kneeAxisProx, proxImuToProxJointCtr, proxImuToDistalJointCtr,true);
R_DistalSeg_to_N = get_R_Segment_to_N(distalImuPose.rotation(), kneeAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr,true);
[flexionAngleRadOrig,adductionAngleRadOrig,externalRotationAngleRadOrig]=consistent3DofJcsAngles(R_ProxSeg_to_N.matrix(), R_DistalSeg_to_N.matrix());
% () now change distal imu orientation
R_B_to_N=distalImuPose.rotation(); % R[B->N], original orientation
% flexion occurs about proximal frame axis, so first find it in distal frame and then rotate about it
kneeAxisProxInDistalFrame=distalImuPose.rotation().inverse().matrix()*proxImuPose.rotation().matrix()*kneeAxisProx.point3().vector(); % vA[B]= R_B_to_N'*R_A_to_N*vA[A]
axang=[kneeAxisProxInDistalFrame(:)',rand(1,1)]; % R[B->B2]
deltaRot=gtsam.Rot3(axang2rotm(axang)); % R[B->B2]
R_B2_to_N=gtsam.Rot3((deltaRot.matrix()*R_B_to_N.inverse().matrix())'); % adjusted orientation
distalImuPose=gtsam.Pose3(R_B2_to_N,distalImuPose.translation()); % update
% () recalculate angles
R_ProxSeg_to_N = get_R_Segment_to_N(proxImuPose.rotation(), kneeAxisProx, proxImuToProxJointCtr, proxImuToDistalJointCtr,true);
R_DistalSeg_to_N = get_R_Segment_to_N(distalImuPose.rotation(), kneeAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr,true);
[flexionAngleRadNew,adductionAngleRadNew,externalRotationAngleRadNew]=consistent3DofJcsAngles(R_ProxSeg_to_N.matrix(), R_DistalSeg_to_N.matrix());
% () ensure only flexion has changed
assert(abs(flexionAngleRadOrig-flexionAngleRadNew)>1.0e-5);
assert(abs(adductionAngleRadOrig-adductionAngleRadNew)<1.0e-4);
assert(abs(externalRotationAngleRadOrig-externalRotationAngleRadNew)<1.0e-4);
end

function unitTestRotateExternalRotAngle(proxImuPose,proxImuToProxJointCtr,proxImuToDistalJointCtr,kneeAxisProx,distalImuPose,distalImuToProxJointCtr,distalImuToDistalJointCtr,kneeAxisDistal)
% ensure that you can change the distal imu orientation and it only affect one of the DOF of the knee
% () calculate original angles
% () calculate R segment to N
R_ProxSeg_to_N = get_R_Segment_to_N(proxImuPose.rotation(), kneeAxisProx, proxImuToProxJointCtr, proxImuToDistalJointCtr,true);
R_DistalSeg_to_N = get_R_Segment_to_N(distalImuPose.rotation(), kneeAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr,true);
[flexionAngleRadOrig,adductionAngleRadOrig,externalRotationAngleRadOrig]=consistent3DofJcsAngles(R_ProxSeg_to_N.matrix(), R_DistalSeg_to_N.matrix());
% () now change distal imu orientation
R_B_to_N=distalImuPose.rotation(); % R[B->N], original orientation
% external/internal rotation occurs about proximal vector of the distal imu frame
proxVecDistalFrame=distalImuToProxJointCtr.vector()-distalImuToDistalJointCtr.vector(); proxVecDistalFrame=proxVecDistalFrame/sqrt(sum(proxVecDistalFrame.^2,1));
axang=[proxVecDistalFrame(:)',rand(1,1)]; % R[B->B2]
deltaRot=gtsam.Rot3(axang2rotm(axang)); % R[B->B2]
R_B2_to_N=gtsam.Rot3((deltaRot.matrix()*R_B_to_N.inverse().matrix())'); % adjusted orientation
distalImuPose=gtsam.Pose3(R_B2_to_N,distalImuPose.translation()); % update
% () recalculate angles
R_ProxSeg_to_N = get_R_Segment_to_N(proxImuPose.rotation(), kneeAxisProx, proxImuToProxJointCtr, proxImuToDistalJointCtr,true);
R_DistalSeg_to_N = get_R_Segment_to_N(distalImuPose.rotation(), kneeAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr,true);
[flexionAngleRadNew,adductionAngleRadNew,externalRotationAngleRadNew]=consistent3DofJcsAngles(R_ProxSeg_to_N.matrix(), R_DistalSeg_to_N.matrix());
% () ensure only external/internal rotation has changed
assert(abs(flexionAngleRadOrig-flexionAngleRadNew)<1.0e-4);
assert(abs(adductionAngleRadOrig-adductionAngleRadNew)<1.0e-4);
assert(abs(externalRotationAngleRadOrig-externalRotationAngleRadNew)>1.0e-5);
end

function unitTestRotateAdductionAngle(proxImuPose,proxImuToProxJointCtr,proxImuToDistalJointCtr,kneeAxisProx,distalImuPose,distalImuToProxJointCtr,distalImuToDistalJointCtr,kneeAxisDistal)
% ensure that you can change the distal imu orientation and it only affect one of the DOF of the knee
% () calculate original angles
% () calculate R segment to N
R_ProxSeg_to_N = get_R_Segment_to_N(proxImuPose.rotation(), kneeAxisProx, proxImuToProxJointCtr, proxImuToDistalJointCtr,true);
R_DistalSeg_to_N = get_R_Segment_to_N(distalImuPose.rotation(), kneeAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr,true);
[flexionAngleRadOrig,adductionAngleRadOrig,externalRotationAngleRadOrig]=consistent3DofJcsAngles(R_ProxSeg_to_N.matrix(), R_DistalSeg_to_N.matrix());
% () now change distal imu orientation
R_B_to_N=distalImuPose.rotation(); % R[B->N], original orientation
% adduction occurs about distal frame axis,
proxVecDistalFrame=distalImuToProxJointCtr.vector()-distalImuToDistalJointCtr.vector(); proxVecDistalFrame=proxVecDistalFrame/sqrt(sum(proxVecDistalFrame.^2,1));
anteriorVecDistalFrame=cross(proxVecDistalFrame,kneeAxisDistal.point3().vector()); anteriorVecDistalFrame=anteriorVecDistalFrame/sqrt(sum(anteriorVecDistalFrame.^2,1)); % y=cross(z,x)
axang=[anteriorVecDistalFrame(:)',rand(1,1)]; % R[B->B2]
deltaRot=gtsam.Rot3(axang2rotm(axang)); % R[B->B2]
R_B2_to_N=gtsam.Rot3((deltaRot.matrix()*R_B_to_N.inverse().matrix())'); % adjusted orientation
distalImuPose=gtsam.Pose3(R_B2_to_N,distalImuPose.translation()); % update
% () recalculate angles
R_ProxSeg_to_N = get_R_Segment_to_N(proxImuPose.rotation(), kneeAxisProx, proxImuToProxJointCtr, proxImuToDistalJointCtr,true);
R_DistalSeg_to_N = get_R_Segment_to_N(distalImuPose.rotation(), kneeAxisDistal, distalImuToProxJointCtr,distalImuToDistalJointCtr,true);
[flexionAngleRadNew,adductionAngleRadNew,externalRotationAngleRadNew]=consistent3DofJcsAngles(R_ProxSeg_to_N.matrix(), R_DistalSeg_to_N.matrix());
% () ensure only adduction has changed
assert(abs(flexionAngleRadOrig-flexionAngleRadNew)<1.0e-4);
assert(abs(adductionAngleRadOrig-adductionAngleRadNew)>1.0e-5);
assert(abs(externalRotationAngleRadOrig-externalRotationAngleRadNew)<1.0e-4);
end
