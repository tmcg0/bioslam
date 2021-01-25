% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function relYawAngle=relativeYawAngle(rotA,rotB)
% rotA and rotB are arrays of gtsam.Rot3 objects
% assumes input rotations are body to nav
% assumes Z is the global vertical
assert(length(rotA)==length(rotB));
% --- settings --- %
doPlot=0; % debug only
calcMode=2; % 1 for MATLAB's dcm2angle(), 2 for manual vector angles
% ---------------- %
if calcMode==1
    % () compute relative DCM
    R_A_to_B=zeros(3,3,length(rotA));
    R_N_to_A=zeros(3,3,length(rotA));
    R_N_to_B=zeros(3,3,length(rotB));
    for k=1:length(rotA)
        R_N_to_A(:,:,k)=rotA(k).inverse().matrix();
        R_N_to_B(:,:,k)=rotB(k).inverse().matrix();
        R_A_to_B(:,:,k)=rotA(k).inverse().matrix()*rotB(k).matrix(); % R[A->B]=R[A->N]'*R[B->N]
    end
    % () turn that into relative euler angles
    [relZ,relY,relX]=dcm2angle(R_A_to_B,'zyx');
    % () get difference in absolute yaw angles
    % note: we use N->B convention here that way the z euler angle properly
    % represents yaw (otherweise if you went B->N it would depend on the mount
    % of the imu)
    [aZ,aY,aX]=dcm2angle(R_N_to_A,'zyx');
    [bZ,bY,bX]=dcm2angle(R_N_to_B,'zyx');
    if doPlot
        figure; % relative yaw angle
        plot(relZ); hold on;
        plot(relY); plot(relX); grid on;
        legend('z','y','x'); title('RELATIVE z,y,x euler angles between IMU frames');
        %     figure; % absolute yaw angle difference
    end
    % () in absolute angles, rel yaw angle is the yawB minus yawA
    relYawAngle=bZ-aZ;
elseif calcMode==2
    % pick an arbitrary vector in the two IMU frames (same vector), rotate it to nav frame
    % cross it with the +vertical (+z) vector to create a vector in the horizontal plane
    % find signed angle between these two vectors
    v=[0 1 0]; % for the treadmill study, the y vector was typically near horizontal
    nZ=[0 0 1]; % vert vector
    vAN=zeros(length(rotA),3); vBN=zeros(length(rotB),3);
    hAN=zeros(length(rotA),3); hBN=zeros(length(rotB),3); % horizontal plane vectors
    ang=zeros(length(rotA),1);
    for k=1:length(rotA)
        vAN(k,:)= (rotA(k).matrix()*v')';
        vBN(k,:)= (rotB(k).matrix()*v')';
        hAN(k,:)=cross(nZ,vAN(k,:));
        hBN(k,:)=cross(nZ,vBN(k,:));
        ang(k)=signedAngle(hAN(k,:),hBN(k,:),nZ);
    end
    relYawAngle=ang;
    if doPlot
       figure; plot(relYawAngle); grid on; ylabel('angle between vectors in hor. plane, rad'); 
    end
else; error('unknown calcmode');
    
end
end

