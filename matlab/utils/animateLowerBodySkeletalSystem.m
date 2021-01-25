% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function animateLowerBodySkeletalSystem(pos,Rimu,vec,Rseg,savename)
% pos, R, and vec are structs which each hold data for the full lower body pose system
% --- options --- %
coordSysLineLengthSeg=.05; % m
coordSysLineLengthImu=.06; % m
coordSysLineWidthSeg=1;
coordSysLineWidthImu=2;
nFramesToSkip=1; % number of frames to skip. 1 => no skipping.
fixSolutionPos=0; % if true, will hold sacrum IMU position constant and adjust everything else accordingly
fixedSacrumImuPosition=[0 0 1]; % if fixSolutionPos==1, this is the position to fix the sacrum IMU position to
showGlobalCoordSysFixedAtOrigin=0;
coordSysLineLengthGlobal=0.1; % m
coordSysLineWidthGlobal=2;
coordSysLineAlphaGlobal=1;
limBoxMode=2; % options: 0=none, 1=fixed (use limBoxModeFixedSizeLimits), 2=fixedFollow (updates fixed-sized box relative to sacrum IMU)
limBoxModeFixedSizeLimits=[-0.8 0.8 -0.8 0.8 -0.8 1]; % [xmin xmax ymin ymax zmin zmax]
% --------------- %
% () pull out anatomical seg orientations for ease/performance
R_Pelvis_to_N=Rseg.RPelvisToN; R_RFemur_to_N=Rseg.RRFemurToN; R_LFemur_to_N=Rseg.RLFemurToN; R_RTibia_to_N=Rseg.RRTibiaToN; R_LTibia_to_N=Rseg.RLTibiaToN;
% () pull out IMU orientations for ease/performance
R_SacrumImu_to_N=Rimu.RSacrumImuToN; R_RThighImu_to_N=Rimu.RRThighImuToN; R_LThighImu_to_N=Rimu.RLThighImuToN; R_RShankImu_to_N=Rimu.RRShankImuToN; R_LShankImu_to_N=Rimu.RLShankImuToN; R_RFootImu_to_N=Rimu.RRFootImuToN; R_LFootImu_to_N=Rimu.RLFootImuToN;
% () pull out IMU positions for ease/performance
pos_SacrumImu=pos.pSacrumImu; pos_RThighImu=pos.pRThighImu; pos_RShankImu=pos.pRShankImu; pos_RFootImu=pos.pRFootImu; pos_LThighImu=pos.pLThighImu; pos_LShankImu=pos.pLShankImu; pos_LFootImu=pos.pLFootImu;
% () if fixing sacrum IMU (fixSolutionPos==1), update data for this
if fixSolutionPos
    % get position offset
    deltaPos=repmat(fixedSacrumImuPosition,size(pos_SacrumImu,1),1)-pos_SacrumImu; % i.e., new sacrum pos = old sacrum pos + deltaPos
    % apply position offset
    pos_SacrumImu=pos_SacrumImu+deltaPos;
    pos_RThighImu=pos_RThighImu+deltaPos;
    pos_RShankImu=pos_RShankImu+deltaPos;
    pos_RFootImu=pos_RFootImu+deltaPos;
    pos_LThighImu=pos_LThighImu+deltaPos;
    pos_LShankImu=pos_LShankImu+deltaPos;
    pos_LFootImu=pos_LFootImu+deltaPos;
end
% () get joint centers according to each imu
rhipJcSac=pos_SacrumImu+rotstaticvecCompose(R_SacrumImu_to_N,vec.sacrumImuToRHipCtr);
lhipJcSac=pos_SacrumImu+rotstaticvecCompose(R_SacrumImu_to_N,vec.sacrumImuToLHipCtr);
rhipJcRT=pos_RThighImu+rotstaticvecCompose(R_RThighImu_to_N,vec.rthighImuToHipCtr);
rkneeJcRT=pos_RThighImu+rotstaticvecCompose(R_RThighImu_to_N ,vec.rthighImuToKneeCtr);
lhipJcLT=pos_LThighImu+rotstaticvecCompose(R_LThighImu_to_N ,vec.lthighImuToHipCtr);
lkneeJcLT=pos_LThighImu+rotstaticvecCompose(R_LThighImu_to_N,vec.lthighImuToKneeCtr);
rkneeJcRS=pos_RShankImu+rotstaticvecCompose(R_RShankImu_to_N,vec.rshankImuToKneeCtr);
rankleJcRS=pos_RShankImu+rotstaticvecCompose(R_RShankImu_to_N,vec.rshankImuToAnkleCtr);
rankleJcRF=pos_RFootImu+rotstaticvecCompose(R_RFootImu_to_N,vec.rfootImuToAnkleCtr);
lkneeJcLS=pos_LShankImu+rotstaticvecCompose(R_LShankImu_to_N,vec.lshankImuToKneeCtr);
lankleJcLS=pos_LShankImu+rotstaticvecCompose(R_LShankImu_to_N,vec.lshankImuToAnkleCtr);
lankleJcLF=pos_LFootImu+rotstaticvecCompose(R_LFootImu_to_N,vec.lfootImuToAnkleCtr);
% () get the position of the anatomical frames as the averages of the joint centers
pPelvis=(lhipJcSac+rhipJcSac)./2;
pRFemur=(rhipJcRT+rkneeJcRT)./2;
pLFemur=(lhipJcLT+lkneeJcLT)./2;
pRTibia=(rkneeJcRS+rankleJcRS)./2;
pLTibia=(lkneeJcLS+lankleJcLS)./2;
% () start plot
h=figure('units','normalized','position',[.1 .1 .5 .5],'NextPlot','replaceChildren');
ax=axes;
h.Color='w';
set(h,'Renderer','painters'); % painters is prettier than OpenGL
% get line handles for first index of coord sys's (true)
Pelvic_CS_lh=updateLocalCoordSysLineHandles([],1,R_Pelvis_to_N,pPelvis,coordSysLineLengthSeg,'-',1,coordSysLineWidthSeg);
RFemur_CS_lh=updateLocalCoordSysLineHandles([],1,R_RFemur_to_N,pRFemur,coordSysLineLengthSeg,'-',1,coordSysLineWidthSeg);
LFemur_CS_lh=updateLocalCoordSysLineHandles([],1,R_LFemur_to_N,pLFemur,coordSysLineLengthSeg,'-',1,coordSysLineWidthSeg);
RTibia_CS_lh=updateLocalCoordSysLineHandles([],1,R_RTibia_to_N,pRTibia,coordSysLineLengthSeg,'-',1,coordSysLineWidthSeg);
LTibia_CS_lh=updateLocalCoordSysLineHandles([],1,R_LTibia_to_N,pLTibia,coordSysLineLengthSeg,'-',1,coordSysLineWidthSeg);
% same but for IMU coordinate systems
SacrumImu_CS_lh=updateLocalCoordSysLineHandles([],1,R_SacrumImu_to_N,pos_SacrumImu,coordSysLineLengthImu,'-',1,coordSysLineWidthImu);
RThighImu_CS_lh=updateLocalCoordSysLineHandles([],1,R_RThighImu_to_N,pos_RThighImu,coordSysLineLengthImu,'-',1,coordSysLineWidthImu);
RShankImu_CS_lh=updateLocalCoordSysLineHandles([],1,R_RShankImu_to_N,pos_RShankImu,coordSysLineLengthImu,'-',1,coordSysLineWidthImu);
LThighImu_CS_lh=updateLocalCoordSysLineHandles([],1,R_LThighImu_to_N,pos_LThighImu,coordSysLineLengthImu,'-',1,coordSysLineWidthImu);
LShankImu_CS_lh=updateLocalCoordSysLineHandles([],1,R_LShankImu_to_N,pos_LShankImu,coordSysLineLengthImu,'-',1,coordSysLineWidthImu);
RFootImu_CS_lh=updateLocalCoordSysLineHandles([],1,R_RFootImu_to_N,pos_RFootImu,coordSysLineLengthImu,'-',1,coordSysLineWidthImu);
LFootImu_CS_lh=updateLocalCoordSysLineHandles([],1,R_LFootImu_to_N,pos_LFootImu,coordSysLineLengthImu,'-',1,coordSysLineWidthImu);
% get line handles for segment lines (dotted)
pelvicSegLh=updateSingleLineHandles([],1,lhipJcSac,rhipJcSac,'--',[.3 .3 .9],1);
rfemurSegLh=updateSingleLineHandles([],1,rhipJcRT,rkneeJcRT,'--',[.9 .3 .3],1);
lfemurSegLh=updateSingleLineHandles([],1,lhipJcLT,lkneeJcLT,'--',[.3 .9 .3],1);
rtibiaSegLh=updateSingleLineHandles([],1,rkneeJcRS,rankleJcRS,'--',[.7 .3 .3],1);
ltibiaSegLh=updateSingleLineHandles([],1,lkneeJcLS,lankleJcLS,'--',[.3 .7 .3],1);
% get line handles for solid lines from imu center to joint center
sacrum2LHipLh=updateSingleLineHandles([],1,pos_SacrumImu,lhipJcSac,'-',[.2 .7 .2],.7);
sacrum2RHipLh=updateSingleLineHandles([],1,pos_SacrumImu,rhipJcSac,'-',[.8 .1 .1],.7);
rthigh2HipLh=updateSingleLineHandles([],1,pos_RThighImu,rhipJcRT,'-',[.8 .2 .2],.7);
rthigh2KneeLh=updateSingleLineHandles([],1,pos_RThighImu,rkneeJcRT,'-',[.1 .5 .1],.7);
rshank2KneeLh=updateSingleLineHandles([],1,pos_RShankImu,rkneeJcRS,'-',[.8 .2 .2],.7);
rshank2AnkleLh=updateSingleLineHandles([],1,pos_RShankImu,rankleJcRS,'-',[.1 .5 .1],.7);
lthigh2HipLh=updateSingleLineHandles([],1,pos_LThighImu,lhipJcLT,'-',[.8 .2 .2],.7);
lthigh2KneeLh=updateSingleLineHandles([],1,pos_LThighImu,lkneeJcLT,'-',[.1 .5 .1],.7);
lshank2KneeLh=updateSingleLineHandles([],1,pos_LShankImu,lkneeJcLS,'-',[.8 .2 .2],.7);
lshank2AnkleLh=updateSingleLineHandles([],1,pos_LShankImu,lankleJcLS,'-',[.1 .5 .1],.7);
rfoot2AnkleLh=updateSingleLineHandles([],1,pos_RFootImu,rankleJcRF,'-',[.8 .2 .2],.7);
lfoot2AnkleLh=updateSingleLineHandles([],1,pos_LFootImu,lankleJcLF,'-',[.8 .2 .2],.7);
% () setup viewing window size/behavior
switch limBoxMode
    case 0
        axis equal;
    case 1
        a=h.Children;
        a.ALimMode='Manual'; a.XLimMode='Manual'; a.YLimMode='Manual'; a.ZLimMode='Manual';
        a.DataAspectRatio=[1 1 1]; a.PlotBoxAspectRatioMode='manual';
        xlim(limBoxModeFixedSizeLimits(1:2)); ylim(limBoxModeFixedSizeLimits(3:4)); zlim(limBoxModeFixedSizeLimits(5:6));
    case 2 % geometric center is center of camera frame
        [xlims,ylims,zlims]=geometricCenterBoundingBox(cat(3,pos_SacrumImu,pos_RThighImu,pos_RShankImu,pos_RFootImu,pos_LThighImu,pos_LShankImu,pos_LFootImu),limBoxModeFixedSizeLimits(1:2), limBoxModeFixedSizeLimits(3:4), limBoxModeFixedSizeLimits(5:6));
    case 3 % geometric center for first frame, then that is fixed to sacrum IMU afterward
        [xlims,ylims,zlims]=geometricCenterBoundingBoxFixedAtImu(pos_SacrumImu,cat(3,pos_SacrumImu,pos_RThighImu,pos_RShankImu,pos_RFootImu,pos_LThighImu,pos_LShankImu,pos_LFootImu),limBoxModeFixedSizeLimits(1:2), limBoxModeFixedSizeLimits(3:4), limBoxModeFixedSizeLimits(5:6));
    otherwise
        error('unknown limBoxMode');
end
grid on; xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); view(3); axis equal;
ax.XTick=[-5:.2:5]; ax.YTick=[-5:.2:5]; ax.ZTick=[-5:.2:5];
% optionally add global coordinate system axes
if showGlobalCoordSysFixedAtOrigin
    gcsh=updateLocalCoordSysLineHandles([],1,eye(3),[0 0 0],coordSysLineLengthGlobal,'-',coordSysLineAlphaGlobal,coordSysLineWidthGlobal);
end
% Preallocate data (for storing frame data)
% NOTE: saving animation to .gif was commented out because it was too slow.
% f = getframe(gcf);
% [samp, map] = rgb2ind(f.cdata, 256, 'nodither');
% mov = zeros(size(samp,1), size(samp,2), 1, length(pos_SacrumImu), 'uint8');
try
    for k=2:nFramesToSkip:length(pPelvis)
        % update the anatoimcal coordinate system handles
        Pelvic_CS_lh=updateLocalCoordSysLineHandles(Pelvic_CS_lh,k,R_Pelvis_to_N,pPelvis,coordSysLineLengthSeg,'-',1,coordSysLineWidthSeg);
        RFemur_CS_lh=updateLocalCoordSysLineHandles(RFemur_CS_lh,k,R_RFemur_to_N,pRFemur,coordSysLineLengthSeg,'-',1,coordSysLineWidthSeg);
        LFemur_CS_lh=updateLocalCoordSysLineHandles(LFemur_CS_lh,k,R_LFemur_to_N,pLFemur,coordSysLineLengthSeg,'-',1,coordSysLineWidthSeg);
        RTibia_CS_lh=updateLocalCoordSysLineHandles(RTibia_CS_lh,k,R_RTibia_to_N,pRTibia,coordSysLineLengthSeg,'-',1,coordSysLineWidthSeg);
        LTibia_CS_lh=updateLocalCoordSysLineHandles(LTibia_CS_lh,k,R_LTibia_to_N,pLTibia,coordSysLineLengthSeg,'-',1,coordSysLineWidthSeg);
        % update the IMU coordinate system handles
        SacrumImu_CS_lh=updateLocalCoordSysLineHandles(SacrumImu_CS_lh,k,R_SacrumImu_to_N,pos_SacrumImu,coordSysLineLengthImu,'-',1,coordSysLineWidthImu);
        RThighImu_CS_lh=updateLocalCoordSysLineHandles(RThighImu_CS_lh,k,R_RThighImu_to_N,pos_RThighImu,coordSysLineLengthImu,'-',1,coordSysLineWidthImu);
        RShankImu_CS_lh=updateLocalCoordSysLineHandles(RShankImu_CS_lh,k,R_RShankImu_to_N,pos_RShankImu,coordSysLineLengthImu,'-',1,coordSysLineWidthImu);
        LThighImu_CS_lh=updateLocalCoordSysLineHandles(LThighImu_CS_lh,k,R_LThighImu_to_N,pos_LThighImu,coordSysLineLengthImu,'-',1,coordSysLineWidthImu);
        LShankImu_CS_lh=updateLocalCoordSysLineHandles(LShankImu_CS_lh,k,R_LShankImu_to_N,pos_LShankImu,coordSysLineLengthImu,'-',1,coordSysLineWidthImu);
        RFootImu_CS_lh=updateLocalCoordSysLineHandles(RFootImu_CS_lh,k,R_RFootImu_to_N,pos_RFootImu,coordSysLineLengthImu,'-',1,coordSysLineWidthImu);
        LFootImu_CS_lh=updateLocalCoordSysLineHandles(LFootImu_CS_lh,k,R_LFootImu_to_N,pos_LFootImu,coordSysLineLengthImu,'-',1,coordSysLineWidthImu);
        % update the dotted lines for the anatomical segments
        pelvicSegLh=updateSingleLineHandles(pelvicSegLh,k,lhipJcSac,rhipJcSac,'--',[.8 .3 .5],1);
        rfemurSegLh=updateSingleLineHandles(rfemurSegLh,k,rhipJcRT,rkneeJcRT,'--',[.5 .8 .5],1);
        lfemurSegLh=updateSingleLineHandles(lfemurSegLh,k,lhipJcLT,lkneeJcLT,'--',[.5 .3 .9],1);
        rtibiaSegLh=updateSingleLineHandles(rtibiaSegLh,k,rkneeJcRS,rankleJcRS,'--',[.5 .8 .5],1);
        ltibiaSegLh=updateSingleLineHandles(ltibiaSegLh,k,lkneeJcLS,lankleJcLS,'--',[.5 .3 .9],1);
        % update the solid lines for the vectors to the joint centers
        sacrum2LHipLh=updateSingleLineHandles(sacrum2LHipLh,k,pos_SacrumImu,lhipJcSac,'-',[.9 .3 .2],1);
        sacrum2RHipLh=updateSingleLineHandles(sacrum2RHipLh,k,pos_SacrumImu,rhipJcSac,'-',[.1 .5 .1],1);
        rthigh2HipLh=updateSingleLineHandles(rthigh2HipLh,k,pos_RThighImu,rhipJcRT,'-',[.8 .2 .2],.7);
        rthigh2KneeLh=updateSingleLineHandles(rthigh2KneeLh,k,pos_RThighImu,rkneeJcRT,'-',[.1 .5 .1],.7);
        rshank2KneeLh=updateSingleLineHandles(rshank2KneeLh,k,pos_RShankImu,rkneeJcRS,'-',[.8 .2 .2],.7);
        rshank2AnkleLh=updateSingleLineHandles(rshank2AnkleLh,k,pos_RShankImu,rankleJcRS,'-',[.1 .5 .1],.7);
        lthigh2HipLh=updateSingleLineHandles(lthigh2HipLh,k,pos_LThighImu,lhipJcLT,'-',[.8 .2 .2],.7);
        lthigh2KneeLh=updateSingleLineHandles(lthigh2KneeLh,k,pos_LThighImu,lkneeJcLT,'-',[.1 .5 .1],.7);
        lshank2KneeLh=updateSingleLineHandles(lshank2KneeLh,k,pos_LShankImu,lkneeJcLS,'-',[.8 .2 .2],.7);
        lshank2AnkleLh=updateSingleLineHandles(lshank2AnkleLh,k,pos_LShankImu,lankleJcLS,'-',[.1 .5 .1],.7);
        rfoot2AnkleLh=updateSingleLineHandles(rfoot2AnkleLh,k,pos_RFootImu,rankleJcRF,'-',[.8 .2 .2],.7);
        lfoot2AnkleLh=updateSingleLineHandles(lfoot2AnkleLh,k,pos_LFootImu,lankleJcLF,'-',[.8 .2 .2],.7);
        % () if desired, update limits
        if limBoxMode==2 || limBoxMode==3
            xlim(xlims(k,:)); ylim(ylims(k,:)); zlim(zlims(k,:));
        end
        % Get frame as an image for saving animation
        %     f = getframe(gcf);
        %     mov(:,:,1,k) = rgb2ind(f.cdata, map, 'nodither');
        % update
        drawnow;
        pause(.05);
    end
catch mException
    if strcmp(mException.identifier,'MATLAB:class:InvalidHandle')
        fprintf('    deleted animation figure.\n');
    else
        rethrow(mException);
    end
end
% optionally save gif
% if ~isempty(savename)
%     imwrite(mov, map, savename, 'DelayTime', 0, 'LoopCount', 1);
% end
end





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
function [xlim1,ylim1,zlim1]=geometricCenterBoundingBoxFixedAtImu(imuPosToFix,pos,xlim0,ylim0,zlim0)
% pos is an Nx3xM array of positions. find the geometric center and return adapted coordinate limits
%    which preserve the input arg xlim0, ylim0, zlim0
% computes the average geometric center relative to the sacrum imu position, then updates the limits to make the relationship constant
% () compute geometric center
geoCtr=zeros(size(pos,1),3);
for n=1:size(pos,1)
    geoCtr(n,:)=mean(pos(n,:,:),3);
end
% () now find average offset between imuPosToFix and geoCtr
imuToCamCtr=mean(geoCtr-imuPosToFix,1); % vector from imu frame to geoCtr
% () compute bound box limits
xlim1=xlim0+imuPosToFix(:,1)+imuToCamCtr(1);
ylim1=ylim0+imuPosToFix(:,2)+imuToCamCtr(2);
zlim1=zlim0+imuPosToFix(:,3)+imuToCamCtr(3);
end

function [xlim1,ylim1,zlim1]=geometricCenterBoundingBox(pos,xlim0,ylim0,zlim0)
% pos is an Nx3xM array of positions. find the geometric center and return adapted coordinate limits
%    which preserve the input arg xlim0, ylim0, zlim0
% () compute geometric center
geoCtr=zeros(size(pos,1),3);
for n=1:size(pos,1)
    geoCtr(n,:)=mean(pos(n,:,:),3);
end
% () compute bound box limits
xlim1=zeros(size(pos,1),2); ylim1=zeros(size(pos,1),2); zlim1=zeros(size(pos,1),2);
for n=1:size(pos,1)
    xlim1(n,:)=xlim0+geoCtr(n,1);
    ylim1(n,:)=ylim0+geoCtr(n,2);
    zlim1(n,:)=zlim0+geoCtr(n,3);
end
end

function newv=rotstaticvecCompose(R,v)
% assert(size(R,3)==size(v,2));
newv=zeros(size(v));
for k=1:size(R,3)
    newv(k,:)=[R(:,:,k)*v']';
end
end

function lh=updateSingleLineHandles(lh,i,pA,pB,linestyle,linecolor,alph)
assert(length(linecolor)==3);
% line goes between two points pA, and pB
% construct the line between the points
if isempty(lh) % construct new
    lh=line('XData',[pA(i,1) pB(i,1)],'YData',[pA(i,2) pB(i,2)],'ZData',[pA(i,3) pB(i,3)],'linewidth',1,'linestyle',linestyle,'color',[linecolor alph]);
else % update
    set(lh(1),'XData',[pA(i,1) pB(i,1)]);
    set(lh(1),'YData',[pA(i,2) pB(i,2)]);
    set(lh(1),'ZData',[pA(i,3) pB(i,3)]);
end
end

function lh=updateLocalCoordSysLineHandles(lh,i,R_B2N,pos,lineLength,linestyle,alph,linewidth)
% xLocalN=R_B2N(1,:,i)'; yLocalN=R_B2N(2,:,i)'; zLocalN=R_B2N(3,:,i)'; % old version
xLocalN=R_B2N(:,1,i); yLocalN=R_B2N(:,2,i); zLocalN=R_B2N(:,3,i); % old version
if isempty(lh) % construct new
    lh(1)=line('XData',[pos(i,1) pos(i,1)+xLocalN(1)'*lineLength],'YData',[pos(i,2) pos(i,2)+xLocalN(2)'*lineLength],'ZData',[pos(i,3) pos(i,3)+xLocalN(3)'*lineLength],'linewidth',linewidth,'linestyle',linestyle,'color',[1 0 0 alph]);
    lh(2)=line('XData',[pos(i,1) pos(i,1)+yLocalN(1)'*lineLength],'YData',[pos(i,2) pos(i,2)+yLocalN(2)'*lineLength],'ZData',[pos(i,3) pos(i,3)+yLocalN(3)'*lineLength],'linewidth',linewidth,'linestyle',linestyle,'color',[0 .5 0 alph]);
    lh(3)=line('XData',[pos(i,1) pos(i,1)+zLocalN(1)'*lineLength],'YData',[pos(i,2) pos(i,2)+zLocalN(2)'*lineLength],'ZData',[pos(i,3) pos(i,3)+zLocalN(3)'*lineLength],'linewidth',linewidth,'linestyle',linestyle,'color',[0 0 1 alph]);
else % update
    set(lh(1),'XData',[pos(i,1) pos(i,1)+xLocalN(1)'*lineLength]);
    set(lh(1),'YData',[pos(i,2) pos(i,2)+xLocalN(2)'*lineLength]);
    set(lh(1),'ZData',[pos(i,3) pos(i,3)+xLocalN(3)'*lineLength]);
    set(lh(2),'XData',[pos(i,1) pos(i,1)+yLocalN(1)'*lineLength]);
    set(lh(2),'YData',[pos(i,2) pos(i,2)+yLocalN(2)'*lineLength]);
    set(lh(2),'ZData',[pos(i,3) pos(i,3)+yLocalN(3)'*lineLength]);
    set(lh(3),'XData',[pos(i,1) pos(i,1)+zLocalN(1)'*lineLength]);
    set(lh(3),'YData',[pos(i,2) pos(i,2)+zLocalN(2)'*lineLength]);
    set(lh(3),'ZData',[pos(i,3) pos(i,3)+zLocalN(3)'*lineLength]);
end
end