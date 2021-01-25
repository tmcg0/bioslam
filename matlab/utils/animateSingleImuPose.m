% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function anih=animateSingleImuPose(time,p,q_B_to_N,plotmode,savename)
% q_B_to_N=quatinv(q_B_to_N);
plotGlobalCoordSys=1;
x=p(:,1); y=p(:,2); z=p(:,3);
toskip=1;
numIntermediateImuFrames=15;
newvec=1:toskip:length(time);
time=time(newvec);
x=x(newvec); y=y(newvec); z=z(newvec); q_B_to_N=q_B_to_N(newvec,:);

% () set IMU size
imuSize=[.2 .2 .1]./2; % meters

% scale x y and z limit range
[xmin,xmax,ymin,ymax,zmin,zmax]=getAxesLimitsAuto(x,y,z,imuSize);
arrowLength=0.1*max([xmax-xmin,ymax-ymin,zmax-zmin]);

[phi,theta,psi]=quat2angle(quatinv(q_B_to_N));
KF=rad2deg(unwrap([phi theta psi]));
phi=KF(:,1); theta=KF(:,2); psi=KF(:,3);

% if ishandle(anih); delete(anih); end;
anih=figure('Units','Inches','Position',[5 .5 7 10],'Color', 'white');
set(h,'Renderer','painters'); % painters is prettier than OpenGL
subplot(3,1,1)
hold on
plot(time,phi,'-r','linewidth',1.5)
plot(time,theta,'Color',[0 .5 0],'linewidth',1.5)
plot(time,psi,'-b','linewidth',1.5)
grid on; box on;
xlabel('time (s)'); ylabel('Angle (deg)')
hold off
hh1(1) = line(time(1), phi(1), 'Marker', '.', 'MarkerSize', 20,'Color', 'r');
hh1(2) = line(time(1), theta(1), 'Marker', '.', 'MarkerSize', 20,'Color',[0 0.5 0]);
hh1(3) = line(time(1), psi(1), 'Marker', '.', 'MarkerSize', 20,'Color', 'b');
animAxes=subplot(3,1,[2,3]);
posPlotH=plot3(x(1),y(1),z(1),'LineWidth',0.5,'linestyle',':','color','k'); % plot position data
if plotGlobalCoordSys; hold on;
    line('XData',[0 arrowLength],'YData',[0 0],'ZData',[0 0],'linewidth',2,'color','r');
    line('XData',[0 0],'YData',[0 arrowLength],'ZData',[0 0],'linewidth',2,'color',[0 .5 0]);
    line('XData',[0 0],'YData',[0 0],'ZData',[0 arrowLength],'linewidth',2,'color','b');
end
switch plotmode
    case 'global'
        axis([xmin xmax ymin ymax zmin zmax]); view(3);
    case 'follow'
        warning('implement this');
end
xlabel('x (mm)'); ylabel('y (mm)'); zlabel('z (mm)'); axis equal;

% cd=[0 1 0; 1 0 0; 0 0 1; 0 0 1; 0 0 1; 0 0 1];
cd=repmat([.5 .5 .5],6,1); % all gray
imuh=drawCuboid([x(1) y(1) z(1) imuSize(1) imuSize(2) imuSize(3) phi(1) theta(1) psi(1)], 'FaceColor','flat','FaceVertexCData',cd);
grid on; box on;


% Get figure size
pos = get(gcf, 'Position');


ht = title(sprintf('Time: %0.4f sec', time(1)));
f = getframe(gcf);
[samp, map] = rgb2ind(f.cdata, 256, 'nodither');

% Preallocate data (for storing frame data)
mov = zeros(size(samp,1), size(samp,2), 1, length(time), 'uint8');
figure(anih);
% plot local imu coordinate systems
xlocal=quatrotate(q_B_to_N(1,:),[1 0 0]*arrowLength*.8);
ylocal=quatrotate(q_B_to_N(1,:),[0 1 0]*arrowLength*.8);
zlocal=quatrotate(q_B_to_N(1,:),[0 0 1]*arrowLength*.8);
hold on; k=1;
xlh=line('XData',[x(k) x(k)+xlocal(1)],'YData',[y(k) y(k)+xlocal(2)],'ZData',[z(k) z(k)+xlocal(3)],'linewidth',.6,'color','red');
ylh=line('XData',[x(k) x(k)+ylocal(1)],'YData',[y(k) y(k)+ylocal(2)],'ZData',[z(k) z(k)+ylocal(3)],'linewidth',.6,'color',[0 .5 0]);
zlh=line('XData',[x(k) x(k)+zlocal(1)],'YData',[y(k) y(k)+zlocal(2)],'ZData',[z(k) z(k)+zlocal(3)],'linewidth',.6,'color','blue');
% () get indeces to leave the coordinate systems behind in the animation
imuIntermediateCoordSysIndeces=round(linspace(1,length(time),numIntermediateImuFrames));
numImuFramesApplied=0;
try
    % Loop through by changing XData and YData
    for k = 1:length(time)
        % Update graphics data. This is more efficient than recreating plots.
        set(hh1(1), 'XData', time(k), 'YData', phi(k))
        set(hh1(2), 'XData', time(k), 'YData', theta(k))
        set(hh1(3), 'XData', time(k), 'YData', psi(k))
        
        imuh.delete;
        imuh=drawCuboid([x(k) y(k) z(k) imuSize(1) imuSize(2) imuSize(3) phi(k) theta(k) psi(k)], 'FaceColor','flat','FaceVertexCData',cd);
        set(ht, 'String', sprintf('Time: %0.3f sec', time(k)))
        axis([xmin xmax ymin ymax zmin zmax]);
        
        % () update the local coordinate system lines of the imu
        [xlh,ylh,zlh]=updateLocalCoordSysLineHandles(xlh,ylh,zlh,x(k),y(k),z(k),q_B_to_N(k,:),arrowLength*.8);
        
        if numImuFramesApplied<(sum((k+1)>imuIntermediateCoordSysIndeces))
            applyLinesAsGhostedToPlot(xlh,ylh,zlh,animAxes);
            numImuFramesApplied=numImuFramesApplied+1;
        end
        
        % () update position line
        posPlotH.XData=x(1:k); posPlotH.YData=y(1:k); posPlotH.ZData=z(1:k);
        % Get frame as an image
        f = getframe(gcf);
        
        mov(:,:,1,k) = rgb2ind(f.cdata, map, 'nodither');
        
        pause(.0000001); drawnow;
        
    end
catch mException
    if strcmp(mException.identifier,'MATLAB:class:InvalidHandle') || strcmp(mException.identifier,'MATLAB:structRefFromNonStruct')
        fprintf('    deleted animation figure.\n');
    else
        rethrow(mException);
    end
end

if ~isempty(savename)
    imwrite(mov, map, 'animation.gif', 'DelayTime', 0, 'LoopCount', 1);
end
end

function applyLinesAsGhostedToPlot(xlh,ylh,zlh,animAxes)
xlnew=copyobj(xlh,animAxes); xlnew.Color=[1 .5 .5];
ylnew=copyobj(ylh,animAxes); ylnew.Color=[.4 .8 .4];
zlnew=copyobj(zlh,animAxes); zlnew.Color=[.5 .5 1];
end

function [xlh,ylh,zlh]=updateLocalCoordSysLineHandles(xlh,ylh,zlh,x,y,z,q_B_to_N,lineLength)
% xlh=line('XData',[x(k) x(k)+xlocal(1)],'YData',[y(k) y(k)+xlocal(2)],'ZData',[z(k) z(k)+xlocal(3)],'linewidth',.6,'color','red');
% ylh=line('XData',[x(k) x(k)+ylocal(1)],'YData',[y(k) y(k)+ylocal(2)],'ZData',[z(k) z(k)+ylocal(3)],'linewidth',.6,'color',[0 .5 0]);
% zlh=line('XData',[x(k) x(k)+zlocal(1)],'YData',[y(k) y(k)+zlocal(2)],'ZData',[z(k) z(k)+zlocal(3)],'linewidth',.6,'color','blue');
xlocalN=quatrotate(q_B_to_N,[1 0 0]*lineLength);
ylocalN=quatrotate(q_B_to_N,[0 1 0]*lineLength);
zlocalN=quatrotate(q_B_to_N,[0 0 1]*lineLength);
xlh.XData=[x x+xlocalN(1)]; xlh.YData=[y y+xlocalN(2)]; xlh.ZData=[z z+xlocalN(3)];
ylh.XData=[x x+ylocalN(1)]; ylh.YData=[y y+ylocalN(2)]; ylh.ZData=[z z+ylocalN(3)];
zlh.XData=[x x+zlocalN(1)]; zlh.YData=[y y+zlocalN(2)]; zlh.ZData=[z z+zlocalN(3)];
end

function [xmin,xmax,ymin,ymax,zmin,zmax]=getAxesLimitsAuto(x,y,z,imuSize)
% create limits which include the origin, the IMU, and its full path at all times
boundingPct=.2; % how many percent larger should the box be?
xmin=min(vertcat(x,0)); xmax=max(vertcat(x,0));
ymin=min(vertcat(y,0)); ymax=max(vertcat(y,0));
zmin=min(vertcat(z,0)); zmax=max(vertcat(z,0));
% () now adjust for imu size and bounding percent
xmin=(1+boundingPct)*(xmin-max(imuSize)); xmax=(1+boundingPct)*(xmax+max(imuSize));
ymin=(1+boundingPct)*(ymin-max(imuSize)); ymax=(1+boundingPct)*(ymax+max(imuSize));
zmin=(1+boundingPct)*(zmin-max(imuSize)); zmax=(1+boundingPct)*(zmax+max(imuSize));
end