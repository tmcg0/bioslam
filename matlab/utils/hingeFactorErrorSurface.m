% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function hingeFactorErrorSurface(rotA,wA,rotB,wB,frame,doPlot,figType)
% sample/plot the entire 3d error surface of the hinge factor given vector of rotations rotA, rotB,
%    and ang velocities wA, wB.
% input frame as 'A' or 'B' for distal or proximal frame
% plots onto active axes
assert(length(rotA)==length(rotB));
% () get relative angular velocity m
switch frame
    case 'A'
        m=relAngVelProxFrame(rotA,wA,rotB,wB);
    case 'B'
        m=relAngVelDistFrame(rotA,wA,rotB,wB);
    otherwise
        error('enter frame as A or B');
end
% remember error metric is ~||(m-projection of m onto k)|| (remember we use the vector error model)
%   => e= ||m - dot(m,r)||
if figType==1 % sphereical model
    % () sample a sphere
    [x,y,z]=sphere(50);
    e=zeros(size(x));
    % () loop through these x y z coordinates, computing error
    S=eye(3,3);
    for i=1:size(x,1)
        for j=1:size(x,2)
            e(i,j)=errorMetric(m,[x(i,j) y(i,j) z(i,j)],S);
        end
    end
    % () now scale x,y,z by e
    for i=1:size(x,1)
        for j=1:size(x,2)
            x(i,j)=x(i,j)*e(i,j);
            y(i,j)=y(i,j)*e(i,j);
            z(i,j)=z(i,j)*e(i,j);
        end
    end
    % () now plot
    if doPlot
        h=surf(x,y,z); axis equal; grid on;
        title(sprintf('hinge constraint error surface: max(e)/min(e)=%.3f',max(max(e))/min(min(e))));
        set(h,'LineWidth',0.1); set(h,'edgealpha',0.4);
        xlabel('x'); ylabel('y'); zlabel('z');
    else
        figh=0;
    end
elseif figType==2 % 2d heatmap
    % sample all possible 2d values. these will then be mapped to 3d axis in order to compute error norm.
    % note that normal range of all spherical values is azimuth [0,2*pi], elevation [-pi,pi].
    %    but since this error model is symmetric you may cut one of these in half. so you could just cut azimuth to [0,pi]
    % https://www.mathworks.com/help/matlab/ref/sph2cart.html
    %     azimuth is the counterclockwise angle in the x-y plane measured in radians from the positive x-axis.
    %     elevation is out of x-y plane
    %     problem: at +/-[0 0 1], this represents elevation +/- pi with any azimuth.
    N1=50; N2=49;
    azimuth=linspace(0,2*pi,N1);
    elevation=linspace(-pi/2,pi/2,N2);
    % preallocate everything
    x=zeros(N1,N2); y=zeros(N1,N2); z=zeros(N1,N2); e=zeros(N1,N2);
    % compute candidate x,y,z for array of [az,el]
    for i=1:length(azimuth)
       for j=1:length(elevation)
           [x(i,j),y(i,j),z(i,j)] = sph2cart(azimuth(i),elevation(j),1);
       end
    end
    % compute error e
    S=eye(3,3);
    for i=1:length(azimuth)
        for j=1:length(elevation)
            e(i,j)=errorMetric(m,[x(i,j) y(i,j) z(i,j)],S);
        end
    end
    % () now plot
    if doPlot
        imagesc(azimuth,elevation,e'); % why do I have to transpose e here?
        c = colorbar;
        c.Label.String = 'Norm error';
        title(sprintf('hinge constraint error surface: max(e)/min(e)=%.3f',max(max(e))/min(min(e))));
        xlabel('azimuth (rad)'); ylabel('elevation (rad)');
    else
        figh=0;
    end
end
end

function eg=errorMetric(m,k,S)
% input Nx3 wrel and 1x3 axis k
% compute error metric as:
% 1/2*sum(e.^2) where e(1:N) is below
% S is the 3x3 sigmas matrix
assert(all(size(k)==[1 3]));
k=repmat(k,[size(m,1) 1]);
e=(m-dot(m,k,2).*k);
en=zeros(length(e),1); % noise weighted error
for i=1:length(m)
    en(i)=e(i,:)*S*e(i,:)';
end
% global error:
eg=.5*sum(en.^2);
end

function wrel=relAngVelProxFrame(rotA,wA,rotB,wB)
wrel=zeros(length(rotA),3);
for k=1:length(rotA)
    wrel(k,:)= (rotA(k).inverse().matrix()*rotB(k).matrix()*wB(k,:)')' - wA(k,:); % w[B-A] in A
end
end

function wrel=relAngVelDistFrame(rotA,wA,rotB,wB)
wrel=zeros(length(rotA),3);
for k=1:length(rotA)
    wrel(k,:)= wB(k,:) - (rotB(k).inverse().matrix()*rotA(k).matrix()*wA(k,:)')' ; % w[B-A] in B
end
end