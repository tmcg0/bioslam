% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function [x,P]=simpleForwardBackwardEkf(gyros,accels,dt,numFBPasses)

numPasses=0;
initX=[1 0 0 0]; initP=1e-4*eye(4);
while numPasses<numFBPasses
    % forward pass
    [x,P]=simpleImuOrientationForwardEkf(gyros,accels,dt,initX,initP);
    % now take that original data, flip it (and neg the gyro data) and then run it backward
    % remember to initialize x and P to last index of the forward pass!
    [x,P]=simpleImuOrientationForwardEkf(flipud(-gyros),flipud(accels),dt,x(end,:),P(:,:,end)*1.1);
    x=flipud(x); P=flipud3dArray(P); % you now have one full f/b pass!
    initX=x(1,:); initP=P(:,:,1);
    % iterate up
    numPasses=numPasses+1;
end
end

function mflipped=flipud3dArray(m)
% reverse order of a 3d array
mflipped=zeros(size(m));
mflipped(:,:,1:size(m,3))=m(:,:,size(m,3):-1:1);
end

function t=trace3dArray(m)
t=zeros(size(m,3),1);
for k=1:size(m,3)
    t(k)=trace(m(:,:,k));
end
end