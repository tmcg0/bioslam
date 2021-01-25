% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function yaw = unwrappedYawAngle(R)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
assert(isa(R,'gtsam.Rot3'));
yaw=zeros(size(R));
for k=1:length(R)
   yaw(k)=R(k).yaw(); 
end
yaw=unwrap(yaw);
end

