% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function [ImuInteriorAngle,toJoint1InteriorAng,toJoint2InteriorAng,ImuDistanceToSegment]=imuToAdjacentJointCtrsTriangularGeometry(v1,v2)
% v1: vector from IMU to joint 1
% v2: vector from IMU to joint 2
assert(isa(v1,'double') && isa(v2,'double')); % make sure these are matlab arrays, not gtsam::Point3
v1=v1(:)'; v2=v2(:)';
ImuInteriorAngle=rad2deg(interiorAngleBetweenTwoVectors(v1,v2));
% equiv to: acosd(dot(v1, v2,1)./(sqrt(sum(v1.^2))*sqrt(sum(v2.^2))))
toJoint1InteriorAng=rad2deg(interiorAngleBetweenTwoVectors(-v1,(v2-v1)));
ImuDistanceToSegment=sqrt(sum( cross(-v1,-v2).^2,2 ))/sqrt(sum([v1-v2].^2,2));
toJoint2InteriorAng=rad2deg(interiorAngleBetweenTwoVectors(-v2,(v1-v2)));
end