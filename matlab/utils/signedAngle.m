% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function ang=signedAngle(v1,v2,refvec)
% find the unsigned angle between a and b, and then use the input reference vector refVec to determine sign,
% i.e., if cross(a,b) points in same direction as refVec, we say this is a positive (right-handed) rotation
% returns signed angle on [-pi,pi]
% give v1 and v2 and refVec Mx3 arrays of vectors
assert(size(v1,2)==3); assert(size(v2,2)==3); assert(size(refvec,2)==3);
assert(size(v1,1)==size(v2,1)); assert(size(v1,1)==size(refvec,1)); 
% preallocate and go
ang=zeros(size(v1,1),1);
for k=1:size(v1,1)
    crossvec=cross(v1(k,:),v2(k,:));
    ang(k)=unsignedAngle(v1(k,:),v2(k,:));
    if unsignedAngle(refvec(k,:),crossvec)>pi/2 % cross(v1,v2) and refvec point in opposite directions => negative angle
        ang(k)=-ang(k);
    end
end
end