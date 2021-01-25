% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function ang=unsignedAngle(v1,v2)
% returns unsigned angle on [0,pi]
% give v1 and v2 as Mx3 arrays of vectors
assert(size(v1,2)==3); assert(size(v2,2)==3); assert(size(v1,1)==size(v2,1));
% preallocate and go
ang=zeros(size(v1,1),1);
for k=1:size(v1,1)
    crossvec=cross(v1(k,:),v2(k,:));
    ang(k)=atan2(sqrt(sum(crossvec.^2,2)),dot(v1,v2));
end
end

