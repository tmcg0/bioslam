% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function angbw = interiorAngleBetweenTwoVectors(v1,v2)
%INTERIORANGLEBETWEENTWOVECTORS Summary of this function goes here
%   Detailed explanation goes here
assert(size(v1,2)==3); assert(size(v2,2)==3); assert(size(v1,1)==size(v2,1));
angbw=zeros(size(v1,1),1);
for k=1:size(v1,1)
   angbw(k)=acos(dot(v1(k,:),v2(k,:),2)./(norm(v1(k,:))*norm(v2(k,:))));
end
end

