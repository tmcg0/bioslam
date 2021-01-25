% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function R=Rot3QuaternionConstructor(q)
% can't have named constructors in MATLAB. This recreates GTSAM's named constructor for quaternions
assert(size(q,2)==4);
for k=1:length(size(q,1))
    D=quatToDcm(q(k,:));
    R(k)=gtsam.Rot3(D(1,1),D(1,2),D(1,3),D(2,1),D(2,2),D(2,3),D(3,1),D(3,2),D(3,3));
end
end