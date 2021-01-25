% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function q=Rot3ArrayToQuaternions(rot3vec)
% take in an N-array of Rot3 objects, convert to an Nx4 matrix of quaternions
q=zeros(length(rot3vec),4);
for k=1:length(rot3vec)
   q(k,:)=[rot3vec(k).quaternion()]'; 
end
q=quat_unwrap(q);
end