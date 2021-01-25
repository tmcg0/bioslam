% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function q=dcmToQuat(r)
% https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Rotation_matrix_%E2%86%94_quaternion
assert(size(r,1)==3); assert(size(r,2)==3);
N=size(r,3);
q=zeros(N,4);
for k=1:N
    q(k,1)=0.5*sqrt(1+r(1,1,k)+r(2,2,k)+r(3,3,k)); % qr
    q(k,2)=1/(4*q(k,1))*(r(3,2,k)-r(2,3,k)); % qi
    q(k,3)=1/(4*q(k,1))*(r(1,3,k)-r(3,1,k)); % qj
    q(k,4)=1/(4*q(k,1))*(r(2,1,k)-r(1,2,k)); % qi
end
end