% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function r=Rot3ArrayToMatrices(rot3vec)
% take in an N-array of Rot3 objects, convert to a dcm array
assert(isa(rot3vec,'gtsam.Rot3'));
r=zeros(3,3,length(rot3vec));
for k=1:length(rot3vec)
   r(:,:,k)=[rot3vec(k).matrix()]; 
end
end