% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function R = flattenedRotationMatrixTo3x3xNMatrix(r)
% previously, the cpp version of the library created rotation matrices that were 9xN. this function fixes that and puts it into a 3x3xN array.
if size(r,2)>size(r,1) % flip it so it's row-indexed
   r=r'; 
end
R=zeros(3,3,numel(r)/9);
for k=1:size(R,3)
    R(1,1,k)=r(k,1); R(1,2,k)=r(k,2); R(1,3,k)=r(k,3); 
    R(2,1,k)=r(k,4); R(2,2,k)=r(k,5); R(2,3,k)=r(k,6); 
    R(3,1,k)=r(k,7); R(3,2,k)=r(k,8); R(3,3,k)=r(k,9); 
end
end

