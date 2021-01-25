% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function p=Point3ArrayToMatrix(point3array)
p=zeros(length(point3array),3);
for k=1:length(point3array)
   p(k,:)=[point3array(k).vector]'; 
end
end