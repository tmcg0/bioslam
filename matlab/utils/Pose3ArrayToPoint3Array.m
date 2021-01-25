% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function point3array=Pose3ArrayToPoint3Array(pose3array)
% pull out Point3's, in order
for k=1:length(pose3array)
    point3array(k)=pose3array(k).translation;
end
end