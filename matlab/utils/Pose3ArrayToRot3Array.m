% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function rot3array=Pose3ArrayToRot3Array(pose3array)
% pull out Rot3's, in order
for k=1:length(pose3array)
    rot3array(k)=pose3array(k).rotation;
end
end