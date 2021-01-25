% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function ang=signedAngleBetweenVectorsInSamePlane(vA, vB, vN)
% you have plane normal vN and two vectors in that plane vA and vB.
% this function computes the signed angle between these vectors as atan2((Vb x Va) . Vn, Va . Vb) (with vN normalized)
% should return an angle in [-pi,pi]
% sign convention is: [insert here]
% based on https:%stackoverflow.com/a/33920320
vA=vA(:)./sqrt(sum(vA(:).^2,1));
vB=vB(:)./sqrt(sum(vB(:).^2,1));
vN=vN(:)./sqrt(sum(vN(:).^2,1));
ang=atan2(dot(cross(vB, vA),vN), dot(vA, vB));
end