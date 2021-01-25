% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function ang = jointAngleUnwrap(ang,range,tol)
% Custom joint angle unwrapping function, knowing that:
% flex/ex and int/ext rot are on [-pi,pi], i.e., their range is 2pi
% ab/ad is on [-pi/2,pi/2] => its range is pi.
% detects differences and corrects them
for k=2:length(ang)
    if (ang(k)-ang(k-1)) > range*tol
        % you jumped upward. shift k down by range
        ang(k)=ang(k)-range;
    elseif (ang(k)-ang(k-1)) < -range*tol
        % you jumped downward. shift k up by range
        ang(k)=ang(k)+range;
    end
end
end

