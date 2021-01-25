% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function [flexionAngleRad,adductionAngleRad,externalRotationAngleRad]=clinical3DofJcsAngles(R_proximalSeg_to_N, R_distalSeg_to_N, side)
% returns angles by the "clinical" convention, rather than the "consistent" convention
% recall "consistent" angle convention is positive in:
%    hip flexion, right hip abduction and internal rotation, left hip adduction and external rotation
% the "clinical" angle convention (consistent with OpenSim) is positive in:
%    hip flexion, adduction, internal rotation
% these examples are written for the hip, but are generalized to the knee as well.
switch side
    case 'R' % right leg, already the correct sign
        [flexionAngleRad,adductionAngleRad,externalRotationAngleRad]=consistent3DofJcsAngles(R_proximalSeg_to_N, R_distalSeg_to_N);
        adductionAngleRad=-adductionAngleRad; % flip for clinical system
    case 'L' % left leg, swap signs on int/ext rot and ab/ad
        [flexionAngleRad,adductionAngleRad,externalRotationAngleRad]=consistent3DofJcsAngles(R_proximalSeg_to_N, R_distalSeg_to_N);
        externalRotationAngleRad=-externalRotationAngleRad; % flip for clinical system
    otherwise
        error('unknown side. specify R or L');
end
end