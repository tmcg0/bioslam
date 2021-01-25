% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function printVectorOptionalVariance(label,vec,variance)
% a simple function to print an N-vector with optional variance printing. also skips line at end.
assert(isa(label,'char'));
vec=vec(:); variance=variance(:);
if isempty(variance)
    fmt=['%s: [',repmat('%.4f, ', 1, length(vec)-1),'%.4f] (norm=%.4f)\n'];
    fprintf(fmt,label,vec,sqrt(sum(vec.^2,1)));
else % print that too
    fmt=['%s: [',repmat('%.4f, ', 1, length(vec)-1),'%.4f] (norm=%.4f) (var: [',repmat('%.4f, ', 1, length(variance)-1),'%.4f])\n'];
    fprintf(fmt,label,vec,sqrt(sum(vec.^2,1)),variance);
end
end