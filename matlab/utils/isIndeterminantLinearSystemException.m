% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function isit=isIndeterminantLinearSystemException(mException)
%ISINDETERMINANTLINEARSYSTEMEXCEPTION Summary of this function goes here
%   Detailed explanation goes here
isit=0;
assert(isa(mException,'MException'),'please input a MATLAB MException');
% check if its message contains the phrase
if ~isempty(strfind(mException.message,'Indeterminant linear system detected while working near variable'))
    isit=1;
end
end

