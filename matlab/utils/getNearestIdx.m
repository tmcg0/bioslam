% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function idx = getNearestIdx(fromVec,valToMatch)
%GETNEARESTIDX Summary of this function goes here
%   Detailed explanation goes here
[~,idx]=min(abs(fromVec-valToMatch));
end

