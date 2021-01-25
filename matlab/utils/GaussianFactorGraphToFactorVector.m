% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function fac = GaussianFactorGraphToFactorVector(graph)
%GRAPHTOFACTORVECTOR Summary of this function goes here
%   Detailed explanation goes here
% this is for Gaussian Factor Graph types--that way all of the factors are of homogeneous type JacobianFactor
assert(isa(graph,'gtsam.GaussianFactorGraph'));
n=graph.size; % number of factors in graph
for k=1:n
    fac(k)=graph.at(k-1); % note that at() is zero-indexed
end
end

