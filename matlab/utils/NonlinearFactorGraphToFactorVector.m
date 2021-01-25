% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function fac = NonlinearFactorGraphToFactorVector(graph)
%GRAPHTOFACTORVECTOR Summary of this function goes here
%   Detailed explanation goes here
% this is for nonlinear factor graphs. factor vectors are non homogenous so store them in a cell array.
assert(isa(graph,'gtsam.NonlinearFactorGraph'));
n=graph.size; % number of factors in graph
for k=1:n
    fac{k}=graph.at(k-1); % note that at() is zero-indexed
end
end

