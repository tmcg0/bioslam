% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function [prinVar,margCov]=computePrincipalVariance(keys,marginals,varSize)
% for a given set of variables, you can perform margnilization to compute the marginal covariance
% this function does that, and then pulls out the prinicpal (diagonal) variance. convenient later for plotting error bars, etc.
% varSize is just used to preallocate an array to hold the marginal covariance
% optionally output the principal variance and the full marginal covariance
assert(isa(marginals,'gtsam.Marginals'));
try
    % preallocate
    prinVar=zeros(length(keys),varSize); % [N x varSize] of principal variance
    margCov=zeros(varSize,varSize,length(keys)); % [varSize x varSize x N] marginal covariance
    for k=1:length(keys)
        margCov(:,:,k)=marginals.marginalCovariance(keys(k));
        prinVar(k,:)=diag(margCov(:,:,k))';
    end
catch mException
    if isIndeterminantLinearSystemException(mException)
        fprintf('\texception: indeterminant system, could not get marginal covariances\n');
    else
        error('what kind of error is this?');
    end
end
end