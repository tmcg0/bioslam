% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function uintkeys = KeyVectorToUintKeyArray(keys)
%KEYVECTORTOUINTKEYARRAY Summary of this function goes here
%   Detailed explanation goes here
assert(isa(keys,'gtsam.KeyVector'));
uintkeys=uint64(zeros(keys.size,1));
for k=1:keys.size
   uintkeys(k)=keys.at(k-1); 
end
end