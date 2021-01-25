% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function [myVecArray,keysOfVecsFoundUint]=getAllVectorFromValues(vals,vecSize)
assert(isa(vecSize,'double'));
allKeys=vals.keys;
nVecsFound=0;
keysOfVecsFound=0;
keysOfVecsFoundUint=uint64(0);
myVecArray=[0 0 0];
for k=1:allKeys.size
    try
        val=vals.atVector(allKeys.at(k-1));
        assert(length(val)==vecSize); % only add vectors of requested size
        nVecsFound=nVecsFound+1;
        myVecArray(nVecsFound,:)=val(:)';
        keysOfVecsFound(nVecsFound)=allKeys.at(k-1);
        keysOfVecsFoundUint(nVecsFound)=allKeys.at(k-1);
    catch
    end
end
% () finally, sort the Vecs by key num in ascending order
% this should probably already be the case. but just to be sure...
[~,idxs]=sort(keysOfVecsFoundUint);
assert(all(diff(idxs)==1),'you expect this condition to be true'); % just to be sure
end