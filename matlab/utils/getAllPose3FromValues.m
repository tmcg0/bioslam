% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function [myPose3Array,keysOfPose3sFoundUint]=getAllPose3FromValues(vals)
assert(length(vals)>0,'your values are empty!');
allKeys=vals.keys;
nPose3Found=0;
keysOfPose3sFound=[];
keysOfPose3sFoundUint=uint64(0);
for k=1:allKeys.size
    try
        val=vals.atPose3(allKeys.at(k-1));
        nPose3Found=nPose3Found+1;
        myPose3Array(nPose3Found)=val;
        keysOfPose3sFound(nPose3Found)=allKeys.at(k-1);
        keysOfPose3sFoundUint(nPose3Found)=allKeys.at(k-1);
    catch
    end
end
% () finally, sort the Pose3s by key num in ascending order
% this should probably already be the case. but just to be sure...
[~,idxs]=sort(keysOfPose3sFoundUint);
assert(all(diff(idxs)==1),'you expect this condition to be true'); % just to be sure
end