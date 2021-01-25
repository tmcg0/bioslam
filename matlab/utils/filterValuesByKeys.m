% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function valsFiltered=filterValuesByKeys(vals,keys)
%FILTERVALUESBYKEYS Pull out subset of Values which matches input key array keys
%   Detailed explanation goes here
% This is a really slow function. To print debug timing, set this variable to 1:
printDebugTiming=0;
if printDebugTiming; fprintf('\n---Printing debug timing for filterValuesByKeys(). Set variable to false to avoid this---\n'); end
% () pull out array of uint64 keys from vals
t0=tic; % total function time tic
%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% old way this was done. was found to be slow. leaving it here for posterity. used to take up >90% of the whole function time.
t1=tic;
valsKeysUint64=uint64(zeros(vals.size,1));
for k=1:length(valsKeysUint64)
    valsKeysUint64(k)=vals.keys.at(k-1); % this is bad practice! slow! construct KeyVector explicitly.
end
if printDebugTiming; fprintf('\tpulling out unit64 keys from ''vals'': %.2f seconds\n',toc(t1)); end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}
t1=tic;
valkeys=vals.keys; % construct KeyVector first from the vals, then pull out the Unit64 Keys. This was found to be faster.
valsKeysUint64=uint64(zeros(valkeys.size,1));
for k=1:length(valsKeysUint64)
    valsKeysUint64(k)=valkeys.at(k-1);
end
if printDebugTiming; fprintf('\tpulling out unit64 keys from ''vals'': %.2f seconds\n',toc(t1)); end

% () do same for subset keys
t2=tic;
keysUint64=uint64(zeros(keys.size,1));
for k=1:length(keysUint64)
    keysUint64(k)=keys.at(k-1);
end
if printDebugTiming; fprintf('\tpulling out unit64 keys from ''keys'': %.2f seconds\n',toc(t2)); end
% () find intersection
t3=tic;
[matchingVals,~] = intersect(valsKeysUint64,keysUint64,'stable');
if printDebugTiming; fprintf('\tfinding intersection of these two unit64 key arrays: %.2f seconds\n',toc(t3)); end
% () pull out just the Values which were found in idxInVals
t4=tic;
valsFiltered=gtsam.Values;
for k=1:length(matchingVals)
    value=getValueFromUnknownKeyType(vals,matchingVals(k));
    valsFiltered.insert(matchingVals(k),value);
end
if printDebugTiming; fprintf('\treconstructing subset of vals with the appropriate keys: %.2f seconds\n',toc(t4)); end
if printDebugTiming; fprintf('---end debug timing (total time = %.4f seconds)---\n',toc(t0)); end
end

