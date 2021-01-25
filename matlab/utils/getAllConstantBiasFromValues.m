% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function myBiasArray=getAllConstantBiasFromValues(vals)
allKeys=vals.keys;
nBiasFound=0;
for k=1:allKeys.size
    try
        val=vals.atConstantBias(allKeys.at(k-1));
        nBiasFound=nBiasFound+1;
        myBiasArray(nBiasFound)=val;
    catch
    end
end
end