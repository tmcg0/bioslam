% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function val=getValueFromUnknownKeyType(vals,key)
keyType=determineVarType(vals,key);
val=getValueFromKnownKeyType(vals,key,keyType);
end