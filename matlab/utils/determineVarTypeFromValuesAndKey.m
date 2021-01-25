% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function varTypes=determineVarTypeFromValuesAndKey(vals,key)
%DETERMINEVARTYPEFROMVALUESANDKEY Summary of this function goes here
%   Detailed explanation goes here
if isa(key,'unit64')
    error('need to write this case');
elseif isa(key,'cell') % hoping you passed in a cell array of single letter/number combinations...
    verifyProperKeyCellFormat(key);
end
symArray=keyCellToGtsamSymbolArray(key);
varTypes=cell(1,length(key));
for k=1:length(symArray)
    varTypes{k}=determineVarType(vals,symArray(k));
end
end

function symArray=keyCellToGtsamSymbolArray(keyCell)
symArray=uint64(zeros(1,length(keyCell)));
for k=1:length(keyCell)
    symArray(k)=gtsam.symbol(keyCell{k}(1),str2num(keyCell{k}(2:end)));
end
end

function verifyProperKeyCellFormat(keyCell)

end

