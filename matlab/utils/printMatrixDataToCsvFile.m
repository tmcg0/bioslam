% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function printMatrixDataToCsvFile(filename,m)
%PRINTMATRIXDATATOCSVFILE Summary of this function goes here
%   Detailed explanation goes here
try
    dlmwrite(filename,m,'delimiter',',','precision',9);
catch mException
    errmessage=sprintf('\texception in printMatrixDataToCsvFile: %s -- Did not write data to file %s\n',mException.message,filename);
end
end