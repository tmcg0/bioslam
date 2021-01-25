% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function H5EasyAdd(file,loc,data)
% a simple function to add data to an h5 file
% make sure it's numeric data
assert(isa(data,'numeric'));
% if dataset doesn't exist, create it
h5create(file,loc,size(data));
h5write(file,loc,data);
end

