% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function out=parseFactorIntoDescriptionString(factor)
if isa(factor,'gtsam.JacobianFactor')
    error('this doesn''t work on jacobianfactors')
    return;
end
str='';
% figure out the type and parse accordingly
if isa(factor,'gtsam.PriorFactorVector')
    str=parsePriorFactorVector(factor);
elseif isa(factor,'gtsam.PriorFactorConstantBias')
    str=parsePriorFactorConstantBias(factor);
elseif isa(factor,'gtsam.PriorFactorPose3')
    str=parsePriorFactorPose3(factor);
elseif isa(factor,'gtsam.ImuFactor')
    str=parseImuFactor(factor);
else
    error('do not recongnize this factor type!');
end
out=str;
end

function out=parsePriorFactorPose3(factor)
str=evalc('factor');
r=regexp(str,'(PriorFactor on [a-zA-z]\d{1,100})','tokenExtents');
out=str(r{1}(1):r{1}(2));
end

function out=parseImuFactor(factor)
str=evalc('factor');
r=regexp(str,'(ImuFactor\([a-zA-z]\d{1,100}\x2C[a-zA-z]\d{1,100}\x2C[a-zA-z]\d{1,100}\x2C[a-zA-z]\d{1,100}\x2C[a-zA-z]\d{1,100}\))','tokenExtents');
out=str(r{1}(1):r{1}(2));
end
 
function out=parsePriorFactorConstantBias(factor)
str=evalc('factor');
r=regexp(str,'(PriorFactor on [a-zA-z]\d{1,100})','tokenExtents');
out=str(r{1}(1):r{1}(2));
end

function out=parsePriorFactorVector(factor)
str=evalc('factor');
r=regexp(str,'(PriorFactor on [a-zA-z]\d{1,100})','tokenExtents');
out=str(r{1}(1):r{1}(2));
end