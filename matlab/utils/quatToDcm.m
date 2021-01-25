% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function A = quatToDcm(q)
% https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Conversion_formulae_between_formalisms
% note: this is different then the results returned by MATLAB. why?
assert(size(q,2)==4);
% assumes scalar first
A=zeros(3,3,size(q,1));
for k=1:size(q,1)
   qr=q(k,1); % scalar part
   qi=q(k,2); qj=q(k,3); qk=q(k,4);
   qhat=[qi qj qk]';
   Q=[0 -qk qj; qk 0 -qi; -qj qi 0];
   A(:,:,k)=(qr^2-qhat'*qhat)*eye(3)+2*qhat*qhat'+2*qr*Q;
end
end

function compareToMatlab(q,d)
d_matlab=quat2dcm(q);
end