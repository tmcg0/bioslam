% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function R=gtsamRot3YprConstructor(y,p,r)
% can't have named constructors in MATLAB. This recreates GTSAM's named constructor for ypr()
% according to the docs, RzRyRx(r,p,y) constructs the Rot3
assert(length(y)==length(p) && length(y)==length(r));
for k=1:length(y)
    R(k)=RzRyRx(r(k),p(k),y(k));
end
% () now backtest. is the constructed Rot3's ypr() output equivalent to the input ypr?
for k=1:length(y)
    yprOut(k,:)=R(k).ypr()';
    assert(all(abs(yprOut(k,:)-[y(k) p(k) r(k)])<.0000000001),'ypr failed');
end
end

function R=RzRyRx(x,y,z)
cx=cos(x);sx=sin(x);
cy=cos(y);sy=sin(y);
cz=cos(z);sz=sin(z);
ss_ = sx * sy;
cs_ = cx * sy;
sc_ = sx * cy;
cc_ = cx * cy;
c_s = cx * sz;
s_s = sx * sz;
mcs = cy * sz;
mcc = cy * cz;
s_c = sx * cz;
c_c = cx * cz;
ssc = ss_ * cz; csc = cs_ * cz; sss = ss_ * sz; css = cs_ * sz;
R=gtsam.Rot3(mcc,-c_s + ssc,  s_s + csc,mcs,  c_c + sss, -s_c + css,-sy,sc_,cc_);
end
