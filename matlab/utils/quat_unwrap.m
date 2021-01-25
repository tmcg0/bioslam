% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function q=quat_unwrap(q)
% pull out rotation axes and unit-ize
quatAxes=q(:,2:4)./sqrt(sum(q(:,2:4).^2,2));
for k=2:length(quatAxes)
   regularAngle=angle_between_vectors_deg(quatAxes(k-1,:),quatAxes(k,:));
   flippedAngle=angle_between_vectors_deg(quatAxes(k-1,:),-quatAxes(k,:));
   if flippedAngle<regularAngle % need to flip
      quatAxes(k,:)=-quatAxes(k,:);
      q(k,:)=-q(k,:);
   end
end
end
