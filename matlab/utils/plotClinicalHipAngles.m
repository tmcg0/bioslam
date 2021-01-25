% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function figh = plotClinicalHipAngles(time,flexion,adduction,rotation,label)
% plot hip angles by their clinical definition: positive in flexion, adduction, internal rotation
% INPUTS:
%    time in seconds
%    angles in radians
%    label: a string identifier to put in chart title. enter empty [] for nothing.
% OUTPUT:
%    figure handle figh
figh=figure('units','normalized','position',[.05 .05 .6 .8]);
subplot(3,1,1);
plot(time,rad2deg(flexion),'k','linewidth',2);
ylabel('\leftarrow extension | flexion \rightarrow'); grid on;
subplot(3,1,2);
plot(time,rad2deg(adduction),'k','linewidth',2);
ylabel('\leftarrow abduction | adduction \rightarrow'); grid on;
subplot(3,1,3);
plot(time,rad2deg(rotation),'k','linewidth',2);
ylabel('\leftarrow ext. rot. | int. rot. \rightarrow'); grid on;
xlabel('time (sec)');
if isempty(label)
    sgtitle('Hip angles (deg)');
else
    sgtitle(sprintf('Hip angles (deg) for ''%s''',label),'interpreter','none');
end
end

