% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function figh=visualizeColoredJacobian(m,matrixName,varargin)
%VISUALIZECOLOREDMATRIX Summary of this function goes here
%   Detailed explanation goes here
if nargin==6 % you input [factorRows,factorDescrip,variableCols,variableDescrip]
    % () add this data as a tooltip to the jacobian graph!
    factorRows=varargin{1}; factorDescrip=varargin{2}; varCols=varargin{3}; varDescrip=varargin{4};
end

zeroTol=.0001;



figh=jacobianMagnitudePlot(m,factorRows,factorDescrip,varCols,varDescrip);
figh2=booleanJacobianPlot(m,zeroTol,factorRows,factorDescrip,varCols,varDescrip);

addTitleAndStatsToFigTop(figh,strcat(matrixName,' Magnitude'),m);
addTitleAndStatsToFigTop(figh2,strcat(matrixName,' Boolean'),m);


% if false % print
%     factorRows
%     factorDescrip
%     varCols'
%     varDescrip
% end

% close all;
end

function figh=booleanJacobianPlot(m,zeroTol,factorRows,factorDescrip,varCols,varDescrip)
jacBool=abs(m)>zeroTol;
figh=figure('units','inches','position',[.5 .5 10 10]);
imagesc(~jacBool);
colormap(gray(2));
% title(sprintf('jacobian entries with abs()>%.2g',zeroTol));
setOutsideMatrixLabels(figh,factorRows,factorDescrip,varCols,varDescrip);
end

function addTitleAndStatsToFigTop(figh,ttl,m)
figure(figh);
% jacobian stats
eigs=eig(m);
mdet=det(m);
mrank=rank(m);
mineigval=min(eigs); maxeigval=max(eigs);
conditionnum=maxeigval/mineigval;
toptext=sprintf('%dx%d %s: rank=%d, det=%.4e, min/max eigval=%.4f/%.4f => cond. num=%.4f',size(m,1),size(m,2),ttl,mrank,mdet,mineigval,maxeigval,conditionnum);
h=suptitle(toptext);
set(h,'FontSize',10);
end

function figh=jacobianMagnitudePlot(jac,factorRows,factorDescrip,varCols,varDescrip)
figh=figure('units','inches','position',[.5 .5 10 10]);
myim=imagesc(jac);
colorbar;
setOutsideMatrixLabels(figh,factorRows,factorDescrip,varCols,varDescrip);
% editColormapWhiteZeros(colormap,m);
end

function setOutsideMatrixLabels(figh,factorRows,factorDescrip,varCols,varDescrip)
colorPlotAxes=findFigureSingleAxesHandle(figh);
set(colorPlotAxes,'Units','normalized','position',[.3 .1 .6 .6]);
% create factor label axes
factorLabelAxes=axes('units','normalized','position',[.05 .1 .2 .6]);
pause(.0000001);
set(factorLabelAxes,'YDir','Reverse');
% () now constuct gray lines using plot() to segment the factors
lineDividerColor=[.2 .2 .2];
line('XData',[0 1],'YData',[0 0],'Color',lineDividerColor); % first line, at top
lineYLoc(1)=0;
sectionCtrY=[];
for k=1:length(factorRows)
   % add line after this section
   lineYLoc(k+1)=double(sum(factorRows(1:k)))+.5; % remember on an imagesc you need to put the line at the 0.5 point between "pixels"
   line('XData',[0 1],'YData',[lineYLoc(k+1) lineYLoc(k+1)],'Color',lineDividerColor);
   % also get the center of the section, for text label purposes
   sectionCtrY(k)=mean([lineYLoc(k) lineYLoc(k+1)]);
   % add text label for factordescrip here
   text(.1,sectionCtrY(k),factorDescrip{k},'VerticalAlignment','middle');
end
ylim([0 double(sum(factorRows))+.5]);
set(factorLabelAxes,'visible','off');
%%% () now is time for the top sections--the variable names
varLabelAxes=axes;
pause(.000001);
set(varLabelAxes,'units','normalized','position',[colorPlotAxes.Position(1) colorPlotAxes.Position(2)+colorPlotAxes.Position(4)+.025 colorPlotAxes.Position(3) .2]);
% () now construct gray lines for dividers between variables
line('XData',[0 0],'YData',[0 1],'Color',lineDividerColor); % first line, at left
lineXLoc(1)=0; sectionCtrX=[];
for k=1:length(varCols)
   % add line after this section
   lineXLoc(k+1)=double(sum(varCols(1:k)))+.5; % remember on an imagesc you need to put the line at the 0.5 point between "pixels"
   line('XData',[lineXLoc(k+1) lineXLoc(k+1)],'YData',[0 1],'Color',lineDividerColor);
   % also get the center of the section, for text label purposes
   sectionCtrX(k)=mean([lineXLoc(k) lineXLoc(k+1)]);
   % add text label for factordescrip here
   text(sectionCtrX(k),.3,varDescrip{k},'HorizontalAlignment','center');
end
xlim([0 double(sum(varCols))+.5]);
set(varLabelAxes,'visible','off');
end

function ax=findFigureSingleAxesHandle(figh)
% find the axes handle in a figure and assert it's the only one
ax=findobj( get(figh,'Children'), '-depth', 1, 'type', 'axes');
assert(length(ax)==1);
end

function editColormapWhiteZeros(map,jac)
% take in a colormap Nx3 matrix, and jacobian to figure out which indeces to recolor to get white zeros
n=size(map,1);
linJac=linspace(round(min(min(jac))),round(max(max(jac))),n);
% idxsOfZeroInMap=
end