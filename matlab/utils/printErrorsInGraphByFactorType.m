% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function printErrorsInGraphByFactorType(graph,vals)
fv=NonlinearFactorGraphToFactorVector(graph);
typeStrings=cell(length(fv),1);
weightedErrors=zeros(length(fv),1);
for k=1:length(fv)
    typeStrings{k}=class(fv{k});
    weightedErrors(k)=fv{k}.error(vals);
end
% now get sums for each type
[C,~,ic]=unique(typeStrings);
weightedSum=zeros(length(C),1);
for j=1:length(C)
    weightedSum(j)=sum(weightedErrors(ic==j));
end
fprintf('sources of weighted error (%.5g) by factor type: (%d factors, %d Values)\n',sum(weightedSum),graph.size(),vals.size());
for j=1:length(C)
    fprintf('\t%s (x%d): %.4f%% (%.3f)\n',C{j},sum(ic==j),weightedSum(j)*100/sum(weightedSum), weightedSum(j));
end
end