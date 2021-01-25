% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function combineH5Files(path)
outputDir=fullfile(path,'combinedFiles'); mkdir(outputDir);
% combined .h5 files in path to a single .h5 file, matching them according to the following token:
pattern='(.+)_(\d+)of(\d+).+\.h5';
% () find all .h5 files in this directory
d=dir('*.h5');
d=d(~ismember({d.name},{'.','..'}));
filenames={d.name};
% () pull out matches
[mat,tok,ext]=regexp(filenames,pattern,'match','tokens','tokenExtents');
% () group files
infoArray=cell(length(filenames),5); % {file name, file group name, number of file group, file #, out of #}
infoArray(:,1)=repmat({' '},length(filenames),1);
infoArray(:,2)=repmat({' '},length(filenames),1);
infoArray(:,3:5)={0};
for k=1:length(filenames)
    assert(length(tok{k}{1})==3); % should have found three tokens
    infoArray{k,2}=tok{k}{1}{1}; % set group name
    % if you haven't found this name yet, add a new index for it.
    groupMatches=strcmp(infoArray(1:k-1,2),tok{k}{1}{1});
    if any(groupMatches) % then look up its group number and set that
        infoArray{k,3}=infoArray{find(groupMatches,1,'first'),3};
    else % create new group number
        infoArray{k,3}=max([infoArray{:,3}])+1;
    end
    % set the file name, number and its out of number
    infoArray{k,1}=mat{k}{1}; infoArray{k,4}=str2num(tok{k}{1}{2}); infoArray{k,5}=str2num(tok{k}{1}{3});
end
% () sort file groups by file number
for g=min([infoArray{:,3}]):max([infoArray{:,3}]) % g iterates the group
    idxs=[infoArray{:,3}]==g;
    groupInfoArray=infoArray(idxs,:);
    [~,srtIdxs]=sort([groupInfoArray{:,4}]);
    groupInfoArraySorted=groupInfoArray(srtIdxs,:);
    infoArray(idxs,:)=groupInfoArraySorted;
end
% () for each file group, combine h5 files into a single file
for g=min([infoArray{:,3}]):max([infoArray{:,3}]) % g iterates the group
    %     fprintf('file group %d/%d: %s\n',g,max([infoArray{:,2}]),);
    groupIdxs=[infoArray{:,3}]==g;
    files=infoArray(groupIdxs,1);
    lbpe=lowerBodyPoseEstimator(); % blank
    for k=1:length(files) % k iterates the file
        newlbpe=lowerBodyPoseEstimator(files{k});
        lbpe.combineResults(newlbpe); % add newlbpe to results from lbpe
    end
    lbpe.saveResultsToH5(fullfile(outputDir,strcat(infoArray{find(groupIdxs,1,'first'),2},'.h5')));
end
end


