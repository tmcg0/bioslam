% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function reprocessCppResultsViaMatlab(d)
% The cpp results aren't producing great joint angles right now. To fix this, just input their .h5 results file, and then in MATLAB:
% 1) recompute joint angles
% 2) plot all the stuff
% 3) print the text files necessary for further processing
% 4) resave the h5 file
% INPUT argument d is the path which holds the .h5 files
%            or d is a specific file to reprocess
% fileList = dir(d);
% --- settings --- %
saveResultsH5File=1;
saveResultsTextFiles=1;
saveResultsPlots=1;
% ---------------- %

if exist(d,'file')==2 % it's a file, do only it
    files={'S4',d};
    outdir=fullfile(pwd,'matlabReprocess'); mkdir(outdir);
    d=[];
else
    outdir=fullfile(d,'matlabReprocess'); mkdir(outdir);
    % () create a cell array of every .h5 file in this directory
    x=dir(fullfile(d,'*.h5'));
    x=x(~ismember({x.name},{'.','..'}));
    files={x.name};
    %{
    % for matlab style
%             files={'S1','S1_LowerBody_Results.h5';
%             'S2','S2_LowerBody_Results.h5';
%             'S3','S3_LowerBody_Results.h5';
%             'S4','S4_LowerBody_Results.h5';
%             'S5','S5_LowerBody_Results.h5';
%             'S6','S6_LowerBody_Results.h5';
%             'S7','S7_LowerBody_Results.h5';
%             'S8','S8_LowerBody_Results.h5';
%             'S9','S9_LowerBody_Results.h5';
%             'S10','S10_LowerBody_Results.h5';
%             'S11','S11_LowerBody_Results.h5';
%             'S12','S12_LowerBody_Results.h5'};
    %}
end

lbpefiles={'S1','20190923-093410-S130MinWalk_Results.h5';
    'S2','20190923-112935-S230MinWalk_Results.h5';
    'S3','20190923-124627-S330MinWalk_Results.h5';
    'S4','20190924-082921-S430MinWalk_Results.h5';
    'S5','20190924-130724-S530MinWalk_Results.h5';
    'S6','20190925-161018-S630MinWalk_Results.h5';
    'S7','20190925-184737-S730MinWalk_Results.h5';
    'S8','20190926-085542-S830MinWalk_Results.h5';
    'S9','20190926-125442-S930MinWalk_Results.h5';
    'S10','20190927-085029-S1030MinWalk_Results.h5';
    'S11','20190927-101452-S1130MinWalk_Results.h5';
    'S12','20190927-113657-S1230MinWalk_Results.h5'
    'S1','S1_LowerBody_Results.h5';
    'S2','S2_LowerBody_Results.h5';
    'S3','S3_LowerBody_Results.h5';
    'S4','S4_LowerBody_Results.h5';
    'S5','S5_LowerBody_Results.h5';
    'S6','S6_LowerBody_Results.h5';
    'S7','S7_LowerBody_Results.h5';
    'S8','S8_LowerBody_Results.h5';
    'S9','S9_LowerBody_Results.h5';
    'S10','S10_LowerBody_Results.h5';
    'S11','S11_LowerBody_Results.h5';
    'S12','S12_LowerBody_Results.h5'};
% () create directory structure as requested
if saveResultsPlots
    plotdir=fullfile(outdir,'plots'); mkdir(plotdir);
end
if saveResultsH5File || saveResultsTextFiles
    datadir=fullfile(outdir,'data'); mkdir(datadir);
end
% construct lbpes and process 
for k=1:length(files)
    if exist(files{k},'file')==2 % file exists
        lbpe=lowerBodyPoseEstimator(fullfile(d,files{k}));
        % () if files{k} is a member of lbpefiles, give it shortcut id and label
        idx=find( strcmp(lbpefiles(:,2),files{k}) );
        if ~isempty(idx) % it's in there
            label=strcat(lbpefiles{idx,1},'_LowerBody');
            lbpe.id=lbpefiles{idx,1};
        else % it's not, give generic names
            [~,b,~]=fileparts(files{k});
            label=b;
            lbpe.id=b;
        end
        % () save/print as requested
        if saveResultsPlots
            lbpePlots(lbpe,plotdir,label);
        end
        if saveResultsTextFiles
            printLowerBodyProblemResultsToFilesDefault(lbpe,datadir,label);
        end
        if saveResultsH5File
            lbpe.saveResultsToH5(fullfile(datadir,strcat(label,'_Results.h5'))); % save all results to an .h5 file
        end
    else
        fprintf('could not find file %s\n',files{k,2});
    end
end
end

function filenameToSubject(imuDataDir,fileList)
allfiles=dir(imuDataDir);
allfilenames={allfiles.name};
correctFile=~cellfun(@isempty,[strfind(allfilenames,strcat('S',num2str(subj),'30MinWalk'))]);
assert(sum(correctFile)==1,sprintf('should only have one file found, but you found %d',sum(correctFile)));
end

function lbpePlots(lbpe,plotDir,label)
figh=plotKneeHingeAxesInNavFrame(lbpe);
savefig(figh,fullfile(plotDir,strcat(label,'_kneeHingeAxesErrNavFrame.fig'))); close(figh);
figh2=plotJointConnectionPositionErrors(lbpe);
savefig(figh2,fullfile(plotDir,strcat(label,'_allJointConnectionErr.fig'))); close(figh2);
figh3=plotRKneeAngles(lbpe);
savefig(figh3,fullfile(plotDir,strcat(label,'_rightKneeAngles.fig'))); close(figh3);
figh4=plotLKneeAngles(lbpe);
savefig(figh4,fullfile(plotDir,strcat(label,'_leftKneeAngles.fig'))); close(figh4);
figh5=plotRHipAngles(lbpe);
savefig(figh5,fullfile(plotDir,strcat(label,'_rightHipAngles.fig'))); close(figh5);
figh6=plotLHipAngles(lbpe);
savefig(figh6,fullfile(plotDir,strcat(label,'_leftHipAngles.fig'))); close(figh6);
%     figh7=plotOptimizerErrorOverTime(lbpe);
%     savefig(figh7,fullfile(plotDir,strcat(label,'_optimizationErrOverTime.fig'))); close(figh7);
%     figh8=plotJointConnectionVelocityErrors(lbpe);
%     savefig(figh8,fullfile(plotDir,strcat(label,'_allJointVelocityDiffs.fig'))); close(figh8);
figh9=plotYawAnglesFromMemberOrientations(lbpe);
savefig(figh9,fullfile(plotDir,strcat(label,'_allYawAngles.fig'))); close(figh9);
figh10=plotYawAngleBetweenConsecutiveJoints(lbpe);
savefig(figh10,fullfile(plotDir,strcat(label,'_relativeYawAngles.fig'))); close(figh10);
figh11=plotInternalPrecessionAngles(lbpe);
savefig(figh11,fullfile(plotDir,strcat(label,'_internalPrecessionAngles.fig'))); close(figh11);
figh12=plotRelativeInternalPrecessionAngles(lbpe);
savefig(figh12,fullfile(plotDir,strcat(label,'_relativeInternalPrecessionAngles.fig'))); close(figh12);
try
figh13=plotHipHingeAxesInNavFrame(lbpe);
savefig(figh13,fullfile(plotDir,strcat(label,'_hipHingeAxesErrNavFrame.fig'))); close(figh13);
catch
end
figh16=plotRKneeRelativeAngularVelocityVectors(lbpe);
savefig(figh16,fullfile(plotDir,strcat(label,'_rkneeRelAngVel.fig'))); close(figh16);
figh17=plotLKneeRelativeAngularVelocityVectors(lbpe);
savefig(figh17,fullfile(plotDir,strcat(label,'_lkneeRelAngVel.fig'))); close(figh17);
try
figh14=plotRHipRelativeAngularVelocityVectors(lbpe);
savefig(figh14,fullfile(plotDir,strcat(label,'_rhipRelAngVel.fig'))); close(figh14);
figh15=plotLHipRelativeAngularVelocityVectors(lbpe);
savefig(figh15,fullfile(plotDir,strcat(label,'_lhipRelAngVel.fig'))); close(figh15);
figh18=plotPelvisRelativeAngularVelocityVectors(lbpe);
savefig(figh18,fullfile(plotDir,strcat(label,'_pelvisRelAngVel.fig'))); close(figh18);
catch
end
fighImus=plotEstimatedImuStates(lbpe); % individual imu results
for k=1:length(fighImus) % save by figh.Name
    savefig(fighImus(k),fullfile(plotDir,strcat(fighImus(k).Name,'.fig')));
    close(fighImus(k)); % close them one at a time
end
%     plotKneeAxisFactorError(lbpe);
end