% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

% getAnthroMeasure retrieves anthropometric measure for use in bioslam modeling

function outstat=getAnthroMeasure(measure,sample,stat)
% INPUTS:
% measure: a string denoting the name of the measurement
% sample: a string denoting the subsample to be retrieved. use 'a' for default all population. or use 'm' or 'f' for gender subsample.
% stat: statistic of the measure to return. e.g., 'mean' or 'std'
switch measure
    case 'tibialength'
        outstat=tibiaLength(sample,stat);
    case 'femurlength'
        outstat=femurLength(sample,stat);
    case 'hipbreadth' % (51) HIP BREADTH, SITTING from ANSUR II [1]
    case 'femoralheadseparation'
        outstat=femoralHeadSeparation(sample,stat);
    otherwise; error(sprintf('measure name ''%s'' not recongized',measure));
end
end

function hipBreadth(gender,stat)

end

function outstat=tibiaLength(gender,stat)
% tibia length is the same as the derived measurement 'calf link'
meanstd=calfLink(gender);
tibiaLengthMean=meanstd(1);
tibiaLengthStd=meanstd(2);
switch stat
    case 'mean'
        outstat=tibiaLengthMean;
    case 'std'
        outstat=tibiaLengthStd;
    otherwise error('what stat is this?');
end
end

function outstat=femurLength(gender,stat)
% femur length is the same as the derived measurement 'thigh link'
meanstd=thighLink(gender);
tibiaLengthMean=meanstd(1);
tibiaLengthStd=meanstd(2);
switch stat
    case 'mean'
        outstat=tibiaLengthMean;
    case 'std'
        outstat=tibiaLengthStd;
    otherwise error('what stat is this?');
end
end

function outstat=femoralHeadSeparation(gender,stat)
meanstd=getFemoralHeadSep(gender);
femHeadSepMean=meanstd(1);
femHeadSepStd=meanstd(2);
switch stat
    case 'mean'
        outstat=femHeadSepMean;
    case 'std'
        outstat=femHeadSepStd;
    otherwise error('what stat is this?');
end
end

%%%% raw measures

function meanstd=getFemoralHeadSep(gender)
% what we really care about isn't the pelvic width, per se, but the distance between the femoral head centers.
% according to [3], the distance was between femoral head center and pelvic midline
%   (N=500, 250 male, 250 female)
meanMidlineM=0.09502; stdMidlineM=0.00882; nSamplesM=250;
meanMidlineF=0.09154; stdMidlineF=0.00264; nSamplesF=250;
% and we know how to add two normal distributions to get the full separation
meanStdM=[meanMidlineM*2, sqrt(2*(stdMidlineM^2))]; % [mean, std]
meanStdF=[meanMidlineF*2, sqrt(2*(stdMidlineF^2))]; % [mean, std]
switch gender
    case 'm'
        meanstd=meanStdM; % m
        nSamples=nSamplesM;
    case 'f'
        meanstd=meanStdF; % m
        nSamples=nSamplesF;
    case 'a'
        [muHat,sigmaHat]=supersetGaussianSampling(meanStdM,nSamplesM,meanStdF,nSamplesF);
        meanstd=[muHat sigmaHat];
    otherwise error('gender unrecognized');
end
end

function meanstd=thighLink(gender)
% (D29) THIGH LINK [1]
% The vertical distance between the trochanterion landmark and the lateral femoral
% epicondyle landmark is calculated as follows: TROCHANTERION HEIGHT minus
% LATERAL FEMORAL EPICONDYLE HEIGHT. 
meanstdM=[40.93 2.84]/100; % m
nSamplesM=4082;
meanstdF=[37.94 2.40]/100; % m
nSamplesF=1986;
switch gender
    case 'm'
        meanstd=meanstdM; % m
        nSamples=nSamplesM;
    case 'f'
        meanstd=meanstdF; % m
        nSamples=nSamplesF;
    case 'a'
        [muHat,sigmaHat]=supersetGaussianSampling(meanstdM,nSamplesM,meanstdF,nSamplesF);
        meanstd=[muHat sigmaHat];
    otherwise error('gender unrecognized');
end
end

function meanstd=calfLink(gender)
% (D6) CALF LINK [1]
meanstdM=[41.88 2.46]/100; % m
nSamplesM=4082;
meanstdF=[40.32 2.59]/100; % m
nSamplesF=1986;
switch gender
    case 'm'
        meanstd=meanstdM; % m
        nSamples=nSamplesM;
    case 'f'
        meanstd=meanstdF; % m
        nSamples=nSamplesF;
    case 'a'
        [muHat,sigmaHat]=supersetGaussianSampling(meanstdM,nSamplesM,meanstdF,nSamplesF);
        meanstd=[muHat sigmaHat];
    otherwise error('gender unrecognized');
end
end

function meanstd=tibialHeight(gender)
% (81) TIBIAL HEIGHT [1]
meanstdM=[46.82 2.66]/100; % m
nSamplesM=4082;
meanstdF=[43.78 2.50]/100; % m
nSamplesF=1986;
switch gender
    case 'm'
        meanstd=meanstdM; % m
        nSamples=nSamplesM;
    case 'f'
        meanstd=meanstdF; % m
        nSamples=nSamplesF;
    case 'a'
        [muHat,sigmaHat]=supersetGaussianSampling(meanstdM,nSamplesM,meanstdF,nSamplesF);
        meanstd=[muHat sigmaHat];
    otherwise error('gender unrecognized');
end
end

function meanstd=lateralFemoralEpicondyleHeight(gender)
% (58) LATERAL FEMORAL EPICONDYLE HEIGHT [1]
meanstdM=[49.17 2.66]/100; % m
nSamplesM=4082;
meanstdF=[46.59 2.71]/100; % m
nSamplesF=1986;
switch gender
    case 'm'
        meanstd=meanstdM; % m
        nSamples=nSamplesM;
    case 'f'
        meanstd=meanstdF; % m
        nSamples=nSamplesF;
    case 'a'
        [muHat,sigmaHat]=supersetGaussianSampling(meanstdM,nSamplesM,meanstdF,nSamplesF);
        meanstd=[muHat sigmaHat];
    otherwise error('gender unrecognized');
end
end

function meanstd=lateralMalleolousHeight(gender)
% (59) LATERAL MALLEOLUS HEIGHT [1]
meanstdM=[7.29 0.57]/100; % m
nSamplesM=4082;
meanstdF=[6.27 0.51]/100; % m
nSamplesF=1986;
switch gender
    case 'm'
        meanstd=meanstdM; % m
        nSamples=nSamplesM;
    case 'f'
        meanstd=meanstdF; % m
        nSamples=nSamplesF;
    case 'a'
        [muHat,sigmaHat]=supersetGaussianSampling(meanstdM,nSamplesM,meanstdF,nSamplesF);
        meanstd=[muHat sigmaHat];
    otherwise error('gender unrecognized');
end
end

function [muHat,sigmaHat]=supersetGaussianSampling(meanstd1,n1,meanstd2,n2)
% find the superset gaussian through random sampling
nTaken=n1+n2;
nSamples=10000000; % number of random samples to take
% nSamples1=round(nSamples*(n1/nTaken)); nSamples2=round(nSamples*(n2/nTaken));
nSamples1=round(nSamples/2); nSamples2=round(nSamples/2);
r1 = normrnd(meanstd1(1),meanstd1(2),[nSamples1 1]);
r2 = normrnd(meanstd2(1),meanstd2(2),[nSamples2 1]);
r=vertcat(r1,r2);
[muHat,sigmaHat,muCI,sigmaCI] = normfit(r);
% could plot this distribution with
% figure; histfit(r); xlabel('m'); title(sprintf('mu=%.5f [%.5f %.5f], std=%.5f [%.5f %.5f]',muHat,muCI',sigmaHat,sigmaCI'));
% how does this compare to analytical expression?
muHatE=(meanstd1(1) + meanstd2(1))/2;
sigmaHatE=sqrt(meanstd1(2)^2+meanstd2(2)^2);
end

% REFERENCES:
% [1] ANSUR II
%   http://tools.openlab.psu.edu/publicData/ANSURII-TR15-007.pdf
% [2] Anthropometric Reference Data for Children and Adults: United States, 2011–2014
%   https://www.cdc.gov/nchs/data/series/sr_03/sr03_039.pdf
% [3] "The distance of the centre of femoral head relative to the midline of the pelvis: a prospective X-ray study of 500 adults"
%   https://pdfs.semanticscholar.org/34ae/ef7fe2ed8e59efaa1e211a7b72b5174b186c.pdf