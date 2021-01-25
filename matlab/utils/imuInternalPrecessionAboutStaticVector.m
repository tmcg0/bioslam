% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function angles= imuInternalPrecessionAboutStaticVector(R_B_to_N, vproxB, seedStartVec)
% you've input a set of orientations R[B->N] and a vector v which is constant in the body frame
% compute the total angle of how much the IMU rotates about this vector in the nav frame
% motivation: the yaw angle is hard to interpret, this is sorta like yaw, but a concept of yaw which is consistent to the internal skeleton of the human
% INPUTS:
%    R_B_to_N: an N-array of gtsam::Rot3 objects
%    vproxB: a 3-vector to the proximal joint center
%    seed start vector = a 3-vector
% ----------------------------------------------------------------------- %
% ---- this is how the original algo looked, but I realized that if you just calculate the angle differential at each iteration and add them up at the end it takes care of unwrapping issues for you.
%{
% now, we ask the question: what's the angle between orthoVec(initial) and vBnorm over time?
for k=1:length(R_B_to_N)
    orthoVecNav=R_B_to_N(k).matrix()*orthoVecB;
    if k==1 % if first iteration, record the initial orthoVecNav
        orthoVecNavInit=orthoVecNav;
    end
    % () pull orthoVecNavInit back into the body frame
    orthoVecNavInitBody=R_B_to_N(k).inverse().matrix()*orthoVecNavInit;
    % () now correct it so that it's in the appropriate plane: orthogonal to vBnorm
    orthoVecNavInitBodyCorrected=cross((cross(vproxBnorm,orthoVecNavInitBody)),vproxBnorm);
    % what's the angle between orthoVecNav and orthoVecNavInit?
    angles(k)=signedAngleBetweenVectorsInSamePlane(orthoVecB, orthoVecNavInitBodyCorrected, vproxBnorm);
    if abs(angle_between_vectors_deg_eitherDir(vproxBnorm',orthoVecNavInitBodyCorrected')-90)>1e-10 % this angle should be 90 if your projections are correct
        error('"angle was not 90 degrees');
    end
end
%}
% ----------------------------------------------------------------------- %
% second version of algo: calculate delta angle from [k-1]->[k] and add up at end to naturally avoid wrapping issues

% setup
angles=zeros(length(R_B_to_N),1);
% find an orthogonal vector to vB using the cross product and seedStartVec
seedStartVecB=seedStartVec(:)./sqrt(sum(seedStartVec(:).^2,1));
vproxBnorm=vproxB(:)./sqrt(sum(vproxB(:).^2,1));
orthoVecB=cross(vproxBnorm,seedStartVecB); % so, orthoVec is orthogonal to vB
orthoVecB=orthoVecB(:)./sqrt(sum(orthoVecB(:).^2,1)); % normalize! vproxBnorm and seedStartVecB aren't garaunteed to be orthogonal
orthoVecNavInit=R_B_to_N(1).matrix()*orthoVecB;
dIPAngle=zeros(length(R_B_to_N)-1,1); % change in internal precession angle
for k=2:length(R_B_to_N)
    % () pull orthoVecNavInit back into the body frame
    orthoVecNavInitBodyKm1=R_B_to_N(k-1).inverse().matrix()*orthoVecNavInit; % k-1
    orthoVecNavInitBodyK=R_B_to_N(k).inverse().matrix()*orthoVecNavInit;
    % () quick debug check on angle between orthoVecNavInitBody and vproxBnorm
    checkAngBwK=abs(angle_between_vectors_deg(orthoVecNavInitBodyK',vproxBnorm'));
    if checkAngBwK>170
        warning(sprintf('k=%d: init. orthoVecN rotated into B frame [%.3f %.3f %.3f] and proximal vector [%.3f %.3f %.3f] are %.3f deg apart. Near 180, this can lead to wrapping issues.',k,orthoVecNavInitBodyK',vproxBnorm',checkAngBwK));
    end
    % () now correct it so that it's in the appropriate plane: orthogonal to vBnorm
    tempVecKm1=(cross(vproxBnorm,orthoVecNavInitBodyKm1)); % temp vector used in cross product to correct orthoVecNavInitBody
    orthoVecNavInitBodyCorrectedKm1=cross(tempVecKm1,vproxBnorm);
    orthoVecNavInitBodyCorrectedKm1=orthoVecNavInitBodyCorrectedKm1./sqrt(sum(orthoVecNavInitBodyCorrectedKm1.^2,1));
    tempVecK=(cross(vproxBnorm,orthoVecNavInitBodyK)); % temp vector used in cross product to correct orthoVecNavInitBody
    orthoVecNavInitBodyCorrectedK=cross(tempVecK,vproxBnorm);
    orthoVecNavInitBodyCorrectedK=orthoVecNavInitBodyCorrectedK./sqrt(sum(orthoVecNavInitBodyCorrectedK.^2,1));
    % what's the angle between: [k]-[k-1]
    dIPAngle(k-1)=signedAngleBetweenVectorsInSamePlane(orthoVecNavInitBodyCorrectedKm1, orthoVecNavInitBodyCorrectedK, vproxBnorm);
%     if abs(angle_between_vectors_deg_eitherDir(vproxBnorm',orthoVecNavInitBodyCorrected')-90)>1e-10 % this angle should be 90 if your projections are correct
%         error('"angle was not 90 degrees');
%     end
end
angles(2:end)=cumsum(dIPAngle);

% ----------------------------------------------------------------------- %
% third version of algo: at each iteration k, find the rotation transformation that brings your inital ortho vec into the kth body frame.
% this is simpler than the others, but more importantly, it avoids an undefined behavior noted below.
%{
angles=zeros(length(R_B_to_N),1);
% find an orthogonal vector to vB using the cross product and seedStartVec
seedStartVecB=seedStartVec(:)./sqrt(sum(seedStartVec(:).^2,1));
vproxBnorm=vproxB(:)./sqrt(sum(vproxB(:).^2,1));
orthoVecB=cross(vproxBnorm,seedStartVecB)'; % so, orthoVec is orthogonal to vB
orthoVecB=orthoVecB./sqrt(sum(orthoVecB.^2,2)); % normalize! vproxBnorm and seedStartVecB aren't garaunteed to be orthogonal
assert((abs(angle_between_vectors_deg(orthoVecB,vproxBnorm'))-90)<1e-10,'your ortho vec isn''t actually orthogonal');
orthoVecBInitAtK=zeros(length(R_B_to_N),3); % this variable carries the original orthovec through all orientations k for comparison to orthoVecB
orthoVecBInitAtK(1,:)=orthoVecB; % at k=1
for k=2:length(R_B_to_N)
    % () find R[B1->Bk]
    R_Bkm1_to_Bk=R_B_to_N(k).inverse().matrix()*R_B_to_N(k-1).matrix(); % R[B1->Bk] = R[Bk->N]'*R[B1->N]
    % () now find orthoVecB in current frame, using the previous definition of ortho vec but rotated into current frame
    orthoVecBkUncorrected=(R_Bkm1_to_Bk*orthoVecBInitAtK(k-1,:)')';
    % () now, orthoVecBk has to be corrected to be orthogonal to vprox. but this correction should be small! (unlike the undefined behavior problem in the second version of this algo above where the correction would be huge)
    orthoVecBInitAtK(k,:)=projectVectorIntoPlane(orthoVecBkUncorrected,vproxBnorm)'; % the corrected version you store for the next iteration!
    assert((abs(angle_between_vectors_deg(orthoVecBInitAtK(k,:),vproxBnorm'))-90)<1e-10,'your ortho vec at k isn''t actually orthogonal');
    % () measure angle between orthoVecB and orthoVecBinitAtK
    angles(k)=signedAngleBetweenVectorsInSamePlane(orthoVecB, orthoVecBInitAtK(k,:), vproxBnorm);
end
%}
% ----------------------------------------------------------------------- %
% fourth version of this algo: like the third, but the delta version to correct for wrapping errors easily
%{
angles=zeros(length(R_B_to_N),1);
% find an orthogonal vector to vB using the cross product and seedStartVec
seedStartVecB=seedStartVec(:)./sqrt(sum(seedStartVec(:).^2,1));
vproxBnorm=vproxB(:)./sqrt(sum(vproxB(:).^2,1));
orthoVecB=cross(vproxBnorm,seedStartVecB)'; % so, orthoVec is orthogonal to vB
orthoVecB=orthoVecB./sqrt(sum(orthoVecB.^2,2)); % normalize! vproxBnorm and seedStartVecB aren't garaunteed to be orthogonal
assert((abs(angle_between_vectors_deg(orthoVecB,vproxBnorm'))-90)<1e-10,'your ortho vec isn''t actually orthogonal');
deltaIPAngle=zeros(length(R_B_to_N)-1,1); % change in internal precession angle
for k=2:length(R_B_to_N)
    % () find R[Bkm1->Bk]
    R_Bkm1_to_Bk=R_B_to_N(k).inverse().matrix()*R_B_to_N(k-1).matrix(); % R[B1->Bk] = R[Bk->N]'*R[B1->N]
    % () now find orthoVecB in current frame, using the previous definition of ortho vec but rotated into current frame
    orthoVecBkm1AtKUncorrected=(R_Bkm1_to_Bk*orthoVecB')'; % so this takes the ortho vector from k-1 and finds what it would be at k
    % () now, orthoVecBkm1AtKUncorrected has to be corrected to be orthogonal to vprox. but this correction should be small! (unlike the undefined behavior problem in the second version of this algo above where the correction would be huge)
    orthoVecBkm1AtK=projectVectorIntoPlane(orthoVecBkm1AtKUncorrected,vproxBnorm)'; % the corrected version you store for the next iteration!
    assert((abs(angle_between_vectors_deg(orthoVecBkm1AtK,vproxBnorm'))-90)<1e-10,'your ortho vec at k isn''t actually orthogonal');
    % () measure angle between orthoVecB and orthoVecBkm1AtK -- this is your delta
    deltaIPAngle(k-1)=signedAngleBetweenVectorsInSamePlane(orthoVecB, orthoVecBkm1AtK, vproxBnorm);
end
angles(2:end)=cumsum(deltaIPAngle);
%}
end

function newvec=projectVectorIntoPlane(vec,planeNormalVec)
vec=vec(:); planeNormalVec=planeNormalVec(:);
tempVec=(cross(planeNormalVec,vec)); % temp vector used in cross product to correct orthoVecNavInitBody
newvec=cross(tempVec,planeNormalVec);
% check correction magnitude
checkAng=angle_between_vectors_deg_eitherDir(vec',newvec');
if abs(checkAng)>90
   warning(sprintf('an angle correction of %.2f degrees occured',checkAng)); 
end
end

% debugging Mar 24 2020: sometimes angle is near pi between k and k-1, how?
%    - noted that indeed orthoVecNavInitBodyCorrectedKm1 and orthoVecNavInitBodyCorrectedK point in opposite directions
%        - this should be really improbable. IMU shouldn't rotate 180 degrees in one iteration.
%    - so going back and looking at the movement of ortho nav vec into body frame
%        - and indeed, orthoVecNavInitBodyKm1 and orthoVecNavInitBodyK are very similar, so this is right.
%    - so something has to be going wrong when correcting these to be orthogonal to the prox vector
%        - the temp vecs are small in magnitude and don't seem to be pointing in the same direction, must be this. what causes this?
%        - it looks like vproxBnorm and orthoVecNavInitBody are nearly 180 degrees apart, so when you take their cross it has small magnitude.
%        - when these vecs get nearly 180 apart, that would obviously lead to some undefined behavior.
%    fix: at each iteration k, rather than recomputing the new ortho vec in the body frame,
%        (which has potentially undefined behavior above)
%        take the previous corrected one (which is already unit and orthogonal to vprox) and rotate it barely so that
