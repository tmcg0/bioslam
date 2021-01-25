% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function [flexExAngleRad,adAdAngleRad,intExtRotAngleRad]=consistent3DofJcsAngles(R_proximalSeg_to_N, R_distalSeg_to_N)
% *consistent joint angles given proximal and distal coordinate systems
% returns angles in radians, as described below.
% --- settings --- %
doAngleUnwrap=0; % if passed in array of segment orientations, should you attempt to unwrap the angles?
% ---------------- %
% ----- construction of proximal and distal segment systems ----- %
% i.e., R_proximalSeg_to_N and R_distalSeg_to_N
% both coordinate systems are represented as rotation convention R[B->N]
% both coordinate systems have local axes +x to the subject's right, +z proximal, and y from cross product (convention per Grood and Suntay 1983)
% --------------------------------------------------------------- %
% --------- "consistent" vs "clinical" angle definitions  ---------- %
% IMPORTANT NOTE: Here we differentiate between "consistent" and "clinical" coordinate systems, where:
%    clinical => angles are mirrored, s.t. angle names take on their implied clinical definition. so, e.g., right knee internal rotation would be the same rotation direction as left knee external rotation. angles are symmetric according to the body sagittal plane.
%    consistent => angles are consistent according to rotation frames. i.e., not clinical. no fixing of signs of the angles to make them consistent with clinical definitions--a rotation of the right leg means that same rotation on the left leg.
% ------------------------------------------------------------------ %
% ----- how angles are computed ----- %
% per ISB recommendations (Grood and Suntay 1983),
%     - flexion/extension occurs about proximal segment right axis (I or e1 below)
%     - internal/external rotation occurs about distal segment proximal vector (k or e3 below)
%     - adduction/abduction occurs about floating e2 axis, e2=cross(e3,e1)
% here, sign convention is determined by right-hand rule, i.e.,
%    - flexion/extension is distal IMU positive rotation about +x (i.e., knee extension and hip flexion are positive)
%    - internal/external rotation is distal IMU positive rotation about +z (i.e., right leg internal rotation and left leg external rotation are positive)
%    - ab/ad is distal IMU positive rotation about +y (i.e., right leg adduction and left leg abduction are positive)
% in general, unsigned angles between vectors a and b are computed as angle = atan2(norm(cross(a,b)), dot(a,b)) + a rotation reference vector for sign
%    - this will always return an angle on [-pi,pi], and has better numerical properties than using acos()
%    - see rotutils::signedAngle(a,b,refVec) for details
%    - we will use the expected rotation axis (above) as our reference vector
% ----------------------------------- %
% ---------- explanation for original concerns about consistency ---------- %
%    - I originally wanted the sign-from-crossproduct interpretation and the *distal IMU* rotation by Rot3::AxisAngle(v,ang) to be consistent... but they weren't.
%    - explanation: it is entirely arbitrary to choose to rotate the distal IMU by this Rot3. If you chose the proximal IMU, it would be consistent.
%        - therefore I think it's best to just stick with the literal geometric interpretation of the cross product and be careful to document what the convention is.
% ------------------------------------------------------------------------- %
% ---------- editing proximal & distal segments to produce desired rotation angles ---------- %
% if you want to produce a desired change in angle theta (***for external/internal rot and flex/ex only!),
%     this is equivalent to rotating the proximal segment (IMU or anatomical) by R[B-B2] = gtsam::Rot3::AxisAngle(v,+theta)
%                        or rotating the distal segment by R[B-B2] = gtsam::Rot3::AxisAngle(v,-theta)
%     where v is the associated rotation axis (k for external/internal rotation, or I for flexion/extension)*
%         *remember that you would have to rotate this axis into the associated frame
%     this is because the angle is always taken as cross([proximal frame vec],[distal frame vec])
% in the case of ab/ad, since this angle is always taken as unsignedAngle(I,k), the interpretation of increasing vs. decreasing angle is fixed:
%    it's only a function of whether I and k point "more" or "less" toward each other.
%    assuming I is always pointing to the subject's right, and k always points proximal, then the angle strictly gets larger when the distal segment rotates outward (i.e., abduction for right leg)
%    or*: a negative rotation of the distal segment about the shared anterior vector e2=cross(k,I) (or a positive rotation of the prox segment about e2)
%        *INCONSISTENT: this doesn't seem to be what actually works in the code. can't figure that out right now.
% ------------------------------------------------------------------------------------------- %
% matlab implementation of cpp function of the same name (below)
assert(isnumeric(R_proximalSeg_to_N)); assert(isnumeric(R_distalSeg_to_N));
% preallocate
N=size(R_proximalSeg_to_N,3);
flexExAngleRad=zeros(N,1); adAdAngleRad=zeros(N,1); intExtRotAngleRad=zeros(N,1);
% go
for c=1:N
    i=R_distalSeg_to_N(:,:,c)*[1.0,0.0,0.0]'; % i in nav frame
    j=R_distalSeg_to_N(:,:,c)*[0.0,1.0,0.0]'; % j in nav frame (unused so commented out)
    k=R_distalSeg_to_N(:,:,c)*[0.0,0.0,1.0]'; % k in nav frame
    I=R_proximalSeg_to_N(:,:,c)*[1.0,0.0,0.0]'; % I in nav frame
    J=R_proximalSeg_to_N(:,:,c)*[0.0,1.0,0.0]'; % J in nav frame (unused so commented out)
    K=R_proximalSeg_to_N(:,:,c)*[0.0,0.0,1.0]'; % K in nav frame
    % now rename to the clinical axes which define the angles
    e1=I/sqrt(sum(I.^2,1)); % e1 = proximal segment x, assumed to the subject's right
    e3=k/sqrt(sum(k.^2,1)); % e3 = distal segment z, assumed to be proximal
    e2=cross(e3,e1); e2=e2/sqrt(sum(e2.^2,1)); % e2 floats, defined from cross product. if subject were at zero angles, e2 would point anterior
    % --- internal/external rotation angle calculation (occurs about distal segment's proximal axis) --- %
    intExtRotAngleRad(c)=signedAngle(e2',j',k');
    %    - this uses k as a reference vector for sign, i.e., if cross(e2,j) points in same direction as k this is a positive angle
    %    - since j is the anterior vec in distal frame & e2 is the reference shared floating anterior vector, the interpretation is then
    %        that the angle is positive iff j is left rotated relative to e2, i.e., left rotation is positive (as in internal rotation of the right leg or external rotation of the left leg)
    % -------------------------------------------------------------------------------------------------- %
    % -------- flexion/extension angle calculation (occurs about proximal segment's right axis) -------- %
    flexExAngleRad(c) = signedAngle(J',e2',I');
    %    - this uses I as a reference vector for sign, i.e., if cross(J,e2) points in the same direction as I this is a positive angle
    %    - since J is the anterior vec in proximal frame & e2 is the reference shared floating anterior vector, the interpretation is then
    %        that the angle is positive iff J is "downward" pointing relative to e2, i.e., knee extension/hip flexion is positive
    % -------------------------------------------------------------------------------------------------- %
    % --------- adduction/abduction angle calculation (occurs about floating anterior axis e2) --------- %
    adAdAngleRad(c) = unsignedAngle(I',k') - pi/2;
    %    - since e2=cross(k,I), then rotutils::signedAngle(I,k,e2) would always return a positive angle!
    %         - then ab/ad is determined from off from 90 degrees this value is, i.e.,
    %         - if unsignedAngle(k,I) = 90 deg, this is zero ab/ad
    %         - if unsignedAngle(k,I) = 95 deg, then the distal segment is ab/ad'd in the direction of I by 5 degrees
    %         - if unsignedAngle(k,I) = 85 deg, then the distal segment is ab/ad'd away from the direction of I by 5 degrees
    %    - LIMITATION: important to note that you cannot spin to ab/ad angle with absolute value greater than pi/2, because beta is strictly on [0,pi]. so abs(abad)>pi/2 is impossible.
    %    - therefore, this returns positive/increasing angles for abduction of the right leg, or adduction of the left leg joints
    %    - note: cannot use signedAngle() here because cross(I,k) and e2 are the same vector, making this function poorly conditioned (cross of a vector with itself is gonna return unexpected results due to numerical inaccuracies)
    % -------------------------------------------------------------------------------------------------- %
end
% () optional: angle unwrap
if N>1 && doAngleUnwrap
    % we assume that the flex/ex and int/ext rot angles are on [-pi,pi] and the ab/ad angle is on [-pi/2,pi/2]
    flexExAngleRad=jointAngleUnwrap(flexExAngleRad,2*pi,0.95); % flex/ex is on [-pi,pi], so let's say jump tol is 0.95*2*M_PI
    adAdAngleRad=jointAngleUnwrap(adAdAngleRad,pi,0.95); % ab/ad is on [-pi/2,pi/2], so let's say jump tol is 0.95*M_PI
    intExtRotAngleRad=jointAngleUnwrap(intExtRotAngleRad,2*pi,0.95); % int/ext rot is on [-pi,pi], so let's say jump tol is 0.95*2*M_PI
end
end

function testFlexExRelationEquivalency(e2,J,K)
% two relations per grood for flexex:
% (1) cos(alpha) = dot(J,e2)
% (2) -sin(alpha) = dot(e2,K)
% are these the same?
rel1Alpha=acos(dot(J,e2));
rel2Alpha=asin(-dot(e2,K));
% assert(abs(abs(rel1Alpha)-abs(rel2Alpha))<1.0e-10); % asserts they only differ by -1 factor
% answer: no! they aren't even the same by absolute value (above line will fail)
end

% --- math notes ---
% if you want to change the flexion angle by altering the distal imu frame:
%    flexion angle is only a function of I, k, and K
%    and in this case, only k can vary
% flexion angle = acos(dot(J,cross(e3,e1))) /propto acos(dot(J,cross(k,I)))
% and we know that k = R_distalSeg_to_N(:,:,c)*[0.0,0.0,1.0]' (or the bottom row of that matrix)
% and % R[Segment->N]=R[IMU->N]*R[Imu->Segment]'
% so k= R[IMU->N]*R[Imu->Segment]'*[0.0,0.0,1.0]'
% adduction is only a function of I and k. if you want to change adduction by the distal frame, simply rotate the distal frame about its local anterior vector
% external rot is only a function of I, k, and i. and note that it is defined as a rotation about the proximal vector of the DISTAL frame (k)
