% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function [hingeAxisA,hingeAxisB,omegaRelBmA_A,omegaRelBmA_B]=hingeAxesAccordingToMcGrath2018(R_A_to_N,R_B_to_N,wA,wB)
% from the estimated angular velocity and orientations, perform McGrath et al. 2018 for hinge axes
% INPUTS:
% R_A_to_N: proximal frame orientation local->global
% R_B_to_N: distal frame orientation local->global
% wA:       [Nx3] array of angular velocities of proximal frame
% wB:       [Nx3] array of angular velocities of distal frame
% HOW TO INPUT ORIENTATIONS:
% a valid orientation input for R_A_to_N or R_B_to_N can be:
%    (1) an N-array of gtsam.Rot3 objects
%    (2) [3x3xN] direction cosine matrix array
%    (3) [Nx4] quaternion array
% OUTPUTS:
% hingeAxisA: static hinge axis represented in the proximal frame
% hingeAxisB: static hinge axis represented in the distal frame
% () transform input orientations to arrays of direction cosine matrices
R_A_to_N=inputOrientationToDcmArray(R_A_to_N);
R_B_to_N=inputOrientationToDcmArray(R_B_to_N);
% () arg checking
assert(isa(wA,'double')); assert(isa(wB,'double'));
assert(length(R_A_to_N)==length(R_B_to_N)); assert(length(R_A_to_N)==length(wA)); assert(length(R_A_to_N)==length(wB));
% preallocate
N=length(R_A_to_N);
omegaRelBmA_A=zeros(N,3); % relative angular velocity, shank minus thigh, right knee, thigh frame
omegaRelBmA_B=zeros(N,3); % relative angular velocity, shank minus thigh, right knee, shank frame
R_B_to_A=zeros(3,3,N);
for k=1:N
    R_B_to_A(:,:,k)=R_A_to_N(:,:,k)'*R_B_to_N(:,:,k); % R[S->T] = R[T->N]'*R[S->N]
    omegaRelBmA_A(k,:)=[R_B_to_A(:,:,k)*wB(k,:)' - wA(k,:)']'; % relative angular velocity, shank minus thigh, right knee, thigh frame
    omegaRelBmA_B(k,:)=[wB(k,:)' - R_B_to_A(:,:,k)'*wA(k,:)']'; % relative angular velocity, shank minus thigh, right knee, shank frame
end
% now use PCA to get the axes
hingeAxisA=pca(omegaRelBmA_A,'NumComponents',1)';
hingeAxisB=pca(omegaRelBmA_B,'NumComponents',1)';
end

function Rout=inputOrientationToDcmArray(R)
% transform input orientation to a [3x3xN] orientation matrix array
if length(size(R))==2 && isa(R,'gtsam.Rot3')
   Rout=Rot3ArrayToMatrices(R);
elseif length(size(R))==2 && isa(R,'double') && size(R,2)==4 % quaternion
   Rout=quatToDcm(R); % remember to use this implemntation, not MATLAB's quat2dcm(). This returns the inverse for some reason!
elseif length(size(R))==3 && size(R,1)==3 && size(R,2)==3
    % already good!
    Rout=R;
else
    error('unknown orientation representation');
end
end
