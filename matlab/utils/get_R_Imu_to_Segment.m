% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function R_Imu_to_Segment=get_R_Imu_to_Segment(rightAxis, imuToProximalJointVec, imuToDistalJointVec,useProximalAsMasterVec)
% change any gtsam types to native MATLAB types
if isa(rightAxis,'gtsam.Unit3'); rightAxis=rightAxis.point3().vector(); end
if isa(imuToProximalJointVec,'gtsam.Point3'); imuToProximalJointVec=imuToProximalJointVec.vector(); end
if isa(imuToDistalJointVec,'gtsam.Point3'); imuToDistalJointVec=imuToDistalJointVec.vector(); end
assert(isa(rightAxis,'double')); assert(isa(imuToProximalJointVec,'double')); assert(isa(imuToDistalJointVec,'double'));
Fx=rightAxis; % right
Fz= imuToProximalJointVec - imuToDistalJointVec; Fz=Fz./sqrt(sum(Fz.^2,1)); % up/proximal/superior
Fy=cross(Fz,Fx); Fy=Fy./sqrt(sum(Fy.^2,1)); % anterior, from cross product
if(useProximalAsMasterVec) % leave Fz alone and correct Fx
    Fx=cross(Fy,Fz); % CROSS PRODUCT CORRECTION: Use the proximal vector as the master vector in the orthonormal system
    assert(abs(1.0-norm(Fx))<1.0e-10); % sanity check: this vector should be normalized by now!
else % leave Fx alone and correct Fz
    Fz=cross(Fx,Fy); % CROSS PRODUCT CORRECTION: Use the right vector as the master vector in the orthonormal system
    assert(abs(1.0-norm(Fz))<1.0e-10); % sanity check: this vector should be normalized by now!
end
% now get the rotation matrix and test it
R_Imu_to_Segment=zeros(3,3);
R_Imu_to_Segment(1,1:3)=Fx'; % verified: this places Fx as the top row.
R_Imu_to_Segment(2,1:3)=Fy';
R_Imu_to_Segment(3,1:3)=Fz';
assert(abs(1.0-det(R_Imu_to_Segment))<1.0e-10); % verify it's a proper DCM
% sanity check: rotate back the master vector and assert it's correct
if useProximalAsMasterVec
    assert(all(abs([0.0,0.0,1.0]' - (R_Imu_to_Segment*Fz)) < 1.0e-12)); % sanity check: when rotated back, Fz should be [0 0 1]'
else
    assert(all(abs([1.0,0.0,0.0]' - (R_Imu_to_Segment*Fx)) < 1.0e-12)); % sanity check: when rotated back, Fx should be [1 0 0]'
end
end
%{
    gtsam::Matrix get_R_Imu_to_Segment(const gtsam::Unit3 &rightAxis, const gtsam::Point3 &imuToProximalJointVec, const gtsam::Point3 &imuToDistalJointVec, bool useProximalAsMasterVec){
        // function to get R[IMU->Segment] given that you know the knee axis and the vector to the superior joint, both in the IMU frame
        // x: right, y: anterior, z: proximal (per Grood & Suntay 1983)
        // used for thighs, shanks, and feet. for pelvis, see bioutils::get_R_SacrumImu_to_Pelvis
        // if useProximalAsMasterVec=true, it will correct the rightAxis, otherwise, rightAxis is assumed master and the proximalvec is corrected to create an orthonormal coordinate system
        gtsam::Vector3 Fx=rightAxis.point3().vector(); // right
        gtsam::Vector3 Fz= imuToProximalJointVec.vector() - imuToDistalJointVec.vector(); Fz.normalize(); // up/proximal/superior
        gtsam::Vector3 Fy=Fz.cross(Fx); Fy.normalize(); // anterior, from cross product
        if(useProximalAsMasterVec){ // leave Fz alone and correct Fx
            Fx=Fy.cross(Fz); // CROSS PRODUCT CORRECTION: Use the proximal vector as the master vector in the orthonormal system
            assert(abs(1.0-Fx.norm())<1.0e-10); // sanity check: this vector should be normalized by now!
        }else{ // leave Fx alone and correct Fz
            Fz=Fx.cross(Fy); // CROSS PRODUCT CORRECTION: Use the right vector as the master vector in the orthonormal system
            assert(abs(1.0-Fz.norm())<1.0e-10); // sanity check: this vector should be normalized by now!
        }
        // now get the rotation matrix and test it
        gtsam::Matrix33 R_Imu_to_Segment;
        R_Imu_to_Segment.block<1,3>(0,0)=Fx.transpose(); // verified: this places Fx as the top row.
        R_Imu_to_Segment.block<1,3>(1,0)=Fy.transpose();
        R_Imu_to_Segment.block<1,3>(2,0)=Fz.transpose();
        assert(abs(1.0-R_Imu_to_Segment.determinant())<1.0e-10); // verify it's a proper DCM
        // sanity check: rotate back the master vector and assert it's correct
        if(useProximalAsMasterVec){
            assert((gtsam::Vector3(0.0,0.0,1.0).transpose() - (R_Imu_to_Segment*Fz).transpose()).norm() < 1.0e-12); // sanity check: when rotated back, Fz should be [0 0 1]'
        }else{
            assert((gtsam::Vector3(1.0,0.0,0.0).transpose() - (R_Imu_to_Segment*Fx).transpose()).norm() < 1.0e-12); // sanity check: when rotated back, Fx should be [1 0 0]'
        }
        return R_Imu_to_Segment;
    }
%}
