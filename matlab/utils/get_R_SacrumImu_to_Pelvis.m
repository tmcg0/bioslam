% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function R_SacrumImu_to_Pelvis=get_R_SacrumImu_to_Pelvis(upVec, sacrumImuToRHipCtr,sacrumImuToLHipCtr,useHipCtrAsMasterVec)
% convert gtsam types to native MATLAB types
if isa(upVec,'gtsam.Unit3'); upVec=rightAxis.point3().vector(); end
if isa(sacrumImuToRHipCtr,'gtsam.Point3'); sacrumImuToRHipCtr=sacrumImuToRHipCtr.vector(); end
if isa(sacrumImuToLHipCtr,'gtsam.Point3'); sacrumImuToLHipCtr=sacrumImuToLHipCtr.vector(); end
assert(isa(upVec,'double')); assert(isa(sacrumImuToRHipCtr,'double')); assert(isa(sacrumImuToLHipCtr,'double'));
% go
Fx=(sacrumImuToRHipCtr-sacrumImuToLHipCtr); Fx=Fx/sqrt(sum(Fx.^2,1)); % right
Fz= upVec(:); Fz=Fz/sqrt(sum(Fz.^2,1));% up/proximal/superior
Fy=cross(Fz,Fx); Fy=Fy/sqrt(sum(Fy.^2,1)); % anterior, from cross product
if useHipCtrAsMasterVec % leave Fx alone and correct Fz
    Fz=cross(Fx,Fy); % CROSS PRODUCT CORRECTION: Use the right vector as the master vector in the orthonormal system
    assert(abs(1.0-sqrt(sum(Fz.^2,1)))<1.0e-10); % sanity check: this vector should be normalized by now!
else % leave Fz alone and correct Fx
    Fx=cross(Fy,Fz); % CROSS PRODUCT CORRECTION: Use the proximal vector as the master vector in the orthonormal system
    assert(abs(1.0-sqrt(sum(Fx.^2,1)))<1.0e-10); % sanity check: this vector should be normalized by now!
end
% now get the rotation matrix and test it
R_SacrumImu_to_Pelvis=[Fx';Fy';Fz'];
assert(abs(1.0-det(R_SacrumImu_to_Pelvis))<1.0e-10); % verify it's a proper DCM
% sanity check: rotate back the master vector and assert it's correct
if useHipCtrAsMasterVec
    assert( all([1 0 0] - (R_SacrumImu_to_Pelvis*Fx)' < 1.0e-12) ); % sanity check: when rotated back, Fx should be [1 0 0]'
else
    assert( all([0 0 1] - (R_SacrumImu_to_Pelvis*Fz)' < 1.0e-12) ); % sanity check: when rotated back, Fz should be [0 0 1]'
end
end

%{
gtsam::Matrix get_R_SacrumImu_to_Pelvis(const gtsam::Vector3& upVec, const gtsam::Point3 &sacrumImuToRHipCtr, const gtsam::Point3 &sacrumImuToLHipCtr, bool useHipCtrAsMasterVec){
        // function to get R[SacrumIMU->Pelvis] given that you know the sacrum imu to hip ctrs vectors and some approximation of the vertical
        // if useHipCtrAsMasterVec=true, it will correct the vertical vector, otherwise, upVec is assumed master and the rightVec is corrected to create an orthonormal coordinate system
        // x: right, y: anterior, z: proximal (per Grood & Suntay 1983)*
        //    *Author's note: We assume the Grood coordiante system, even though in Wu 2002 (where this JCS for the hip was proposed) they assume y-up. We had to change one to keep the coordinate systems consistent, so I chose Grood.
        gtsam::Vector3 Fx=(sacrumImuToRHipCtr.vector()-sacrumImuToLHipCtr.vector()).normalized(); // right
        gtsam::Vector3 Fz= upVec.normalized(); // up/proximal/superior
        gtsam::Vector3 Fy=Fz.cross(Fx); Fy.normalize(); // anterior, from cross product
        if(useHipCtrAsMasterVec){ // leave Fx alone and correct Fz
            Fz=Fx.cross(Fy); // CROSS PRODUCT CORRECTION: Use the right vector as the master vector in the orthonormal system
            assert(abs(1.0-Fz.norm())<1.0e-10); // sanity check: this vector should be normalized by now!
        }else{ // leave Fz alone and correct Fx
            Fx=Fy.cross(Fz); // CROSS PRODUCT CORRECTION: Use the proximal vector as the master vector in the orthonormal system
            assert(abs(1.0-Fx.norm())<1.0e-10); // sanity check: this vector should be normalized by now!
        }
        // now get the rotation matrix and test it
        gtsam::Matrix33 R_Imu_to_Segment;
        R_Imu_to_Segment.block<1,3>(0,0)=Fx.transpose(); // verified: this places Fx as the top row.
        R_Imu_to_Segment.block<1,3>(1,0)=Fy.transpose();
        R_Imu_to_Segment.block<1,3>(2,0)=Fz.transpose();
        assert(abs(1.0-R_Imu_to_Segment.determinant())<1.0e-10); // verify it's a proper DCM
        // sanity check: rotate back the master vector and assert it's correct
        if(useHipCtrAsMasterVec){
            assert((gtsam::Vector3(1.0,0.0,0.0).transpose() - (R_Imu_to_Segment*Fx).transpose()).norm() < 1.0e-12); // sanity check: when rotated back, Fx should be [1 0 0]'
        }else{
            assert((gtsam::Vector3(0.0,0.0,1.0).transpose() - (R_Imu_to_Segment*Fz).transpose()).norm() < 1.0e-12); // sanity check: when rotated back, Fz should be [0 0 1]'
        }
        return R_Imu_to_Segment;
    }
%}