// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include <mathutils.h>
#include "bioutils.h"

namespace bioutils{

    gtsam::Matrix get_R_Imu_to_Segment(const gtsam::Unit3 &rightAxis, const gtsam::Point3 &imuToProximalJointVec, const gtsam::Point3 &imuToDistalJointVec, bool useProximalAsMasterVec){
        // function to get R[IMU->Segment] given that you know the knee axis and the vector to the superior joint, both in the IMU frame
        // x: right, y: anterior, z: proximal (per Grood & Suntay 1983)
        // used for thighs, shanks, and feet. for pelvis, see bioutils::get_R_SacrumImu_to_Pelvis
        // if useProximalAsMasterVec=true, it will correct the rightAxis, otherwise, rightAxis is assumed master and the proximalvec is corrected to create an orthonormal coordinate system
        gtsam::Vector3 Fx=rightAxis.point3(); // right
        gtsam::Vector3 Fz= imuToProximalJointVec - imuToDistalJointVec; Fz.normalize(); // up/proximal/superior
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

    std::vector<gtsam::Rot3> get_R_Segment_to_N(const std::vector<gtsam::Rot3>& R_Imu_to_N,const gtsam::Unit3 &rightAxis, const gtsam::Point3 &imuToProximalJointVec, const gtsam::Point3 &imuToDistalJointVec, bool useProximalAsMasterVec){
        // () get constant imu to segment relationship
        const gtsam::Rot3 R_Imu_to_Segment=gtsam::Rot3(get_R_Imu_to_Segment(rightAxis,imuToProximalJointVec, imuToDistalJointVec, useProximalAsMasterVec));
        // () now loop over orientations and create R[Segment->N]
        std::vector<gtsam::Rot3> R_Segment_to_N(R_Imu_to_N.size());
        for(uint k=0; k<R_Imu_to_N.size();k++){
            R_Segment_to_N[k]=R_Imu_to_N[k]*R_Imu_to_Segment.inverse(); // R[Segment->N]=R[IMU->N]*R[Imu->Segment]'
        }
        return R_Segment_to_N;
    }

    gtsam::Rot3 get_R_Segment_to_N(const gtsam::Rot3& R_Imu_to_N,const gtsam::Unit3 &rightAxis, const gtsam::Point3 &imuToProximalJointVec, const gtsam::Point3 &imuToDistalJointVec, bool useProximalAsMasterVec){
        // () get constant imu to segment relationship
        const gtsam::Rot3 R_Imu_to_Segment=gtsam::Rot3(get_R_Imu_to_Segment(rightAxis,imuToProximalJointVec, imuToDistalJointVec, useProximalAsMasterVec));
        // () now loop over orientations and create R[Segment->N]
        const gtsam::Rot3 R_Segment_to_N=R_Imu_to_N*R_Imu_to_Segment.inverse(); // R[Segment->N]=R[IMU->N]*R[Imu->Segment]'
        return R_Segment_to_N;
    }

    gtsam::Matrix get_R_SacrumImu_to_Pelvis(const gtsam::Vector3& upVec, const gtsam::Point3 &sacrumImuToRHipCtr, const gtsam::Point3 &sacrumImuToLHipCtr, bool useHipCtrAsMasterVec){
        // function to get R[SacrumIMU->Pelvis] given that you know the sacrum imu to hip ctrs vectors and some approximation of the vertical
        // if useHipCtrAsMasterVec=true, it will correct the vertical vector, otherwise, upVec is assumed master and the rightVec is corrected to create an orthonormal coordinate system
        // x: right, y: anterior, z: proximal (per Grood & Suntay 1983)*
        //    *Author's note: We assume the Grood coordiante system, even though in Wu 2002 (where this JCS for the hip was proposed) they assume y-up. We had to change one to keep the coordinate systems consistent, so I chose Grood.
        gtsam::Vector3 Fx=(sacrumImuToRHipCtr-sacrumImuToLHipCtr).normalized(); // right
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

    std::vector<gtsam::Rot3> get_R_Pelvis_to_N(const std::vector<gtsam::Rot3>& R_SacrumImu_to_N,const gtsam::Vector3& upVec, const gtsam::Point3 &sacrumImuToRHipCtr, const gtsam::Point3 &sacrumImuToLHipCtr, bool useHipCtrAsMasterVec){
        // () get constant imu to segment relationship
        const gtsam::Rot3 R_Imu_to_Segment=gtsam::Rot3(get_R_SacrumImu_to_Pelvis(upVec, sacrumImuToRHipCtr, sacrumImuToLHipCtr,useHipCtrAsMasterVec));
        // () now loop over orientations and create R[Segment->N]
        std::vector<gtsam::Rot3> R_Pelvis_to_N(R_SacrumImu_to_N.size());
        for(uint k=0; k<R_SacrumImu_to_N.size();k++){
            R_Pelvis_to_N[k]= R_SacrumImu_to_N[k]*R_Imu_to_Segment.inverse() ; // R[Segment->N]=R[IMU->N]*R[Imu->Segment]'
        }
        return R_Pelvis_to_N;
    }

    gtsam::Rot3 get_R_Pelvis_to_N(const gtsam::Rot3& R_SacrumImu_to_N,const gtsam::Vector3& upVec, const gtsam::Point3 &sacrumImuToRHipCtr, const gtsam::Point3 &sacrumImuToLHipCtr, bool useHipCtrAsMasterVec){
        // () get constant imu to segment relationship
        const gtsam::Rot3 R_Imu_to_Segment=gtsam::Rot3(get_R_SacrumImu_to_Pelvis(upVec, sacrumImuToRHipCtr, sacrumImuToLHipCtr,useHipCtrAsMasterVec));
        // () now loop over orientations and create R[Segment->N]
        const gtsam::Rot3 R_Pelvis_to_N = R_SacrumImu_to_N*R_Imu_to_Segment.inverse() ; // R[Segment->N]=R[IMU->N]*R[Imu->Segment]'
        return R_Pelvis_to_N;
    }

    Eigen::RowVector3d consistent3DofJcsAngles(const gtsam::Rot3& R_proximalSeg_to_N, const gtsam::Rot3& R_distalSeg_to_N){
        // *consistent joint angles given proximal and distal coordinate systems
        // returns a RowVector of angles in radians, as described below.
        // ----- construction of proximal and distal segment systems ----- //
        // i.e., R_proximalSeg_to_N and R_distalSeg_to_N
        // both coordinate systems are represented as rotation convention R[B->N]
        // both coordinate systems have local axes +x to the subject's right, +z proximal, and y from cross product (convention per Grood and Suntay 1983)
        // --------------------------------------------------------------- //
        // --------- "consistent" vs "clinical" angle definitions  ---------- //
        // IMPORTANT NOTE: Here we differentiate between "consistent" and "clinical" coordinate systems, where:
        //    clinical => angles are mirrored, s.t. angle names take on their implied clinical definition. so, e.g., right knee internal rotation would be the same rotation direction as left knee external rotation. angles are symmetric according to the body sagittal plane.
        //    consistent => angles are consistent according to rotation frames. i.e., not clinical. no fixing of signs of the angles to make them consistent with clinical definitions--a rotation of the right leg means that same rotation on the left leg.
        // ------------------------------------------------------------------ //
        // ----- how angles are computed ----- //
        // per ISB recommendations (Grood and Suntay 1983),
        //     - flexion/extension occurs about proximal segment right axis (I or e1 below)
        //     - internal/external rotation occurs about distal segment proximal vector (k or e3 below)
        //     - adduction/abduction occurs about floating e2 axis, e2=cross(e3,e1)
        // here, sign convention is determined by right-hand rule, i.e.,
        //    - flexion/extension is distal IMU positive rotation about +x (i.e., knee extension and hip flexion are positive)
        //    - internal/external rotation is distal IMU positive rotation about +z (i.e., right leg internal rotation and left leg external rotation are positive)
        //    - ab/ad is distal IMU prox vec. positive rotation about +y (i.e., right leg abduction and left leg adduction are positive)
        // in general, unsigned angles between vectors a and b are computed as angle = atan2(norm(cross(a,b)), dot(a,b)) + a rotation reference vector for sign
        //    - this will always return an angle on [-pi,pi], and has better numerical properties than using acos()
        //    - see mathutils::signedAngle(a,b,refVec) for details
        //    - we will use the expected rotation axis (above) as our reference vector
        // ----------------------------------- //
        // ---------- explanation for original concerns about consistency ---------- //
        //    - I originally wanted the sign-from-crossproduct interpretation and the *distal IMU* rotation by Rot3::AxisAngle(v,ang) to be consistent... but they weren't.
        //    - explanation: it is entirely arbitrary to choose to rotate the distal IMU by this Rot3. If you chose the proximal IMU, it would be consistent.
        //        - therefore I think it's best to just stick with the literal geometric interpretation of the cross product and be careful to document what the convention is.
        // ------------------------------------------------------------------------- //
        // ---------- editing proximal & distal segments to produce desired rotation angles ---------- //
        // if you want to produce a desired change in angle theta (***for external/internal rot and flex/ex only!),
        //     this is equivalent to rotating the proximal segment (IMU or anatomical) by R[B-B2] = gtsam::Rot3::AxisAngle(v,+theta)
        //                        or rotating the distal segment by R[B-B2] = gtsam::Rot3::AxisAngle(v,-theta)
        //     where v is the associated rotation axis (k for external/internal rotation, or I for flexion/extension)*
        //         *remember that you would have to rotate this axis into the associated frame
        //     this is because the angle is always taken as cross([proximal frame vec],[distal frame vec])
        // in the case of ab/ad, since this angle is always taken as unsignedAngle(I,k), the interpretation of increasing vs. decreasing angle is fixed:
        //    it's only a function of whether I and k point "more" or "less" toward each other.
        //    assuming I is always pointing to the subject's right, and k always points proximal, then the angle strictly gets larger when the distal segment rotates outward (i.e., abduction for right leg)
        //    or*: a negative rotation of the distal segment about the shared anterior vector e2=cross(k,I) (or a positive rotation of the prox segment about e2)
        //        *INCONSISTENT: this doesn't seem to be what actually works in the code. can't figure that out right now.
        // ------------------------------------------------------------------------------------------- //
        // () turn coordinate systems into JCS (Grood & Suntay) vectors i, j, k, I, J, K.
        //    where [i,j,k] are the x y z axes of the distal frame and [I,J,K] are the x y z axes of the proximal frame
        // and of course, note that these need to be rotated into the nav frame.
        Eigen::Vector3d i=R_distalSeg_to_N*Eigen::Vector3d(1.0,0.0,0.0); // i in nav frame
        Eigen::Vector3d j=R_distalSeg_to_N*Eigen::Vector3d(0.0,1.0,0.0); // j in nav frame
        Eigen::Vector3d k=R_distalSeg_to_N*Eigen::Vector3d(0.0,0.0,1.0); // k in nav frame
        Eigen::Vector3d I=R_proximalSeg_to_N*Eigen::Vector3d(1.0,0.0,0.0); // I in nav frame
        Eigen::Vector3d J=R_proximalSeg_to_N*Eigen::Vector3d(0.0,1.0,0.0); // J in nav frame
        Eigen::Vector3d K=R_proximalSeg_to_N*Eigen::Vector3d(0.0,0.0,1.0); // K in nav frame
        // () now rename to the clinical axes which define the axes of rotation
        Eigen::Vector3d e1=I, e3=k; // don't need to normalize these--their cross product gets normalized
        Eigen::Vector3d e2=(e3.cross(e1)).normalized(); // e2 floats, defined from cross product k x I
        // --- internal/external rotation angle calculation (occurs about distal segment's proximal axis) --- //
        double externalRotationAngleRad=mathutils::signedAngle(e2, j, k);
        //    - this uses k as a reference vector for sign, i.e., if cross(e2,j) points in same direction as k this is a positive angle
        //    - since j is the anterior vec in distal frame & e2 is the reference shared floating anterior vector, the interpretation is then
        //        that the angle is positive iff j is left rotated relative to e2, i.e., left rotation is positive (as in internal rotation of the right leg or external rotation of the left leg)
        // -------------------------------------------------------------------------------------------------- //
        // -------- flexion/extension angle calculation (occurs about proximal segment's right axis) -------- //
        double flexionAngleRad = mathutils::signedAngle(J, e2, I);
        //    - this uses I as a reference vector for sign, i.e., if cross(J,e2) points in the same direction as I this is a positive angle
        //    - since J is the anterior vec in proximal frame & e2 is the reference shared floating anterior vector, the interpretation is then
        //        that the angle is positive iff J is "downward" pointing relative to e2, i.e., knee extension/hip flexion is positive
        // -------------------------------------------------------------------------------------------------- //
        // --------- adduction/abduction angle calculation (occurs about floating anterior axis e2) --------- //
        double adductionAngleRad = mathutils::unsignedAngle(I, k) - M_PI / 2.0;
        //    - since e2=cross(k,I), then mathutils::signedAngle(I,k,e2) would always return a positive angle!
        //         - then ab/ad is determined from off from 90 degrees this value is, i.e.,
        //         - if unsignedAngle(k,I) = 90 deg, this is zero ab/ad
        //         - if unsignedAngle(k,I) = 95 deg, then the distal segment is ab/ad'd in the direction of I by 5 degrees
        //         - if unsignedAngle(k,I) = 85 deg, then the distal segment is ab/ad'd away from the direction of I by 5 degrees
        //    - LIMITATION: important to note that you cannot spin to ab/ad angle with absolute value greater than pi/2, because beta is strictly on [0,pi]. so abs(abad)>pi/2 is impossible.
        //    - therefore, this returns positive/increasing angles for abduction of the right leg, or adduction of the left leg joints
        //    - note: cannot use signedAngle() here because cross(I,k) and e2 are the same vector, making this function poorly conditioned (cross of a vector with itself is gonna return unexpected results due to numerical inaccuracies)
        // -------------------------------------------------------------------------------------------------- //
        return {flexionAngleRad, adductionAngleRad, externalRotationAngleRad}; // that's all, folks
    }

    Eigen::MatrixXd consistent3DofJcsAngles(const std::vector<gtsam::Rot3>& R_proximalSeg_to_N, const std::vector<gtsam::Rot3>& R_distalSeg_to_N, const bool& angleUnwrap){
        // vector version of the function of the same name for single Rot3s. See comments in consistent3DofJcsAngles(const gtsam::Rot3& R_proximalSeg_to_N, const gtsam::Rot3& R_distalSeg_to_N)
        // if angleUnwrap=true, also applied angle unwrapping algorithm
        // arg checking
        assert(R_proximalSeg_to_N.size()==R_distalSeg_to_N.size());
        // go
        Eigen::MatrixXd jointAngles(R_proximalSeg_to_N.size(),3);
        for(uint k=0; k<jointAngles.rows();k++){
            jointAngles.block<1,3>(k,0)=consistent3DofJcsAngles(R_proximalSeg_to_N[k], R_distalSeg_to_N[k]);
        }
        if(angleUnwrap){ // perform angle unwrap
            // we assume that the flex/ex and int/ext rot angles are on [-pi,pi] and the ab/ad angle is on [-pi/2,pi/2]
            jointAngles.col(0)=mathutils::angleUnwrapOnKnownDomain(jointAngles.col(0), -1.0*M_PI, M_PI,0.95 * 2.0 * M_PI); // flex/ex is on [-pi,pi], so let's say jump tol is 0.95*2*M_PI
            jointAngles.col(1)=mathutils::angleUnwrapOnKnownDomain(jointAngles.col(1), -1.0*M_PI/2.0, M_PI/2.0, 0.95 * M_PI); // ab/ad is on [-pi/2,pi/2], so let's say jump tol is 0.95*M_PI
            jointAngles.col(2)=mathutils::angleUnwrapOnKnownDomain(jointAngles.col(2), -1.0*M_PI, M_PI,0.95 * 2.0 * M_PI); // int/ext rot is on [-pi,pi], so let's say jump tol is 0.95*2*M_PI
        }
        return jointAngles; // now return as [flexion,adduction,externalRot].
    }

    Eigen::RowVector3d clinical3DofJcsAngles(const gtsam::Rot3& R_proximalSeg_to_N, const gtsam::Rot3& R_distalSeg_to_N, const bool& isRightLeg, const bool& isHip){
        // --- remember sign convention from consistent JCS angles --- //
        // flex/ex (first angle) is positive in knee extension and hip flexion
        // int/ext rot (third angle) is positive in internal rotation of right leg, external rotation of left leg
        // ab/ad (second angle) is positive in abduction of right leg/adduction of left leg
        // ----------------------------------------------------------- //
        // --- this function enforces the following "clinical" convention: --- //
        // Knee angles are positive in:
        //    extension, XXX, XXX
        // Hip angles are positive in:
        //    XXX, XXX, XXX
        // ------------------------------------------------------------------- //
        Eigen::RowVector3d clinicalAngles;
        if(isRightLeg){ // already correct in the consistent angles
            clinicalAngles=consistent3DofJcsAngles(R_proximalSeg_to_N, R_distalSeg_to_N);
        }else{ // swap signs on int/ext rot and ab/ad
            Eigen::RowVector3d consistentAngles=consistent3DofJcsAngles(R_proximalSeg_to_N, R_distalSeg_to_N);
            clinicalAngles(0)=consistentAngles(0); // flex/ex angle, + => flexion for both legs
            clinicalAngles(1)=-consistentAngles(1); // ab/ad angle, + => XXX (for both legs, because we flipped the sign here)
            clinicalAngles(2)=-consistentAngles(2); // int/ext rot angle, + => XXX (for both legs, because we flipped the sign here)
        }
        return clinicalAngles;
    }

    Eigen::MatrixXd clinical3DofJcsAngles(const std::vector<gtsam::Rot3>& R_proximalSeg_to_N, const std::vector<gtsam::Rot3>& R_distalSeg_to_N, const bool& isRightLeg, const bool& isHip, const bool& angleUnwrap){
        // vectorized version of function of same name. see it for documentation.
        // arg checking
        assert(R_proximalSeg_to_N.size()==R_distalSeg_to_N.size());
        // go
        Eigen::MatrixXd clinicalAngles(R_proximalSeg_to_N.size(),3);
        for(uint k=0; k<R_proximalSeg_to_N.size(); k++){
            clinicalAngles.block<1,3>(k,0)=clinical3DofJcsAngles(R_proximalSeg_to_N[k], R_distalSeg_to_N[k],isRightLeg,isHip);
        }
        if(angleUnwrap){ // perform angle unwrap
            // we assume that the flex/ex and int/ext rot angles are on [-pi,pi] and the ab/ad angle is on [-pi/2,pi/2]
            clinicalAngles.col(0)=mathutils::angleUnwrapOnKnownDomain(clinicalAngles.col(0), -1.0*M_PI, M_PI,0.95 * 2.0 * M_PI); // flex/ex is on [-pi,pi], so let's say jump tol is 0.95*2*M_PI
            clinicalAngles.col(1)=mathutils::angleUnwrapOnKnownDomain(clinicalAngles.col(1), -1.0*M_PI/2.0, M_PI/2.0, 0.95 * M_PI); // ab/ad is on [-pi/2,pi/2], so let's say jump tol is 0.95*M_PI
            clinicalAngles.col(2)=mathutils::angleUnwrapOnKnownDomain(clinicalAngles.col(2), -1.0*M_PI, M_PI,0.95 * 2.0 * M_PI); // int/ext rot is on [-pi,pi], so let's say jump tol is 0.95*2*M_PI
        }
        return clinicalAngles; // now return as [flexion,adduction,externalRot].
    }
}