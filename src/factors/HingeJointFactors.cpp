// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include "factors/HingeJointFactors.h"

namespace bioslam {

    // -------------------------------------------------------------------------------------------------------------- //
    // ------------------------------------- vector error version of this factor ------------------------------------ //
    // -------------------------------------------------------------------------------------------------------------- //
    gtsam::Vector HingeJointConstraintVecErrEstAngVel::evaluateError(const gtsam::Pose3 &xA, const gtsam::Vector3 &omegaA, const gtsam::Pose3 &xB, const gtsam::Vector3 &omegaB, const gtsam::Unit3 &axisA,
                                                                     boost::optional<gtsam::Matrix &> H1, // <- also include optional derivatives
                                                                                  boost::optional<gtsam::Matrix &> H2,
                                                                     boost::optional<gtsam::Matrix &> H3,
                                                                     boost::optional<gtsam::Matrix &> H4,
                                                                     boost::optional<gtsam::Matrix &> H5) const {
        // --- calculate error ---
        gtsam::Vector3 errorVec=errorModel(xA,omegaA,xB,omegaB,axisA);
        // --- optionally handle derivatives ---
        if (H1) {
            gtsam::Matrix36 derr_dxA;
            errorModel(xA,omegaA,xB,omegaB,axisA,derr_dxA,boost::none,boost::none,boost::none,boost::none);
            *H1=derr_dxA;
        }
        if (H2) {
            gtsam::Matrix33 derr_dwA;
            errorModel(xA,omegaA,xB,omegaB,axisA,boost::none,derr_dwA,boost::none,boost::none,boost::none);
            *H2=derr_dwA;
        }
        if (H3) {
            gtsam::Matrix36 derr_dxB;
            errorModel(xA,omegaA,xB,omegaB,axisA,boost::none,boost::none,derr_dxB,boost::none,boost::none);
            *H3=derr_dxB;
        }
        if (H4) {
            gtsam::Matrix33 derr_dwB;
            errorModel(xA,omegaA,xB,omegaB,axisA,boost::none,boost::none,boost::none,derr_dwB,boost::none);
            *H4=derr_dwB;
        }
        if (H5) {
            gtsam::Matrix32 derr_dkA;
            errorModel(xA,omegaA,xB,omegaB,axisA,boost::none,boost::none,boost::none,boost::none,derr_dkA);
            *H5=derr_dkA;
        }
        return errorVec;
    };

    gtsam::Vector3 HingeJointConstraintVecErrEstAngVel::errorModel(const gtsam::Pose3& xA, const gtsam::Vector3& wA, const gtsam::Pose3& xB, const gtsam::Vector3& wB, const gtsam::Unit3& kA,
                                                                   boost::optional<gtsam::Matrix36 &> H_xA,
                                                                   boost::optional<gtsam::Matrix33 &> H_wA,
                                                                   boost::optional<gtsam::Matrix36 &> H_xB,
                                                                   boost::optional<gtsam::Matrix33 &> H_wB,
                                                                   boost::optional<gtsam::Matrix32 &> H_kA){
        // ---------------------------------------------------------------------------------------------------------- //
        // implements error model of the factor written for the proximal frame hinge axis kA, along with computing optional jacobians.
        // INPUTS:
        //    xA: SE(3) pose of the proximal frame IMU (gtsam::Pose3)
        //    wA: R(3) estimated angular velocity of the proximal frame IMU (gtsam::Vector3)
        //    xB: SE(3) pose of the distal frame IMU (gtsam::Pose3)
        //    wB: R(3) estimated angular velocity of the distal frame IMU (gtsam::Vector3)
        //    kA: SU(2) unit axis, representing the hinge axis of rotation in the proximal IMU frame (gtsam::Unit3)
        //    H_xA, H_wA, H_xB, H_wB, H_kA: (optional) jacobians w.r.t. each of the input variables
        // OUTPUTS:
        //    3x1 error vector, representing the error vector of projecting the relative angular velocity of the joint onto hinge axis kA
        // ---------------------------------------------------------------------------------------------------------- //
        // ---- an easier-to-read version of this model would be: ---- //
        // gtsam::Vector3 m = R_A_to_N.inverse().matrix()*R_B_to_N.matrix() * omegaB - omegaA; // w[B-A] in A
        // gtsam::Vector3 err = m - m.dot(k) * k;
        // ----------------------------------------------------------- //
        gtsam::Matrix36 drA_dxA, drB_dxB;
        gtsam::Rot3 rA=xA.rotation(drA_dxA), rB=xB.rotation(drB_dxB); // pull out SO(3) rotation components from SE(3) poses
        gtsam::Matrix33 dwBN_dwB, dwBN_drB;
        gtsam::Vector3 wBN=rB.rotate(wB,dwBN_drB,dwBN_dwB); // wB[B] in nav frame
        gtsam::Matrix33 dwBNA_drA,dwBNA_dwBN;
        gtsam::Vector3 wBNA=rA.unrotate(wBN,dwBNA_drA,dwBNA_dwBN); // wB[N] rotated into A frame
        gtsam::Matrix33 dm_dwBNA, dm_dwA;
        gtsam::Vector3 m=mathutils::sub(wBNA, wA, dm_dwBNA, dm_dwA); // m=wB[A]-wA[A] (in A frame)
        gtsam::Matrix33 dmdrtr_dm; gtsam::Matrix32 dmdrtr_dkA;
        gtsam::Vector3 mdrtr = mathutils::projmk(m, kA, dmdrtr_dm, dmdrtr_dkA); // mdrtr = projection of m onto k
        gtsam::Matrix33 derr_dm, derr_dmdrtr;
        gtsam::Vector3 err = mathutils::sub(m, mdrtr, derr_dm, derr_dmdrtr); // err = m - m.dot(k) * k;
        // dmdrtr_dm & dmdrtr_dk
        if(H_xA){ // derr_dxA
            // derr_dxA = derr_dmdrtr*dmdrtr_dmdr*dmdr_dm*dm_dwBNA*dwBNA_drA*drA_dxA + derr_dm*dm_dwBNA*dwBNA_drA*drA_dxA
            //          = derr_dmdrtr*dmdrtr_dm*dm_dwBNA*dwBNA_drA*drA_dxA + derr_dm*dm_dwBNA*dwBNA_drA*drA_dxA
            *H_xA=derr_dmdrtr*dmdrtr_dm*dm_dwBNA*dwBNA_drA*drA_dxA+derr_dm*dm_dwBNA*dwBNA_drA*drA_dxA;
        }
        if(H_wA){
            // derr_dwA = derr_dmdrtr*dmdrtr_dmdr*dmdr_dm*dm_dwA+derr_dm*dm_dwA
            *H_wA=derr_dmdrtr*dmdrtr_dm*dm_dwA+derr_dm*dm_dwA;
        }
        if(H_xB){
            // derr_dxB = derr_dmdrtr*dmdrtr_dmdr*dmdr_dm*dm_dwBNA*dwBNA_dwBN*dwBN_drB*drB_dxB + derr_dm*dm_dwBNA*dwBNA_dwBN*dwBN_drB*drB_dxB
            *H_xB=derr_dmdrtr*dmdrtr_dm*dm_dwBNA*dwBNA_dwBN*dwBN_drB*drB_dxB + derr_dm*dm_dwBNA*dwBNA_dwBN*dwBN_drB*drB_dxB;
        }
        if(H_wB){
            // derr_dwB = derr_dmdrtr*dmdrtr_dmdr*dmdr_dm*dm_dwBNA*dwBNA_dwBN*dwBN_dwB + derr_dm*dm_dwBNA*dwBNA_dwBN*dwBN_dwB
            *H_wB= derr_dmdrtr*dmdrtr_dm*dm_dwBNA*dwBNA_dwBN*dwBN_dwB + derr_dm*dm_dwBNA*dwBNA_dwBN*dwBN_dwB;
        }
        if(H_kA){
            // derr_dkA = derr_dmdrtr*dmdrtr_dmdr*dmdr_dr*dr_dkA + derr_dmdrtr*dmdrtr_dr*dr_dkA
            //          = derr_dmdrtr*(dmdrtr_dmdr*dmdr_dr*dr_dkA + dmdrtr_dr*dr_dkA)
            *H_kA=derr_dmdrtr*dmdrtr_dkA;
        }
        return err;
    }

    gtsam::Vector HingeJointConstraintVecErrEstAngVel::evaluateErrorNoJacCall(const gtsam::Pose3 &xA, const gtsam::Vector3 &omegaA, const gtsam::Pose3 &xB, const gtsam::Vector3 &omegaB, const gtsam::Unit3 &axisA) const {
        return evaluateError(xA, omegaA, xB, omegaB, axisA, boost::none, boost::none, boost::none, boost::none,boost::none);
    }

    // -------------------------------------------------------------------------------------------------------------- //
    // -------------------------------------- norm error version of this factor ------------------------------------- //
    // -------------------------------------------------------------------------------------------------------------- //
    gtsam::Vector HingeJointConstraintNormErrEstAngVel::evaluateError(const gtsam::Pose3 &xA, const gtsam::Vector3 &omegaA, const gtsam::Pose3 &xB, const gtsam::Vector3 &omegaB, const gtsam::Unit3 &axisA,
                                                                      boost::optional<gtsam::Matrix &> H1, // <- also include optional derivatives
                                                                                  boost::optional<gtsam::Matrix &> H2,
                                                                      boost::optional<gtsam::Matrix &> H3,
                                                                      boost::optional<gtsam::Matrix &> H4,
                                                                      boost::optional<gtsam::Matrix &> H5) const {
        // --- calculate error ---
        gtsam::Vector1 errorVec=errorModel(xA,omegaA,xB,omegaB,axisA);
        // --- optionally handle derivatives ---
        if (H1) {
            gtsam::Matrix16 derr_dxA;
            errorModel(xA,omegaA,xB,omegaB,axisA,derr_dxA,boost::none,boost::none,boost::none,boost::none);
            *H1=derr_dxA;
        }
        if (H2) {
            gtsam::Matrix13 derr_dwA;
            errorModel(xA,omegaA,xB,omegaB,axisA,boost::none,derr_dwA,boost::none,boost::none,boost::none);
            *H2=derr_dwA;
        }
        if (H3) {
            gtsam::Matrix16 derr_dxB;
            errorModel(xA,omegaA,xB,omegaB,axisA,boost::none,boost::none,derr_dxB,boost::none,boost::none);
            *H3=derr_dxB;
        }
        if (H4) {
            gtsam::Matrix13 derr_dwB;
            errorModel(xA,omegaA,xB,omegaB,axisA,boost::none,boost::none,boost::none,derr_dwB,boost::none);
            *H4=derr_dwB;
        }
        if (H5) {
            gtsam::Matrix12 derr_dkA;
            errorModel(xA,omegaA,xB,omegaB,axisA,boost::none,boost::none,boost::none,boost::none,derr_dkA);
            *H5=derr_dkA;
        }
        return errorVec;
    };

    gtsam::Vector1 HingeJointConstraintNormErrEstAngVel::errorModel(const gtsam::Pose3& xA, const gtsam::Vector3& wA, const gtsam::Pose3& xB, const gtsam::Vector3& wB, const gtsam::Unit3& kA,
                                                                    boost::optional<gtsam::Matrix16 &> H_xA,
                                                                    boost::optional<gtsam::Matrix13 &> H_wA,
                                                                    boost::optional<gtsam::Matrix16 &> H_xB,
                                                                    boost::optional<gtsam::Matrix13 &> H_wB,
                                                                    boost::optional<gtsam::Matrix12 &> H_kA){
        // err model is just like the vector version of the factor, but norm'd
        // call the original error o
        gtsam::Matrix36 do_dxA, do_dxB; gtsam::Matrix33 do_dwA, do_dwB; gtsam::Matrix32 do_dkA;
        gtsam::Vector3 o=HingeJointConstraintVecErrEstAngVel::errorModel(xA, wA, xB, wB, kA, do_dxA, do_dwA, do_dxB, do_dwB, do_dkA);
        gtsam::Matrix13 derr_do;
        double err=gtsam::norm3(o,derr_do);
        if(H_xA){ *H_xA=derr_do*do_dxA; }
        if(H_wA){ *H_wA=derr_do*do_dwA; }
        if(H_xB){ *H_xB=derr_do*do_dxB; }
        if(H_wB){ *H_wB=derr_do*do_dwB; }
        if(H_kA){ *H_kA=derr_do*do_dkA; }
        return gtsam::Vector1(err);
    }
    gtsam::Vector HingeJointConstraintNormErrEstAngVel::evaluateErrorNoJacCall(const gtsam::Pose3 &xA, const gtsam::Vector3 &omegaA, const gtsam::Pose3 &xB, const gtsam::Vector3 &omegaB, const gtsam::Unit3 &axisA) const {
        return evaluateError(xA, omegaA, xB, omegaB, axisA, boost::none, boost::none, boost::none, boost::none,boost::none);
    }

} // namespace