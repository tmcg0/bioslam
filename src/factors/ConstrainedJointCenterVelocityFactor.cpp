// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include <factors/ConstrainedJointCenterVelocityFactor.h>
#include <gtsam/base/numericalDerivative.h>
#include <mathutils.h>

namespace bioslam {

        gtsam::Vector ConstrainedJointCenterVelocityFactor::evaluateError(const gtsam::Pose3& poseA, const gtsam::Vector3& linVelA, const gtsam::Vector3& angVelA, const gtsam::Point3& sA, const gtsam::Pose3& poseB, const gtsam::Vector3& linVelB, const gtsam::Vector3& angVelB, const gtsam::Point3& sB,
                                                                          boost::optional<gtsam::Matrix &> H1, // include optional jacobians H
                                                                          boost::optional<gtsam::Matrix &> H2,
                                                                          boost::optional<gtsam::Matrix &> H3,
                                                                          boost::optional<gtsam::Matrix &> H4,
                                                                          boost::optional<gtsam::Matrix &> H5,
                                                                          boost::optional<gtsam::Matrix &> H6,
                                                                          boost::optional<gtsam::Matrix &> H7,
                                                                          boost::optional<gtsam::Matrix &> H8) const {
            // ---- calculate error ----- //
            gtsam::Vector3 errorVec = errorModel(poseA,linVelA,angVelA,sA,poseB,linVelB,angVelB,sB);
            // --- handle derivatives --- //
            if(H1) {
                gtsam::Matrix36 derr_dxA;
                errorModel(poseA,linVelA,angVelA,sA,poseB,linVelB,angVelB,sB,derr_dxA,boost::none,boost::none,boost::none,boost::none,boost::none,boost::none,boost::none);
                *H1=derr_dxA;
            }
            if(H2) {
                gtsam::Matrix33 derr_dvA;
                errorModel(poseA,linVelA,angVelA,sA,poseB,linVelB,angVelB,sB,boost::none,derr_dvA,boost::none,boost::none,boost::none,boost::none,boost::none,boost::none);
                *H2=derr_dvA;
            }
            if(H3) {
                gtsam::Matrix33 derr_dwA;
                errorModel(poseA,linVelA,angVelA,sA,poseB,linVelB,angVelB,sB,boost::none,boost::none,derr_dwA,boost::none,boost::none,boost::none,boost::none,boost::none);
                *H3=derr_dwA;
            }
            if(H4) {
                gtsam::Matrix33 derr_dpA;
                errorModel(poseA,linVelA,angVelA,sA,poseB,linVelB,angVelB,sB,boost::none,boost::none,boost::none,derr_dpA,boost::none,boost::none,boost::none,boost::none);
                *H4=derr_dpA;
            }
            if(H5) {
                gtsam::Matrix36 derr_dxB;
                errorModel(poseA,linVelA,angVelA,sA,poseB,linVelB,angVelB,sB,boost::none,boost::none,boost::none,boost::none,derr_dxB,boost::none,boost::none,boost::none);
                *H5=derr_dxB;
            }
            if(H6) {
                gtsam::Matrix33 derr_dvB;
                errorModel(poseA,linVelA,angVelA,sA,poseB,linVelB,angVelB,sB,boost::none,boost::none,boost::none,boost::none,boost::none,derr_dvB,boost::none,boost::none);
                *H6=derr_dvB;
            }
            if(H7) {
                gtsam::Matrix33 derr_dwB;
                errorModel(poseA,linVelA,angVelA,sA,poseB,linVelB,angVelB,sB,boost::none,boost::none,boost::none,boost::none,boost::none,boost::none,derr_dwB,boost::none);
                *H7=derr_dwB;
            }
            if(H8) {
                gtsam::Matrix33 derr_dpB;
                errorModel(poseA,linVelA,angVelA,sA,poseB,linVelB,angVelB,sB,boost::none,boost::none,boost::none,boost::none,boost::none,boost::none,boost::none,derr_dpB);
                *H8=derr_dpB;
            }
            return errorVec;
        }

        gtsam::Vector3 ConstrainedJointCenterVelocityFactor::errorModel(const gtsam::Pose3& xA, const gtsam::Vector3& vA, const gtsam::Vector3& wA, const gtsam::Point3& sA, const gtsam::Pose3& xB, const gtsam::Vector3& vB, const gtsam::Vector3& wB, const gtsam::Point3& sB,
                                     boost::optional<gtsam::Matrix36 &> H_xA, // include optional jacobians H
                                     boost::optional<gtsam::Matrix33 &> H_vA,
                                     boost::optional<gtsam::Matrix33 &> H_wA,
                                     boost::optional<gtsam::Matrix33 &> H_sA,
                                     boost::optional<gtsam::Matrix36 &> H_xB,
                                     boost::optional<gtsam::Matrix33 &> H_vB,
                                     boost::optional<gtsam::Matrix33 &> H_wB,
                                     boost::optional<gtsam::Matrix33 &> H_sB){
            // --- implements error model of this factor --- //
            // model compares velocity of IMU A and IMU B in nav frame and expects zero difference
            // accounts for linear and angular velocity components
            // error = velA - velB ~ N([0 0 0], Sigma_3x3)
            //       = linVelA[N] + R[A->N]*( cross(angVelA,sA) ) - linVelA[B] - R[B->N]*( cross(angVelB,sB) ) ~ N([0 0 0], Sigma_3x3)
            // --------------------------------------------- //
            // t: the total velocity of a point in the navigation frame
            gtsam::Matrix36 dtA_dxA, dtB_dxB;
            gtsam::Matrix33 dtA_dvA, dtA_dwA, dtA_dsA, dtB_dvB, dtB_dwB, dtB_dsB;
            gtsam::Point3 tA=linearVelocity(xA,vA,wA,sA,dtA_dxA,dtA_dvA,dtA_dwA,dtA_dsA);
            gtsam::Point3 tB=linearVelocity(xB,vB,wB,sB,dtB_dxB,dtB_dvB,dtB_dwB,dtB_dsB);
            // error is tA - tB
            gtsam::Matrix33 derr_dtA, derr_dtB;
            gtsam::Point3 err=mathutils::sub(tA, tB, derr_dtA, derr_dtB);
            // handle all the derivatives
            if(H_xA){ *H_xA=derr_dtA*dtA_dxA; }
            if(H_vA){ *H_vA=derr_dtA*dtA_dvA; }
            if(H_wA){ *H_wA=derr_dtA*dtA_dwA; }
            if(H_sA){ *H_sA=derr_dtA*dtA_dsA; }
            if(H_xB){ *H_xB=derr_dtB*dtB_dxB; }
            if(H_vB){ *H_vB=derr_dtB*dtB_dvB; }
            if(H_wB){ *H_wB=derr_dtB*dtB_dwB; }
            if(H_sB){ *H_sB=derr_dtB*dtB_dsB; }
            return err;
        }

        gtsam::Vector3 ConstrainedJointCenterVelocityFactor::linearVelocity(const gtsam::Pose3& x, const gtsam::Vector3& v, const gtsam::Vector3& w, const gtsam::Vector3& p,
                                         boost::optional<gtsam::Matrix36 &> H_x, // include optional jacobians H
                                         boost::optional<gtsam::Matrix33 &> H_v,
                                         boost::optional<gtsam::Matrix33 &> H_w,
                                         boost::optional<gtsam::Matrix33 &> H_p){
            // ---- calculate the linear velocity of a point in the navigation frame which is rotating in its local frame --- //
            // INPUTS:
            //    - x: the pose of the body frame
            //    - v: the linear velocity of the frame in the navigation frame
            //    - w: the angular velocity of the point in the frame
            //    - p: a point in the body frame which is rotating with angular velocity w
            // OUTPUTS:
            //    the combined linear velocity of the point due to both frame linear velocity and angular velocity
            // --------------------------------------------------------------------------------------------------------------- //
            // total velocity = v+r*cross(w,p)
            gtsam::Matrix36 dr_dx;
            gtsam::Rot3 r=x.rotation(dr_dx); // R[B->N]
            gtsam::Matrix33 dc_dw, dc_dp;
            gtsam::Point3 c=gtsam::cross(w,p,dc_dw,dc_dp); // c=cross(w,p)
            gtsam::Matrix33 dcN_dr, dcN_dc;
            gtsam::Point3 cN=r.rotate(c,dcN_dr,dcN_dc); // c in nav frame (lin vel component due to rotation)
            gtsam::Matrix33 dt_dv=gtsam::Matrix33::Identity(), dt_dcN=gtsam::Matrix33::Identity();
            gtsam::Point3 t=v+cN; // t: total velocity
            if(H_x){ *H_x=dt_dcN*dcN_dr*dr_dx; }
            if(H_v){ *H_v=dt_dv; }
            if(H_w){ *H_w=dt_dcN*dcN_dc*dc_dw; }
            if(H_p){ *H_p=dt_dcN*dcN_dc*dc_dp; }
            return t;
        }

        gtsam::Vector ConstrainedJointCenterVelocityFactor::evaluateErrorNoJacCall(const gtsam::Pose3& poseA, const gtsam::Vector3& linVelA, const gtsam::Vector3& angVelA, const gtsam::Point3& sA, const gtsam::Pose3& poseB, const gtsam::Vector3& linVelB, const gtsam::Vector3& angVelB, const gtsam::Point3& sB) const {
            return evaluateError(poseA,linVelA, angVelA, sA, poseB, linVelB, angVelB, sB, boost::none, boost::none, boost::none, boost::none, boost::none, boost::none);
        }

// ------------------------------------------------------------------------------------------------------------------ //
// ------------------------------------ norm error model version of this factor ------------------------------------- //
// ------------------------------------------------------------------------------------------------------------------ //
    gtsam::Vector ConstrainedJointCenterNormVelocityFactor::evaluateError(const gtsam::Pose3& poseA, const gtsam::Vector3& linVelA, const gtsam::Vector3& angVelA, const gtsam::Point3& sA, const gtsam::Pose3& poseB, const gtsam::Vector3& linVelB, const gtsam::Vector3& angVelB, const gtsam::Point3& sB,
                                                                          boost::optional<gtsam::Matrix &> H1, // include optional jacobians H
                                                                      boost::optional<gtsam::Matrix &> H2,
                                                                          boost::optional<gtsam::Matrix &> H3,
                                                                          boost::optional<gtsam::Matrix &> H4,
                                                                          boost::optional<gtsam::Matrix &> H5,
                                                                          boost::optional<gtsam::Matrix &> H6,
                                                                          boost::optional<gtsam::Matrix &> H7,
                                                                          boost::optional<gtsam::Matrix &> H8) const {
        // ---- calculate error ----- //
        gtsam::Vector1 errorVec = errorModel(poseA,linVelA,angVelA,sA,poseB,linVelB,angVelB,sB);
        // --- handle derivatives --- //
        if(H1) {
            gtsam::Matrix16 derr_dxA;
            errorModel(poseA,linVelA,angVelA,sA,poseB,linVelB,angVelB,sB,derr_dxA,boost::none,boost::none,boost::none,boost::none,boost::none,boost::none,boost::none);
            *H1=derr_dxA;
        }
        if(H2) {
            gtsam::Matrix13 derr_dvA;
            errorModel(poseA,linVelA,angVelA,sA,poseB,linVelB,angVelB,sB,boost::none,derr_dvA,boost::none,boost::none,boost::none,boost::none,boost::none,boost::none);
            *H2=derr_dvA;
        }
        if(H3) {
            gtsam::Matrix13 derr_dwA;
            errorModel(poseA,linVelA,angVelA,sA,poseB,linVelB,angVelB,sB,boost::none,boost::none,derr_dwA,boost::none,boost::none,boost::none,boost::none,boost::none);
            *H3=derr_dwA;
        }
        if(H4) {
            gtsam::Matrix13 derr_dpA;
            errorModel(poseA,linVelA,angVelA,sA,poseB,linVelB,angVelB,sB,boost::none,boost::none,boost::none,derr_dpA,boost::none,boost::none,boost::none,boost::none);
            *H4=derr_dpA;
        }
        if(H5) {
            gtsam::Matrix16 derr_dxB;
            errorModel(poseA,linVelA,angVelA,sA,poseB,linVelB,angVelB,sB,boost::none,boost::none,boost::none,boost::none,derr_dxB,boost::none,boost::none,boost::none);
            *H5=derr_dxB;
        }
        if(H6) {
            gtsam::Matrix13 derr_dvB;
            errorModel(poseA,linVelA,angVelA,sA,poseB,linVelB,angVelB,sB,boost::none,boost::none,boost::none,boost::none,boost::none,derr_dvB,boost::none,boost::none);
            *H6=derr_dvB;
        }
        if(H7) {
            gtsam::Matrix13 derr_dwB;
            errorModel(poseA,linVelA,angVelA,sA,poseB,linVelB,angVelB,sB,boost::none,boost::none,boost::none,boost::none,boost::none,boost::none,derr_dwB,boost::none);
            *H7=derr_dwB;
        }
        if(H8) {
            gtsam::Matrix13 derr_dpB;
            errorModel(poseA,linVelA,angVelA,sA,poseB,linVelB,angVelB,sB,boost::none,boost::none,boost::none,boost::none,boost::none,boost::none,boost::none,derr_dpB);
            *H8=derr_dpB;
        }
        return errorVec;
    }

    gtsam::Vector1 ConstrainedJointCenterNormVelocityFactor::errorModel(const gtsam::Pose3& xA, const gtsam::Vector3& vA, const gtsam::Vector3& wA, const gtsam::Point3& sA, const gtsam::Pose3& xB, const gtsam::Vector3& vB, const gtsam::Vector3& wB, const gtsam::Point3& sB,
                                                                        boost::optional<gtsam::Matrix16 &> H_xA, // include optional jacobians H
                                                                    boost::optional<gtsam::Matrix13 &> H_vA,
                                                                        boost::optional<gtsam::Matrix13 &> H_wA,
                                                                        boost::optional<gtsam::Matrix13 &> H_sA,
                                                                        boost::optional<gtsam::Matrix16 &> H_xB,
                                                                        boost::optional<gtsam::Matrix13 &> H_vB,
                                                                        boost::optional<gtsam::Matrix13 &> H_wB,
                                                                        boost::optional<gtsam::Matrix13 &> H_sB){
        // this error model is equivalent to the vector error version, but with norm'd error
        // call the original vector error o, and get new derivatives from chain rule
        gtsam::Matrix36 do_dxA, do_dxB; gtsam::Matrix33 do_dwA, do_dvA, do_dsA, do_dwB, do_dvB, do_dsB;
        gtsam::Vector3 o=ConstrainedJointCenterVelocityFactor::errorModel(xA,vA,wA,sA,xB,vB,wB,sB,do_dxA,do_dvA,do_dwA,do_dsA,do_dxB,do_dvB,do_dwB,do_dsB);
        gtsam::Matrix13 derr_do;
        double err=gtsam::norm3(o,derr_do);
        // handle all the derivatives
        if(H_xA){ *H_xA=derr_do*do_dxA; }
        if(H_vA){ *H_vA=derr_do*do_dvA; }
        if(H_wA){ *H_wA=derr_do*do_dwA; }
        if(H_sA){ *H_sA=derr_do*do_dsA; }
        if(H_xB){ *H_xB=derr_do*do_dxB; }
        if(H_vB){ *H_vB=derr_do*do_dvB; }
        if(H_wB){ *H_wB=derr_do*do_dwB; }
        if(H_sB){ *H_sB=derr_do*do_dsB; }
        return gtsam::Vector1(err);
    }

    gtsam::Vector ConstrainedJointCenterNormVelocityFactor::evaluateErrorNoJacCall(const gtsam::Pose3& poseA, const gtsam::Vector3& linVelA, const gtsam::Vector3& angVelA, const gtsam::Point3& sA, const gtsam::Pose3& poseB, const gtsam::Vector3& linVelB, const gtsam::Vector3& angVelB, const gtsam::Point3& sB) const {
        return evaluateError(poseA,linVelA, angVelA, sA, poseB, linVelB, angVelB, sB, boost::none, boost::none, boost::none, boost::none, boost::none, boost::none);
    }

} // namespace bioslam