// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include <mathutils.h>
#include "factors/ConstrainedJointCenterPositionFactor.h"

namespace bioslam {

    gtsam::Vector ConstrainedJointCenterPositionFactor::evaluateError(const gtsam::Pose3 &xA, const gtsam::Pose3 &xB, const gtsam::Point3 &L_A_to_ctr, const gtsam::Point3 &L_B_to_ctr,
                                                                      boost::optional<gtsam::Matrix &> H1, // <- also include optional derivatives
                                                        boost::optional<gtsam::Matrix &> H2,
                                                                      boost::optional<gtsam::Matrix &> H3,
                                                                      boost::optional<gtsam::Matrix &> H4) const {
        // --------- calculate error --------- //
        gtsam::Vector3 errVec = errorModel(xA,xB,L_A_to_ctr,L_B_to_ctr);
        // --- handle optional derivatives --- //
        if (H1) { //this is d_errorVec/d_Pose3_thighImu --> needs to be 3x6
            gtsam::Matrix36 derr_dxA;
            errorModel(xA,xB,L_A_to_ctr,L_B_to_ctr,derr_dxA,boost::none,boost::none,boost::none);
            *H1=derr_dxA;
        }
        if (H2) { // this is d_errorVec/d_shankPose3 --> needs to be 3x6
            gtsam::Matrix36 derr_dxB;
            errorModel(xA,xB,L_A_to_ctr,L_B_to_ctr,boost::none,derr_dxB,boost::none,boost::none);
            *H2=derr_dxB;
        }
        if (H3) { // this is d_errorVec/d_L_A_to_ctr --> needs to be 3x3
            gtsam::Matrix33 derr_dvA;
            errorModel(xA,xB,L_A_to_ctr,L_B_to_ctr,boost::none,boost::none,derr_dvA,boost::none);
            *H3=derr_dvA;
        }
        if (H4) { // this is d_errorVec/d_L_B_to_ctr --> needs to be 3x3
            gtsam::Matrix33 derr_dvB;
            errorModel(xA,xB,L_A_to_ctr,L_B_to_ctr,boost::none,boost::none,boost::none,derr_dvB);
            *H4=derr_dvB;
        }
        return errVec;
    }

    gtsam::Vector3 ConstrainedJointCenterPositionFactor::errorModel(const gtsam::Pose3 &xA, const gtsam::Pose3 &xB, const gtsam::Point3 &vA, const gtsam::Point3 &vB,
                                                                    boost::optional<gtsam::Matrix36 &> H_xA, // <- also include optional derivatives
                                     boost::optional<gtsam::Matrix36 &> H_xB,
                                                                    boost::optional<gtsam::Matrix33 &> H_vA,
                                                                    boost::optional<gtsam::Matrix33 &> H_vB){
        // ---------- implements error model of this factor ---------- //
        // this factor encodes the dynamics of two IMUs which flank a common point-center rotation joint
        // geometrically, these are two IMU poses in SE(3) which both have R(3) vectors that point to the common center
        // in the navigation frame, these vectors should point to the same point (with an error distribution on their "agreement", mean=expectedDist)
        // error = xA*vA - xB*vB ~ N(expectedDist,Sigma_3x3)
        // per GTSAM document on Pose3::transformFrom(): "takes point in Pose coordinates and transforms it to world coordinates"
        // I recently moved this function and derivatives into mathutils::ptSeparation(). Check there for more details.
        gtsam::Matrix36 derr_dxA, derr_dxB; gtsam::Matrix33 derr_dvA, derr_dvB;
        gtsam::Vector3 err=mathutils::ptSeparation(xA, vA, xB, vB, derr_dxA, derr_dvA, derr_dxB, derr_dvB);
        if(H_xA){ *H_xA=derr_dxA; }
        if(H_xB){ *H_xB=derr_dxB; }
        if(H_vA){ *H_vA=derr_dvA; }
        if(H_vB){ *H_vB=derr_dvB; }
        return err;
    }

    // ------ norm error version of factor ------ //
    gtsam::Vector ConstrainedJointCenterNormPositionFactor::evaluateError(const gtsam::Pose3 &xA, const gtsam::Pose3 &xB, const gtsam::Point3 &L_A_to_ctr, const gtsam::Point3 &L_B_to_ctr,
                                                                          boost::optional<gtsam::Matrix &> H1, // <- also include optional derivatives
                                                        boost::optional<gtsam::Matrix &> H2,
                                                                          boost::optional<gtsam::Matrix &> H3,
                                                                          boost::optional<gtsam::Matrix &> H4) const {
        // --------- calculate error --------- //
        gtsam::Vector1 errVec = errorModel(xA,xB,L_A_to_ctr,L_B_to_ctr,m_expectedDist);
        // --- handle optional derivatives --- //
        if (H1) { //this is d_errorVec/d_Pose3_thighImu --> needs to be 1x6
            gtsam::Matrix16 derr_dxA;
            errorModel(xA,xB,L_A_to_ctr,L_B_to_ctr,m_expectedDist,derr_dxA,boost::none,boost::none,boost::none);
            *H1=derr_dxA;
        }
        if (H2) { // this is d_errorVec/d_shankPose3 --> needs to be 1x6
            gtsam::Matrix16 derr_dxB;
            errorModel(xA,xB,L_A_to_ctr,L_B_to_ctr,m_expectedDist,boost::none,derr_dxB,boost::none,boost::none);
            *H2=derr_dxB;
        }
        if (H3) { // this is d_errorVec/d_L_A_to_ctr --> needs to be 1x3
            gtsam::Matrix13 derr_dvA;
            errorModel(xA,xB,L_A_to_ctr,L_B_to_ctr,m_expectedDist,boost::none,boost::none,derr_dvA,boost::none);
            *H3=derr_dvA;
        }
        if (H4) { // this is d_errorVec/d_L_B_to_ctr --> needs to be 1x3
            gtsam::Matrix13 derr_dvB;
            errorModel(xA,xB,L_A_to_ctr,L_B_to_ctr,m_expectedDist,boost::none,boost::none,boost::none,derr_dvB);
            *H4=derr_dvB;
        }
        return errVec;
    }

    gtsam::Vector1 ConstrainedJointCenterNormPositionFactor::errorModel(const gtsam::Pose3 &xA, const gtsam::Pose3 &xB, const gtsam::Point3 &vA, const gtsam::Point3 &vB, double expectedDist,
                                                                        boost::optional<gtsam::Matrix16 &> H_xA, // <- also include optional derivatives
                                                      boost::optional<gtsam::Matrix16 &> H_xB,
                                                                        boost::optional<gtsam::Matrix13 &> H_vA,
                                                                        boost::optional<gtsam::Matrix13 &> H_vB){
        // ---------- implements error model of this factor ---------- //
        // it's actually just the norm version of ImuPoseToJoinCtrFactor, so the error is trivial from chain rule.
        // call the original error (i.e., the vector separation between the points in the Nav frame) s
        gtsam::Matrix36 ds_dxA, ds_dxB; gtsam::Matrix33 ds_dvA, ds_dvB;
        gtsam::Vector3 s=ConstrainedJointCenterPositionFactor::errorModel(xA, xB, vA, vB, ds_dxA, ds_dxB, ds_dvA, ds_dvB);
        gtsam::Matrix13 dn_ds;
        double n=gtsam::norm3(s,dn_ds); // norm of measured pt separation
        gtsam::Matrix11 derr_dn; derr_dn(0,0)=1.0; // trivial derivative of below eq.
        double err=n-expectedDist; // err = measured sep. norm - expected sep norm
        if(H_xA){
            *H_xA=derr_dn*dn_ds*ds_dxA;
        }
        if(H_xB){
            *H_xB=derr_dn*dn_ds*ds_dxB;
        }
        if(H_vA){
            *H_vA=derr_dn*dn_ds*ds_dvA;
        }
        if(H_vB){
            *H_vB=derr_dn*dn_ds*ds_dvB;
        }
        return gtsam::Vector1(err);
    }

} // namespace bioslam