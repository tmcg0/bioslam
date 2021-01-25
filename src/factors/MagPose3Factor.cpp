// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include <mathutils.h>
#include "factors/MagPose3Factor.h"

namespace bioslam {

    gtsam::Vector MagPose3Factor::evaluateError(const gtsam::Pose3 &x, boost::optional<gtsam::Matrix &> H) const {
        // ----- evaluate error ----- //
        gtsam::Vector3 errorVec=errorModel(x,m_measurement,B_Global);
        // --- handle derivatives --- //
        if (H) {
            gtsam::Matrix36 derr_dx;
            errorModel(x,m_measurement,B_Global,derr_dx);
            *H=derr_dx;
        }
        return errorVec;
    }

    gtsam::Vector3 MagPose3Factor::errorModel(const gtsam::Pose3& x, const gtsam::Vector3& meas, const gtsam::Vector3& bN, boost::optional<gtsam::Matrix36&> H_x){
        // ---------- error model ---------- //
        // you have a pose x (with R[B->N]) and a local measurement meas and a belief about what that measurement should be when rotated into the nav frame, bN
        // error = R[B->N]*measB - bN
        gtsam::Matrix36 dr_dx;
        gtsam::Rot3 r=x.rotation(dr_dx); // R[B->N]
        gtsam::Matrix33 dmN_dr;
        gtsam::Vector3 mN=r.rotate(meas,dmN_dr,boost::none); // meas in nav frame
        gtsam::Matrix33 derr_dmN;
        gtsam::Vector3 err=mathutils::sub(mN, bN, derr_dmN, boost::none);
        if(H_x){ // derr_dx=derr_dmN*dmN_dr*dr_dx;
            *H_x=derr_dmN*dmN_dr*dr_dx;
        }
        return err;
    }
} // namespace bioslam