// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include <SegmentLengthDiscrepancyFactor.h>
#include <mathutils.h>

namespace bioslam {

        gtsam::Vector SegmentLengthDiscrepancyFactor::evaluateError(const gtsam::Point3& imuAToProximalJoint, const gtsam::Point3& imuAToDistalJoint, const gtsam::Point3& imuBToProximalJoint, const gtsam::Point3& imuBToDistalJoint,
                                    boost::optional<gtsam::Matrix &> H1, // <- also include optional derivatives
                                    boost::optional<gtsam::Matrix &> H2,
                                    boost::optional<gtsam::Matrix &> H3,
                                    boost::optional<gtsam::Matrix &> H4) const {
        // ----- calculate error ----- //
        gtsam::Vector1 errorVec=errorModel(imuAToProximalJoint,imuAToDistalJoint,imuBToProximalJoint,imuBToDistalJoint);
        // --- handle derivatives --- //
        if(H1) {
            gtsam::Matrix13 derr_dvAProx;
            errorModel(imuAToProximalJoint,imuAToDistalJoint,imuBToProximalJoint,imuBToDistalJoint,derr_dvAProx,boost::none,boost::none,boost::none);
            *H1=derr_dvAProx;
        }
        if(H2) {
            gtsam::Matrix13 derr_dvADist;
            errorModel(imuAToProximalJoint,imuAToDistalJoint,imuBToProximalJoint,imuBToDistalJoint,boost::none,derr_dvADist,boost::none,boost::none);
            *H2=derr_dvADist;
        }
        if(H3) {
            gtsam::Matrix13 derr_dvBProx;
            errorModel(imuAToProximalJoint,imuAToDistalJoint,imuBToProximalJoint,imuBToDistalJoint,boost::none,boost::none,derr_dvBProx,boost::none);
            *H3=derr_dvBProx;
        }
        if(H4) {
            gtsam::Matrix13 derr_dvBDist;
            errorModel(imuAToProximalJoint,imuAToDistalJoint,imuBToProximalJoint,imuBToDistalJoint,boost::none,boost::none,boost::none,derr_dvBDist);
            *H4=derr_dvBDist;
        }
        return errorVec;
        }

        gtsam::Vector1 SegmentLengthDiscrepancyFactor::errorModel(const gtsam::Point3& vAProx, const gtsam::Point3& vADist, const gtsam::Point3& vBProx, const gtsam::Point3& vBDist,
                                         boost::optional<gtsam::Matrix13 &> H_vAProx, // <- also include optional derivatives
                                         boost::optional<gtsam::Matrix13 &> H_vADist,
                                         boost::optional<gtsam::Matrix13 &> H_vBProx,
                                         boost::optional<gtsam::Matrix13 &> H_vBDist){
            // --- implements error model for this factor --- //
            // error is very simple: error = norm(vAProx-vADist) - norm(vBProx-vBDist)
            gtsam::Matrix33 dvA_dvAProx, dvA_dvADist, dvB_dvBProx, dvB_dvBDist;
            gtsam::Vector3 vA=mathutils::sub(vAProx, vADist, dvA_dvAProx, dvA_dvADist);
            gtsam::Vector3 vB=mathutils::sub(vBProx, vBDist, dvB_dvBProx, dvB_dvBDist);
            gtsam::Matrix13 dnvA_dvA, dnvB_dvB; // deriv of norms w.r.t. vectors
            double nvA=gtsam::norm3(vA,dnvA_dvA);
            double nvB=gtsam::norm3(vB,dnvB_dvB);
            // err = normA - normB
            // remember the trivial derivative for subtracting two scalars: f(a,b)=a-b => df/da=1, df/db=-1
            double err=nvA-nvB, derr_dnvA=1.0, derr_dnvB=-1.0;
            if(H_vAProx){ // derr_dvAProx=derr_dnvA*dnvA_dvA*dvA_dvAProx
                *H_vAProx=derr_dnvA*dnvA_dvA*dvA_dvAProx;
            }
            if(H_vADist){ // derr_dvADist=derr_dnvA*dnvA_dvA*dvA_dvADist
                *H_vADist=derr_dnvA*dnvA_dvA*dvA_dvADist;
            }
            if(H_vBProx){ // derr_dvBProx=derr_dnvB*dnvB_dvB*dvB_dvBProx
                *H_vBProx=derr_dnvB*dnvB_dvB*dvB_dvBProx;
            }
            if(H_vBDist){ // derr_dvBDist=derr_dnvB*dnvB_dvB*dvB_dvBDist
                *H_vBDist=derr_dnvB*dnvB_dvB*dvB_dvBDist;
            }
            return gtsam::Vector1(err);
        }
} // namespace bioslam