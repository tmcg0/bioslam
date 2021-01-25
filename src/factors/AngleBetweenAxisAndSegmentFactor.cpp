// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include <AngleBetweenAxisAndSegmentFactor.h>
#include <gtsam/base/numericalDerivative.h>
#include <mathutils.h>

namespace bioslam {

    gtsam::Vector AngleBetweenAxisAndSegmentFactor::evaluateError(const gtsam::Unit3 &axis, const gtsam::Point3 &v1, const gtsam::Point3 &v2,
                                boost::optional<gtsam::Matrix &> H1,
                                boost::optional<gtsam::Matrix &> H2,
                                boost::optional<gtsam::Matrix &> H3) const {
        // ------- error model ------ //
        gtsam::Vector1 err=errorModel(axis,v1,v2,m_ang);
        // --- handle derivatives --- //
        if(H1) { // derr/daxis -> should be 1x2
            gtsam::Matrix12 derr_dk;
            errorModel(axis,v1,v2,m_ang,derr_dk,boost::none,boost::none);
            *H1=derr_dk;
        }
        if(H2) { // derr/dv1 -> should be 1x3
            gtsam::Matrix13 derr_dv1;
            errorModel(axis,v1,v2,m_ang,boost::none,derr_dv1,boost::none);
            *H2=derr_dv1;
        }
        if(H3) { // derr/dv2 -> should be 1x3
            gtsam::Matrix13 derr_dv2;
            errorModel(axis,v1,v2,m_ang,boost::none,boost::none,derr_dv2);
            *H3=derr_dv2;
        }
        return err;
    }

    gtsam::Vector1 AngleBetweenAxisAndSegmentFactor::errorModel(const gtsam::Unit3& k, const gtsam::Point3& v1, const gtsam::Point3& v2, const double& expectedAngle,
                                     boost::optional<gtsam::Matrix12 &> H_k, // <- optional jacobians
                                     boost::optional<gtsam::Matrix13 &> H_v1,
                                     boost::optional<gtsam::Matrix13 &> H_v2){
        // ---------- error model ---------- //
        // The error model of this factor encodes an expected (unsigned) angle between the proximal vector (v1-v2) and the hinge axis of the segment (k)
        gtsam::Matrix12 dm_dk; gtsam::Matrix13 dm_dv1, dm_dv2;
        double m=angleBetweenAxisAndSegment(k,v1,v2,dm_dk,dm_dv1,dm_dv2);
        // err = expected ang exp minus measured angle m
        // remember the trivial derivative for subtracting two scalars: f(a,b)=a-b => df/da=1, df/db=-1
        double err=expectedAngle-m, derr_dexp=1.0, derr_dm=-1.0; // but also expectedAngle is a constant so you'll never use that.
        if(H_k){
            // derr_dk = derr_dm*dm_dkp*dkp_dk
            *H_k=derr_dm*dm_dk;
        }
        if(H_v1){
            // derr_dv1 = derr_dm*dm_dv*dv_dv1
            *H_v1=derr_dm*dm_dv1;
        }
        if(H_v2){
            // derr_dv2 = derr_dm*dm_dv*dv_dv2
            *H_v2=derr_dm*dm_dv2;
        }
        return gtsam::Vector1(err);
    }

    double AngleBetweenAxisAndSegmentFactor::angleBetweenAxisAndSegment(const gtsam::Unit3& k, const gtsam::Point3& v1, const gtsam::Point3& v2,
                                              boost::optional<gtsam::Matrix12 &> H_k, // <- optional jacobians
                                              boost::optional<gtsam::Matrix13 &> H_v1,
                                              boost::optional<gtsam::Matrix13 &> H_v2){
        // calculate the angle between axis k and segment (v1-v2), and return optional jacobians
        gtsam::Matrix33 dv_dv1, dv_dv2;
        gtsam::Point3 v=mathutils::sub(v1, v2, dv_dv1, dv_dv2); // v=v1-v2
        if(v.norm()<1.0e-10){ // poorly scaled, your segment length has gone to zero. (i.e., v1=v2)
            std::cerr<<"your segment distance is poorly scaled! norm="<<v.norm()<<std::endl;
        }
        gtsam::Matrix32 dkp_dk;
        gtsam::Point3 kp=k.unitVector(dkp_dk); // k as a point3 in R(3)
        // m is the measured angle between v and k. note that unsignedAngle() does not require you to normalize a vector beforehand.
        gtsam::Matrix13 dm_dv, dm_dkp;
        double m=mathutils::unsignedAngle(v, kp, dm_dv, dm_dkp);
        if(H_k){
            // dm_dk = dm_dkp*dkp_dk
            *H_k=dm_dkp*dkp_dk;
        }
        if(H_v1){
            // dm_dv1 = dm_dv*dv_dv1
            *H_v1=dm_dv*dv_dv1;
        }
        if(H_v2){
            // dm_dv2 = dm_dv*dv_dv2
            *H_v2=dm_dv*dv_dv2;
        }
        return m;
    }

// ------------------------------------------------------------------------------------------------------------------ //
// ---------------------- minimum allowable angle constraint version of this factor --------------------------------- //
// ------------------------------------------------------------------------------------------------------------------ //

    gtsam::Vector MinAngleBetweenAxisAndSegmentFactor::evaluateError(const gtsam::Unit3 &axis, const gtsam::Point3 &v1, const gtsam::Point3 &v2,
                                                                  boost::optional<gtsam::Matrix &> H1,
                                                                  boost::optional<gtsam::Matrix &> H2,
                                                                  boost::optional<gtsam::Matrix &> H3) const {
        // ------- error model ------ //
        gtsam::Vector1 err=errorModel(axis,v1,v2,m_minAng,a_);
        // --- handle derivatives --- //
        if(H1) { // derr/daxis -> should be 1x2
            gtsam::Matrix12 derr_dk;
            errorModel(axis,v1,v2,m_minAng,a_,derr_dk,boost::none,boost::none);
            *H1=derr_dk;
        }
        if(H2) { // derr/dv1 -> should be 1x3
            gtsam::Matrix13 derr_dv1;
            errorModel(axis,v1,v2,m_minAng,a_,boost::none,derr_dv1,boost::none);
            *H2=derr_dv1;
        }
        if(H3) { // derr/dv2 -> should be 1x3
            gtsam::Matrix13 derr_dv2;
            errorModel(axis,v1,v2,m_minAng,a_,boost::none,boost::none,derr_dv2);
            *H3=derr_dv2;
        }
        return err;
    }


    gtsam::Vector1 MinAngleBetweenAxisAndSegmentFactor::errorModel(const gtsam::Unit3& k, const gtsam::Point3& v1, const gtsam::Point3& v2, const double& minAngle, const double& a,
                                          boost::optional<gtsam::Matrix12 &> H_k, // <- optional jacobians
                                          boost::optional<gtsam::Matrix13 &> H_v1,
                                          boost::optional<gtsam::Matrix13 &> H_v2){
        // ----- implements error model for this factor ----- //
        // the error model is a simple polynomial penalty model on top of the error model already encoded in AngleBetweenAxisAndSegmentFactor
        // () get angle between axis and segment (v1-v2)
        gtsam::Matrix12 dang_dk; gtsam::Matrix13 dang_dv1, dang_dv2;
        double ang=AngleBetweenAxisAndSegmentFactor::angleBetweenAxisAndSegment(k,v1,v2,dang_dk,dang_dv1,dang_dv2);
        // () now evaluate error function
        gtsam::Matrix11 derr_dang;
        double err=mathutils::errorFunScalarMin(ang, minAngle, a, derr_dang);
        if(H_k){ *H_k=derr_dang*dang_dk; }
        if(H_v1){ *H_v1=derr_dang*dang_dv1; }
        if(H_v2){ *H_v2=derr_dang*dang_dv2; }
        return gtsam::Vector1(err);
    }

// ------------------------------------------------------------------------------------------------------------------ //
// ---------------------- maximum allowable angle constraint version of this factor --------------------------------- //
// ------------------------------------------------------------------------------------------------------------------ //

    gtsam::Vector MaxAngleBetweenAxisAndSegmentFactor::evaluateError(const gtsam::Unit3 &axis, const gtsam::Point3 &v1, const gtsam::Point3 &v2,
                                                                     boost::optional<gtsam::Matrix &> H1,
                                                                     boost::optional<gtsam::Matrix &> H2,
                                                                     boost::optional<gtsam::Matrix &> H3) const {
        // ------- error model ------ //
        gtsam::Vector1 err=errorModel(axis,v1,v2,m_maxAng,a_);
        // --- handle derivatives --- //
        if(H1) { // derr/daxis -> should be 1x2
            gtsam::Matrix12 derr_dk;
            errorModel(axis,v1,v2,m_maxAng,a_,derr_dk,boost::none,boost::none);
            *H1=derr_dk;
        }
        if(H2) { // derr/dv1 -> should be 1x3
            gtsam::Matrix13 derr_dv1;
            errorModel(axis,v1,v2,m_maxAng,a_,boost::none,derr_dv1,boost::none);
            *H2=derr_dv1;
        }
        if(H3) { // derr/dv2 -> should be 1x3
            gtsam::Matrix13 derr_dv2;
            errorModel(axis,v1,v2,m_maxAng,a_,boost::none,boost::none,derr_dv2);
            *H3=derr_dv2;
        }
        return err;
    }

    gtsam::Vector1 MaxAngleBetweenAxisAndSegmentFactor::errorModel(const gtsam::Unit3& k, const gtsam::Point3& v1, const gtsam::Point3& v2, const double& maxAngle, const double& a,
                                                                   boost::optional<gtsam::Matrix12 &> H_k, // <- optional jacobians
                                                                   boost::optional<gtsam::Matrix13 &> H_v1,
                                                                   boost::optional<gtsam::Matrix13 &> H_v2){
        // ----- implements error model for this factor ----- //
        // the error model is a simple polynomial penalty model on top of the error model already encoded in AngleBetweenAxisAndSegmentFactor
        // () get angle between axis and segment (v1-v2)
        gtsam::Matrix12 dang_dk; gtsam::Matrix13 dang_dv1, dang_dv2;
        double ang=AngleBetweenAxisAndSegmentFactor::angleBetweenAxisAndSegment(k,v1,v2,dang_dk,dang_dv1,dang_dv2);
        // () now evaluate error function
        gtsam::Matrix11 derr_dang;
        double err=mathutils::errorFunScalarMax(ang, maxAngle, a, derr_dang);
        if(H_k){ *H_k=derr_dang*dang_dk; }
        if(H_v1){ *H_v1=derr_dang*dang_dv1; }
        if(H_v2){ *H_v2=derr_dang*dang_dv2; }
        return gtsam::Vector1(err);
    }

} // namespace bioslam
