// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include <factors/SegmentLengthMagnitudeFactor.h>
#include <mathutils.h>

namespace bioslam {

        gtsam::Vector SegmentLengthMagnitudeFactor::evaluateError(const gtsam::Point3 &v1, const gtsam::Point3 &v2,
                                    boost::optional<gtsam::Matrix &> H1, // <- also include optional derivatives
                                    boost::optional<gtsam::Matrix &> H2) const {
            // ----- calculate error ----- //
            gtsam::Vector1 errorVec=errorModel(v1,v2,m_idealSegmentLength);
            // --- handle derivatives --- //
            if(H1) { // dErr/dv1 (1x3)
                gtsam::Matrix13 derr_dv1;
                errorModel(v1,v2,m_idealSegmentLength,derr_dv1,boost::none);
                *H1=derr_dv1;
            }
            if(H2) { // dErr/dv2 (1x3)
                gtsam::Matrix13 derr_dv2;
                errorModel(v1,v2,m_idealSegmentLength,boost::none,derr_dv2);
                *H2=derr_dv2;
            }
            return errorVec;
        }

        gtsam::Vector1 SegmentLengthMagnitudeFactor::errorModel(const gtsam::Point3 &v1, const gtsam::Point3 &v2, double expectedLength,
                                             boost::optional<gtsam::Matrix13 &> H_v1, // <- also include optional derivatives
                                             boost::optional<gtsam::Matrix13 &> H_v2){
            // ---- implements error model for this factor ---- //
            // error is very trivially: error = (v1-v2).norm() - expectedLength;
            gtsam::Matrix13 dn_dv1, dn_dv2;
            double n=segmentLength(v1,v2,dn_dv1,dn_dv2);
            // err = n - expectedLength
            // remember the trivial derivative for subtracting two scalars: f(a,b)=a-b => df/da=1, df/db=-1
            double err=n-expectedLength, derr_dn=1.0;
            if(H_v1){ // derr_dv1=derr_dn*dn_dv*dv_dv1;
                *H_v1=derr_dn*dn_dv1;
            }
            if(H_v2){ // derr_dv2=derr_dn*dn_dv*dv_dv2;
                *H_v2=derr_dn*dn_dv2;
            }
            return gtsam::Vector1(err);
        }

        double SegmentLengthMagnitudeFactor::segmentLength(const gtsam::Point3 &v1, const gtsam::Point3 &v2,
                                        boost::optional<gtsam::Matrix13 &> H_v1, // <- also include optional derivatives
                                        boost::optional<gtsam::Matrix13 &> H_v2){
            // calculate the segment length as (v1-v2).norm() and return optional jacobians
            gtsam::Matrix33 dv_dv1, dv_dv2;
            gtsam::Point3 v=mathutils::sub(v1, v2, dv_dv1, dv_dv2);
            gtsam::Matrix13 dn_dv;
            double n=gtsam::norm3(v,dn_dv);
            if(H_v1){ // dn_dv1=dn_dv*dv_dv1
                *H_v1=dn_dv*dv_dv1;
            }
            if(H_v2){ // dn_dv2=dn_dv*dv_dv2
                *H_v2=dn_dv*dv_dv2;
            }
            return n;
        }

    // -------------------------------------------------------------------------------------------------------------- //
    // -------------------------------- maximum segment length version of the factor -------------------------------- //
    // -------------------------------------------------------------------------------------------------------------- //

    gtsam::Vector SegmentLengthMaxMagnitudeFactor::evaluateError(const gtsam::Point3 &v1, const gtsam::Point3 &v2,
                                                                 boost::optional<gtsam::Matrix &> H1, // <- also include optional derivatives
                                                                 boost::optional<gtsam::Matrix &> H2) const {
        // ----- calculate error ----- //
        gtsam::Vector1 errorVec=errorModel(v1,v2,m_maxSegmentLength,a_);
        // --- handle derivatives --- //
        if(H1){
            gtsam::Matrix13 derr_dv1;
            errorModel(v1,v2,m_maxSegmentLength,a_,derr_dv1,boost::none);
            *H1=derr_dv1;
        }
        if(H2){
            gtsam::Matrix13 derr_dv2;
            errorModel(v1,v2,m_maxSegmentLength,a_,boost::none,derr_dv2);
            *H2=derr_dv2;
        }
        return errorVec;
    }

    gtsam::Vector1 SegmentLengthMaxMagnitudeFactor::errorModel(const gtsam::Point3 &v1, const gtsam::Point3 &v2, double maxSegmentLength, double a,
                                                               boost::optional<gtsam::Matrix13 &> H_v1, // <- also include optional derivatives
                                                               boost::optional<gtsam::Matrix13 &> H_v2){
        // ----- implements error model for this factor ----- //
        // the error model is a simple polynomial penalty model on top of the error model already encoded in SegmentLengthMagntitudeFactor
        // () get segment length
        gtsam::Matrix13 dn_dv1, dn_dv2;
        double n=SegmentLengthMagnitudeFactor::segmentLength(v1,v2,dn_dv1,dn_dv2);
        // () compute error
        gtsam::Matrix11 derr_dn;
        double err=mathutils::errorFunScalarMax(n, maxSegmentLength, a, derr_dn);
        if(H_v1){ *H_v1=derr_dn*dn_dv1; }
        if(H_v2){ *H_v2=derr_dn*dn_dv2; }
        return gtsam::Vector1(err);

    }

    // -------------------------------------------------------------------------------------------------------------- //
    // -------------------------------- minimum segment length version of the factor -------------------------------- //
    // -------------------------------------------------------------------------------------------------------------- //

    gtsam::Vector SegmentLengthMinMagnitudeFactor::evaluateError(const gtsam::Point3 &v1, const gtsam::Point3 &v2,
                                                                 boost::optional<gtsam::Matrix &> H1, // <- also include optional derivatives
                                                                 boost::optional<gtsam::Matrix &> H2) const {
        // ----- calculate error ----- //
        gtsam::Vector1 errorVec=errorModel(v1,v2,m_minSegmentLength,a_);
        // --- handle derivatives --- //
        if(H1){
            gtsam::Matrix13 derr_dv1;
            errorModel(v1,v2,m_minSegmentLength,a_,derr_dv1,boost::none);
            *H1=derr_dv1;
        }
        if(H2){
            gtsam::Matrix13 derr_dv2;
            errorModel(v1,v2,m_minSegmentLength,a_,boost::none,derr_dv2);
            *H2=derr_dv2;
        }
        return errorVec;
    }

    gtsam::Vector1 SegmentLengthMinMagnitudeFactor::errorModel(const gtsam::Point3 &v1, const gtsam::Point3 &v2, double minSegmentLength, double a,
                                                               boost::optional<gtsam::Matrix13 &> H_v1, // <- also include optional derivatives
                                                               boost::optional<gtsam::Matrix13 &> H_v2){
        // ----- implements error model for this factor ----- //
        // the error model is a simple polynomial penalty model on top of the error model already encoded in SegmentLengthMagntitudeFactor
        // () get segment length
        gtsam::Matrix13 dn_dv1, dn_dv2;
        double n=SegmentLengthMagnitudeFactor::segmentLength(v1,v2,dn_dv1,dn_dv2);
        // () compute error
        gtsam::Matrix11 derr_dn;
        double err=mathutils::errorFunScalarMin(n, minSegmentLength, a, derr_dn);
        if(H_v1){ *H_v1=derr_dn*dn_dv1; }
        if(H_v2){ *H_v2=derr_dn*dn_dv2; }
        return gtsam::Vector1(err);
    }

} // namespace bioslam