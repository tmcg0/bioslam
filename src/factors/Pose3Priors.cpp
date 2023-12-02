// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include <factors/Pose3Priors.h>
#include <mathutils.h>

namespace bioslam{
    gtsam::Vector Pose3TranslationPrior::evaluateError(const gtsam::Pose3& pose, boost::optional<gtsam::Matrix &> H1) const {
        // --- evaluate error --- //
        gtsam::Vector3 e=errorModel(pose,m_priorTranslation);
        if(H1){
            gtsam::Matrix36 de_dx;
            errorModel(pose,m_priorTranslation,de_dx);
            *H1=de_dx;
        }
        return e;
    }

    gtsam::Vector3 Pose3TranslationPrior::errorModel(const gtsam::Pose3& x, const gtsam::Vector3& priorTranslation, boost::optional<gtsam::Matrix36&> H_x){
        // implements prior on the translation component of a gtsam::Pose3
        gtsam::Matrix36 dt_dx;
        gtsam::Point3 t=x.translation(dt_dx);
        gtsam::Matrix33 de_dt;
        gtsam::Vector3 e=mathutils::sub(t, priorTranslation, de_dt, boost::none);
        if(H_x){
            *H_x=de_dt*dt_dx;
        }
        return e;
    }

    // --- compass prior factor --- //
    double Pose3CompassPrior::compassAngle(const gtsam::Rot3& r, const gtsam::Vector3& refVecLocal, const gtsam::Vector3& refVecNav, const gtsam::Vector3& upVecNav, boost::optional<gtsam::Matrix13&> Hr){
        // compute the compass-style angle from a Rot3 and optionally return derivatives
        // call local compass vector l
        gtsam::Matrix33 dlN_dr;
        gtsam::Vector3 lN=r.rotate(refVecLocal,dlN_dr, boost::none); // rotate l into nav frame
        // debug check: make sure compass isn't vertical. this represents an ambiguity (gimbal lock) of the compass.
        if(Pose3CompassPrior::isCompassVertical(r,refVecLocal,upVecNav)){
            std::cout<< "WARNING: compass vector is near vertical (ang=" << mathutils::unsignedAngle(lN, upVecNav) << ")" << std::endl;
        }
        gtsam::Matrix33 dlNH_dlN;
        gtsam::Vector3 lNH=mathutils::projVecIntoPlane(lN, upVecNav, dlNH_dlN, boost::none); // project lN into nav horizontal plane
        gtsam::Matrix13 dang_dlNH;
        double ang=mathutils::signedAngle(lNH, refVecNav, upVecNav, dang_dlNH, boost::none); // compass angle, radians
        if(Hr){*Hr=dang_dlNH*dlNH_dlN*dlN_dr;} // dang/dr
        return ang;
    }

    double Pose3CompassPrior::compassAngleFromVertical(const gtsam::Rot3& r, const gtsam::Vector3& refVecLocal, const gtsam::Vector3& upVecNav){
        // returns how far compass is from being vertical (radians). 0 implies compass is pointed exactly up, pi implies compass is pointed down.
        gtsam::Vector3 lN=r.rotate(refVecLocal, boost::none, boost::none); // rotate compass vector l into nav frame
        return mathutils::unsignedAngle(lN, upVecNav);
    }

    bool Pose3CompassPrior::isCompassVertical(const gtsam::Rot3& r, const gtsam::Vector3& refVecLocal, const gtsam::Vector3& upVecNav, double verticalAngTolRadians){
        // returns true/false decision on if compass is held vertically (i.e., up or down)
        double compassVertAngle=Pose3CompassPrior::compassAngleFromVertical(r,refVecLocal,upVecNav);
        return (abs(compassVertAngle)<verticalAngTolRadians || abs(compassVertAngle)>(M_PI-verticalAngTolRadians));
    }

    gtsam::Vector1 Pose3CompassPrior::errorModel(const gtsam::Pose3& pose,const gtsam::Vector3& refVecLocal, const gtsam::Vector3& refVecNav, const gtsam::Vector3& upVecNav, double expectedAng, boost::optional<gtsam::Matrix16&> Hx){
        // implements error model for this factor
        gtsam::Matrix36 dr_dx;
        gtsam::Rot3 r=pose.rotation(dr_dx);
        gtsam::Matrix13 dang_dr;
        double ang=compassAngle(r,refVecLocal,refVecNav,upVecNav,dang_dr);
        gtsam::Matrix11 derr_dang=gtsam::Matrix11::Ones(); // err=ang-expectedAng => derr/dang=1.0
        double err=ang-expectedAng;
        if(Hx){*Hx=derr_dang*dang_dr*dr_dx;} // derr/dx
        return gtsam::Vector1(err);
    }

    gtsam::Vector Pose3CompassPrior::evaluateError(const gtsam::Pose3& pose, boost::optional<gtsam::Matrix &> H1) const {
        // --- calc error --- //
        gtsam::Vector1 err=errorModel(pose,m_refVecLocal,m_refVecNav,m_upVecNav,m_expectedAng);
        // --- handle derivatives --- //
        if(H1){
            gtsam::Matrix16 derr_dx;
            errorModel(pose,m_refVecLocal,m_refVecNav,m_upVecNav,m_expectedAng,derr_dx);
            *H1=derr_dx;
        }
        return err;
    }


}