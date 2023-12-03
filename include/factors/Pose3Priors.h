// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// general priors on gtsam::Pose3

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace bioslam {

    // -------------------------------------------------------------------------------------------------------------- //
    // class to put a simple translation prior on a gtsam::Pose3
    // -------------------------------------------------------------------------------------------------------------- //

    class Pose3TranslationPrior : public gtsam::NoiseModelFactorN<gtsam::Pose3> {

    public:
        gtsam::Vector3 m_priorTranslation;
        // shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<Pose3TranslationPrior> shared_ptr;

        Pose3TranslationPrior(const gtsam::Key &poseKey, const gtsam::Vector3& priorTranslation, const gtsam::SharedNoiseModel &model) :
                NoiseModelFactorN<gtsam::Pose3>(model, poseKey), m_priorTranslation(priorTranslation) {};

        static gtsam::Vector3 errorModel(const gtsam::Pose3& pose, const gtsam::Vector3& priorTranslation, boost::optional<gtsam::Matrix36&> H_1 = boost::none);

        gtsam::Vector evaluateError(const gtsam::Pose3& pose, boost::optional<gtsam::Matrix &> H1 = boost::none) const;

        // also need to override a second method. according to Frank Dellaert:
        // "The second is a 'clone' function that allows the factor to be copied. Under most
        // circumstances, the following code that employs the default copy constructor should
        // work fine."
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new Pose3TranslationPrior(*this)));
        }

        virtual ~Pose3TranslationPrior() = default; // trivial deconstructor
    }; // class

    // -------------------------------------------------------------------------------------------------------------- //
    // class to put a compass-style prior on a gtsam::Pose3
    //     i.e., give local reference vector in body frame you want to project into nav frame horizontal plane (refVecLocal)
    //           give nav frame upward vector (upVecNav)
    //           give reference vector in nav frame (refVecNav)--this represents where the compass "zero" angle is
    //     refVecNav and upVecNav should be orthogonal
    // -------------------------------------------------------------------------------------------------------------- //

    class Pose3CompassPrior : public gtsam::NoiseModelFactorN<gtsam::Pose3> {

    public:
        gtsam::Vector3 m_refVecLocal;
        gtsam::Vector3 m_refVecNav;
        gtsam::Vector3 m_upVecNav;
        double m_expectedAng;

        // shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<Pose3CompassPrior> shared_ptr;

        Pose3CompassPrior(const gtsam::Key &poseKey, const gtsam::Vector3& refVecLocal, const gtsam::Vector3& refVecNav, const gtsam::Vector3& upVecNav, double expectedAng, const gtsam::SharedNoiseModel &model) :
                NoiseModelFactorN<gtsam::Pose3>(model, poseKey), m_refVecLocal(refVecLocal), m_refVecNav(refVecNav), m_upVecNav(upVecNav), m_expectedAng(expectedAng) {
            if(expectedAng>M_PI || expectedAng<-M_PI){
                throw std::runtime_error("Pose3CompassPrior: prior angle "+std::to_string(expectedAng)+" out of bounds. angle should be in [-pi,pi]");
            }
        };

        static gtsam::Vector1 errorModel(const gtsam::Pose3& pose,const gtsam::Vector3& refVecLocal, const gtsam::Vector3& refVecNav, const gtsam::Vector3& upVecNav, double expectedAng, boost::optional<gtsam::Matrix16&> Hx = boost::none);
        static double compassAngle(const gtsam::Rot3& r, const gtsam::Vector3& refVecLocal, const gtsam::Vector3& refVecNav, const gtsam::Vector3& upVecNav, boost::optional<gtsam::Matrix13&> Hr = boost::none);
        static double compassAngleFromVertical(const gtsam::Rot3& r, const gtsam::Vector3& refVecLocal, const gtsam::Vector3& upVecNav); // returns how far compass is from being vertical (radians)
        static bool isCompassVertical(const gtsam::Rot3& r, const gtsam::Vector3& refVecLocal, const gtsam::Vector3& upVecNav, double verticalAngTolRadians=10.0*M_PI/180.0); // returns true/false decision on if compass is held vertically

        gtsam::Vector evaluateError(const gtsam::Pose3& pose, boost::optional<gtsam::Matrix &> H1 = boost::none) const;

        // also need to override a second method. according to Frank Dellaert:
        // "The second is a 'clone' function that allows the factor to be copied. Under most
        // circumstances, the following code that employs the default copy constructor should
        // work fine."
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new Pose3CompassPrior(*this)));
        }

        virtual ~Pose3CompassPrior() = default; // trivial deconstructor
    }; // class


} // namespace bioslam