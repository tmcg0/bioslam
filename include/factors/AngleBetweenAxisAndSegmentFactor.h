// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// a factor which encodes the angle between a gtsam::Unit3 axis and a segment spanned by gtsam::Point3 v1 and v2

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/geometry/Point3.h>

namespace bioslam {
    class AngleBetweenAxisAndSegmentFactor : public gtsam::NoiseModelFactorN<gtsam::Unit3,gtsam::Point3,gtsam::Point3> {
        const double m_ang = M_PI/2; // radians, must be between 0 and pi
    public:
        /* Constructor */
        AngleBetweenAxisAndSegmentFactor(const gtsam::Key& axisKey, const gtsam::Key& v1Key, const gtsam::Key& v2Key, const double &ang, const gtsam::SharedNoiseModel &model) :
                gtsam::NoiseModelFactorN<gtsam::Unit3,gtsam::Point3,gtsam::Point3>(model, axisKey, v1Key, v2Key), m_ang(ang){
              // assert that the input angle is within bounds
              if(m_ang<0.0){ // must be at least 0 radians
                  throw std::runtime_error("input angle limit must was less than zero.");
              }
              if(m_ang>M_PI){  // must be at most pi radians
                  throw std::runtime_error("input angle limit was greater than pi");
              }
        };

        // return a deep copy of this factor
        virtual NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new AngleBetweenAxisAndSegmentFactor(*this)));
        }

        gtsam::Vector evaluateError(const gtsam::Unit3& axis, const gtsam::Point3& v1, const gtsam::Point3& v2,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none, // <- optional jacobians
                                    boost::optional<gtsam::Matrix &> H2 = boost::none,
                                    boost::optional<gtsam::Matrix &> H3 = boost::none) const;

        static gtsam::Vector1 errorModel(const gtsam::Unit3& k, const gtsam::Point3& v1, const gtsam::Point3& v2, const double& expectedAngle,
                                         boost::optional<gtsam::Matrix12 &> H_k = boost::none, // <- optional jacobians
                                         boost::optional<gtsam::Matrix13 &> H_v1 = boost::none,
                                         boost::optional<gtsam::Matrix13 &> H_v2 = boost::none);

        // calculate unsigned angle between axis k and segment (v1-v2)
        static double angleBetweenAxisAndSegment(const gtsam::Unit3& k, const gtsam::Point3& v1, const gtsam::Point3& v2,
                                         boost::optional<gtsam::Matrix12 &> H_k = boost::none, // <- optional jacobians
                                         boost::optional<gtsam::Matrix13 &> H_v1 = boost::none,
                                         boost::optional<gtsam::Matrix13 &> H_v2 = boost::none);

        virtual ~AngleBetweenAxisAndSegmentFactor() = default; // trivial deconstructor

        // shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<AngleBetweenAxisAndSegmentFactor> shared_ptr;
    };


// ------------------------------------------------------------------------------------------------------------------ //
// ---------------------- minimum allowable angle constraint version of this factor --------------------------------- //
// ------------------------------------------------------------------------------------------------------------------ //

    class MinAngleBetweenAxisAndSegmentFactor : public gtsam::NoiseModelFactorN<gtsam::Unit3,gtsam::Point3,gtsam::Point3> {
        const double m_minAng = 0.0001; // radians, must be between 0 and pi
        double a_ = 10.000000000001;
    public:
        /* Constructor */
        MinAngleBetweenAxisAndSegmentFactor(const gtsam::Key& axisKey, const gtsam::Key& v1Key, const gtsam::Key& v2Key, const double &minang, const gtsam::SharedNoiseModel &model) :
                gtsam::NoiseModelFactorN<gtsam::Unit3,gtsam::Point3,gtsam::Point3>(model, axisKey, v1Key, v2Key), m_minAng(minang){
            // assert that the input angle is within bounds
            if(m_minAng<0.0){ // must be at least 0 radians
                throw std::runtime_error("input angle limit must was less than zero.");
            }
            if(m_minAng>M_PI){  // must be at most pi radians
                throw std::runtime_error("input angle limit was greater than pi");
            }
        };

        // return a deep copy of this factor
        virtual NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new MinAngleBetweenAxisAndSegmentFactor(*this)));
        }

        gtsam::Vector evaluateError(const gtsam::Unit3& axis, const gtsam::Point3& v1, const gtsam::Point3& v2,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none, // <- optional jacobians
                                    boost::optional<gtsam::Matrix &> H2 = boost::none,
                                    boost::optional<gtsam::Matrix &> H3 = boost::none) const;

        static gtsam::Vector1 errorModel(const gtsam::Unit3& k, const gtsam::Point3& v1, const gtsam::Point3& v2, const double& minAngle, const double& a,
                                         boost::optional<gtsam::Matrix12 &> H_k = boost::none, // <- optional jacobians
                                         boost::optional<gtsam::Matrix13 &> H_v1 = boost::none,
                                         boost::optional<gtsam::Matrix13 &> H_v2 = boost::none);

        virtual ~MinAngleBetweenAxisAndSegmentFactor() = default; // trivial deconstructor

        // shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<MinAngleBetweenAxisAndSegmentFactor> shared_ptr;
    };

// ------------------------------------------------------------------------------------------------------------------ //
// ---------------------- maximum allowable angle constraint version of this factor --------------------------------- //
// ------------------------------------------------------------------------------------------------------------------ //

    class MaxAngleBetweenAxisAndSegmentFactor : public gtsam::NoiseModelFactorN<gtsam::Unit3,gtsam::Point3,gtsam::Point3> {
        const double m_maxAng = M_PI*0.9999; // radians, must be between 0 and pi
        double a_ = 10.000000000001;
    public:
        /* Constructor */
        MaxAngleBetweenAxisAndSegmentFactor(const gtsam::Key& axisKey, const gtsam::Key& v1Key, const gtsam::Key& v2Key, const double &maxang, const gtsam::SharedNoiseModel &model) :
                gtsam::NoiseModelFactorN<gtsam::Unit3,gtsam::Point3,gtsam::Point3>(model, axisKey, v1Key, v2Key), m_maxAng(maxang){
            // assert that the input angle is within bounds
            if(m_maxAng<0.0){ // must be at least 0 radians
                throw std::runtime_error("input angle limit must was less than zero.");
            }
            if(m_maxAng>M_PI){  // must be at most pi radians
                throw std::runtime_error("input angle limit was greater than pi");
            }
        };

        // return a deep copy of this factor
        virtual NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new MaxAngleBetweenAxisAndSegmentFactor(*this)));
        }

        gtsam::Vector evaluateError(const gtsam::Unit3& axis, const gtsam::Point3& v1, const gtsam::Point3& v2,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none, // <- optional jacobians
                                    boost::optional<gtsam::Matrix &> H2 = boost::none,
                                    boost::optional<gtsam::Matrix &> H3 = boost::none) const;

        static gtsam::Vector1 errorModel(const gtsam::Unit3& k, const gtsam::Point3& v1, const gtsam::Point3& v2, const double& maxAngle, const double& a,
                                         boost::optional<gtsam::Matrix12 &> H_k = boost::none, // <- optional jacobians
                                         boost::optional<gtsam::Matrix13 &> H_v1 = boost::none,
                                         boost::optional<gtsam::Matrix13 &> H_v2 = boost::none);

        virtual ~MaxAngleBetweenAxisAndSegmentFactor() = default; // trivial deconstructor

        // shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<MaxAngleBetweenAxisAndSegmentFactor> shared_ptr;
    };

} // namespace bioslam