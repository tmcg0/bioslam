// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// general priors on Point3 objects--lengths, differences in lengths, etc.

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>

// --- a factor which encodes a numerical maximum length on a Point3 --- //
namespace bioslam {
    class MaxPoint3MagnitudeFactor : public gtsam::NoiseModelFactorN<gtsam::Point3> {
        const double maxNorm_ = 0.5; // meters. default is 0.5m -- joint center distance shouldn't get any longer than this.
        const double P_ = 2.0000001; // the order of the polynomial error function. this is important. for example in the unit test, if P_=4, then the jacobian is too high for the optimizer to iterate (jacobian forces it to take too large of a step). must be greater than 2!
    public:
        /* Constructor */
        MaxPoint3MagnitudeFactor(gtsam::Key point3key, const double &maxNorm, const gtsam::SharedNoiseModel &model) :
                gtsam::NoiseModelFactorN<gtsam::Point3>(model, point3key), maxNorm_(maxNorm) {}

        // shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<MaxPoint3MagnitudeFactor> shared_ptr;

        // return a deep copy of this factor
        virtual NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new MaxPoint3MagnitudeFactor(*this)));
        }

        gtsam::Vector evaluateError(const gtsam::Point3 &p, boost::optional<gtsam::Matrix &> H = boost::none) const;

        virtual ~MaxPoint3MagnitudeFactor() = default; // trivial deconstructor
    };

    // constrains the difference between magnitudes of two gtsam::Point3 objects
    // assumes expected difference is zero. Could expand this factor to add a difference offset if desired.
    // noise model is a scalar representing std of this difference distribution.
    class Point3MagnitudeDifferenceFactor : public gtsam::NoiseModelFactorN<gtsam::Point3,gtsam::Point3> {
    public:
        /* Constructor */
        Point3MagnitudeDifferenceFactor(const gtsam::Key& v1Key, const gtsam::Key& v2Key, const gtsam::SharedNoiseModel &model) :
                gtsam::NoiseModelFactorN<gtsam::Point3,gtsam::Point3>(model, v1Key, v2Key){}

        // return a deep copy of this factor
        virtual NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new Point3MagnitudeDifferenceFactor(*this)));
        }

        // shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<Point3MagnitudeDifferenceFactor> shared_ptr;

        gtsam::Vector evaluateError(const gtsam::Point3& v1, const gtsam::Point3& v2,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none, // <- optional jacobians
                                    boost::optional<gtsam::Matrix &> H2 = boost::none) const;

        virtual ~Point3MagnitudeDifferenceFactor() = default; // trivial deconstructor
    };
}