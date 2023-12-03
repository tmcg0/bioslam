// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// A factor to constrain the length of a limb segment from anthropometric data, given a single IMU pose and two static vector offsets to its neighboring joint centers

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace bioslam {

    class SegmentLengthMagnitudeFactor : public gtsam::NoiseModelFactorN<gtsam::Point3, gtsam::Point3> {

    public:
        // member variables for gyro measurements
        double m_idealSegmentLength;

        SegmentLengthMagnitudeFactor(const gtsam::Key &v1, const gtsam::Key &v2, double segmentLengthMean, const gtsam::SharedNoiseModel &model) :
                NoiseModelFactorN<gtsam::Point3, gtsam::Point3>(model, v1, v2),
                m_idealSegmentLength(std::move(segmentLengthMean)) {};

        gtsam::Vector evaluateError(const gtsam::Point3 &v1, const gtsam::Point3 &v2,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none, // <- also include optional derivatives
                                    boost::optional<gtsam::Matrix &> H2 = boost::none) const;

        static gtsam::Vector1 errorModel(const gtsam::Point3 &v1, const gtsam::Point3 &v2, double expectedLength,
                                    boost::optional<gtsam::Matrix13 &> H_v1 = boost::none, // <- also include optional derivatives
                                    boost::optional<gtsam::Matrix13 &> H_v2 = boost::none);

        static double segmentLength(const gtsam::Point3 &v1, const gtsam::Point3 &v2,
                                    boost::optional<gtsam::Matrix13 &> H_v1 = boost::none, // <- also include optional derivatives
                                    boost::optional<gtsam::Matrix13 &> H_v2 = boost::none);

        // also need to override a second method. according to Frank Dellaert:
        // "The second is a 'clone' function that allows the factor to be copied. Under most
        // circumstances, the following code that employs the default copy constructor should
        // work fine."
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new SegmentLengthMagnitudeFactor(*this)));
        }
        virtual ~SegmentLengthMagnitudeFactor() {} // trivial deconstructor
        // shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<SegmentLengthMagnitudeFactor> shared_ptr;

    }; // class

    // -------------------------------------------------------------------------------------------------------------- //
    // -------------------------------- maximum segment length version of the factor -------------------------------- //
    // -------------------------------------------------------------------------------------------------------------- //

    class SegmentLengthMaxMagnitudeFactor : public gtsam::NoiseModelFactorN<gtsam::Point3, gtsam::Point3> {

    public:
        const double m_maxSegmentLength;
        const double a_ = 4.000000000000001; // the order of the polynomial error function


        SegmentLengthMaxMagnitudeFactor(const gtsam::Key &v1, const gtsam::Key &v2, double segmentLengthMax, const gtsam::SharedNoiseModel &model) :
                NoiseModelFactorN<gtsam::Point3, gtsam::Point3>(model, v1, v2),
                m_maxSegmentLength(segmentLengthMax) {};

        gtsam::Vector evaluateError(const gtsam::Point3 &v1, const gtsam::Point3 &v2,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none, // <- also include optional derivatives
                                    boost::optional<gtsam::Matrix &> H2 = boost::none) const;

        static gtsam::Vector1 errorModel(const gtsam::Point3 &v1, const gtsam::Point3 &v2, double maxSegmentLength, double a,
                                         boost::optional<gtsam::Matrix13 &> H_v1 = boost::none, // <- also include optional derivatives
                                         boost::optional<gtsam::Matrix13 &> H_v2 = boost::none);

        // also need to override a second method. according to Frank Dellaert:
        // "The second is a 'clone' function that allows the factor to be copied. Under most
        // circumstances, the following code that employs the default copy constructor should
        // work fine."
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new SegmentLengthMaxMagnitudeFactor(*this)));
        }
        // shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<SegmentLengthMaxMagnitudeFactor> shared_ptr;
        virtual ~SegmentLengthMaxMagnitudeFactor() {} // trivial deconstructor

    }; // class

    // -------------------------------------------------------------------------------------------------------------- //
    // -------------------------------- minimum segment length version of the factor -------------------------------- //
    // -------------------------------------------------------------------------------------------------------------- //

    class SegmentLengthMinMagnitudeFactor : public gtsam::NoiseModelFactorN<gtsam::Point3, gtsam::Point3> {

    public:
        const double m_minSegmentLength;
        const double a_ = 4.000000000000001; // the order of the polynomial error function

        SegmentLengthMinMagnitudeFactor(const gtsam::Key &v1, const gtsam::Key &v2, double segmentLengthMin, const gtsam::SharedNoiseModel &model) :
                NoiseModelFactorN<gtsam::Point3, gtsam::Point3>(model, v1, v2),
                m_minSegmentLength(segmentLengthMin) {};

        gtsam::Vector evaluateError(const gtsam::Point3 &v1, const gtsam::Point3 &v2,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none, // <- also include optional derivatives
                                    boost::optional<gtsam::Matrix &> H2 = boost::none) const;

        static gtsam::Vector1 errorModel(const gtsam::Point3 &v1, const gtsam::Point3 &v2, double minSegmentLength, double a,
                                         boost::optional<gtsam::Matrix13 &> H_v1 = boost::none, // <- also include optional derivatives
                                         boost::optional<gtsam::Matrix13 &> H_v2 = boost::none);

        // also need to override a second method. according to Frank Dellaert:
        // "The second is a 'clone' function that allows the factor to be copied. Under most
        // circumstances, the following code that employs the default copy constructor should
        // work fine."
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new SegmentLengthMinMagnitudeFactor(*this)));
        }
        // shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<SegmentLengthMinMagnitudeFactor> shared_ptr;
        virtual ~SegmentLengthMinMagnitudeFactor() {} // trivial deconstructor
    }; // class
} // namespace bioslam