// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// A factor to encode discrepency minimization between any two segments, for example the left femur vs. right femur

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>

namespace bioslam {

    class SegmentLengthDiscrepancyFactor : public gtsam::NoiseModelFactorN<gtsam::Point3, gtsam::Point3, gtsam::Point3, gtsam::Point3> {

    public:
        SegmentLengthDiscrepancyFactor(const gtsam::Key& imuAToProximalJointKey, const gtsam::Key& imuAToDistalJointKey, const gtsam::Key& imuBToProximalJointKey, const gtsam::Key& imuBToDistalJointKey, const gtsam::SharedNoiseModel &model) :
            NoiseModelFactorN<gtsam::Point3, gtsam::Point3, gtsam::Point3, gtsam::Point3>(model, imuAToProximalJointKey, imuAToDistalJointKey, imuBToProximalJointKey, imuBToDistalJointKey) {};

        gtsam::Vector evaluateError(const gtsam::Point3& imuAToProximalJoint, const gtsam::Point3& imuAToDistalJoint, const gtsam::Point3& imuBToProximalJoint, const gtsam::Point3& imuBToDistalJoint,
                                    boost::optional<gtsam::Matrix &> H1 = boost::none, // <- also include optional derivatives
                                    boost::optional<gtsam::Matrix &> H2 = boost::none,
                                    boost::optional<gtsam::Matrix &> H3 = boost::none,
                                    boost::optional<gtsam::Matrix &> H4 = boost::none) const;

        static gtsam::Vector1 errorModel(const gtsam::Point3& vAProx, const gtsam::Point3& vADist, const gtsam::Point3& vBProx, const gtsam::Point3& vBDist,
                                    boost::optional<gtsam::Matrix13 &> H1 = boost::none, // <- also include optional derivatives
                                    boost::optional<gtsam::Matrix13 &> H2 = boost::none,
                                    boost::optional<gtsam::Matrix13 &> H3 = boost::none,
                                    boost::optional<gtsam::Matrix13 &> H4 = boost::none);
        // also need to override a second method. according to Frank Dellaert:
        // "The second is a 'clone' function that allows the factor to be copied. Under most
        // circumstances, the following code that employs the default copy constructor should
        // work fine."
        virtual gtsam::NonlinearFactor::shared_ptr clone() const {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new SegmentLengthDiscrepancyFactor(*this)));
        }
        virtual ~SegmentLengthDiscrepancyFactor() = default; // trivial deconstructor
        // shorthand for a smart pointer to a factor
        typedef boost::shared_ptr<SegmentLengthDiscrepancyFactor> shared_ptr;

    }; // class
} // namespace bioslam