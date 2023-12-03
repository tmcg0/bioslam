// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include <factors/Point3Priors.h>

namespace bioslam{
    // --- a factor which encodes a numerical maximum length on a Point3 --- //
    gtsam::Vector MaxPoint3MagnitudeFactor::evaluateError(const gtsam::Point3& p, boost::optional<gtsam::Matrix&> H) const {
        // error scalar function is e=VMax-||Vi||
        const gtsam::Vector3 v=p;
        double vnorm=v.norm();
        if(vnorm > maxNorm_){ // velocity is too high, make actual error and jacobian
            double error=0.5*pow((vnorm - maxNorm_), P_);
            gtsam::Vector1 errorVec; errorVec<<error;
            if(H) {
                gtsam::Matrix j=gtsam::Matrix::Zero(1,3);
                gtsam::Vector3 Jac= ((0.5*P_*pow((vnorm-maxNorm_),P_-1.0))/vnorm) * v;
                j(0)=Jac(0); j(1)=Jac(1); j(2)=Jac(2);
                *H=j;
            }
            return errorVec;
        }else{ // norm<maxnorm, call error and jacobian zero
            double error=0.0;
            gtsam::Vector1 errorVec; errorVec<<error;
            if(H){
                *H=gtsam::Matrix::Zero(1,3);
            }
            return errorVec;
        };
    }

    //. prior on magnitude difference between Point3's
    gtsam::Vector Point3MagnitudeDifferenceFactor::evaluateError(const gtsam::Point3 &v1, const gtsam::Point3 &v2,
                                                                 boost::optional<gtsam::Matrix &> H1,
                                                                 boost::optional<gtsam::Matrix &> H2) const {
        // error is:
        //      e = norm2(v1)-norm2(v2), e~N(0,sigma)
        //      de/dv1=1/(norm2(v1)) dot v1
        //      de/dv2=-1/(norm2(v2)) dot v2
        double error=v1.norm()-v2.norm();
        if(H1) { // de/dv1 -> should be 1x3
            *H1= 1.0/(v1.norm()) * v1.transpose();
        }
        if(H2) { // de/dv2 -> should be 1x3
            *H2= -1.0/(v2.norm()) * v2.transpose();
        }
        return gtsam::Vector1(error);
    }
}