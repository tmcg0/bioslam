// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// convenient utilities to expand GTSAM type functionality

#pragma once

#include <vector>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include "imu/imu.h"
#include <gtsam/base/numericalDerivative.h>

namespace gtsamutils{
    Eigen::MatrixXd Point3VectorToEigenMatrix(const std::vector<gtsam::Point3>& p);
    Eigen::MatrixXd Vector3VectorToEigenMatrix(const std::vector<gtsam::Vector3>& v);
    std::vector<gtsam::Rot3> Pose3VectorToRot3Vector(const std::vector<gtsam::Pose3>& poses);
    std::vector<gtsam::Point3> Pose3VectorToPoint3Vector(const std::vector<gtsam::Pose3>& poses);
    std::vector<double> vectorizePoint3x(std::vector<gtsam::Point3>);
    std::vector<double> vectorizePoint3y(std::vector<gtsam::Point3>);
    std::vector<double> vectorizePoint3z(std::vector<gtsam::Point3>);
    std::vector<double> vectorizeVector3X(std::vector<gtsam::Vector3>);
    std::vector<double> vectorizeVector3Y(std::vector<gtsam::Vector3>);
    std::vector<double> vectorizeVector3Z(std::vector<gtsam::Vector3>);
    std::vector<double> vectorizeQuaternionS(std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>>);
    std::vector<double> vectorizeQuaternionX(std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>>);
    std::vector<double> vectorizeQuaternionY(std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>>);
    std::vector<double> vectorizeQuaternionZ(std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>>);
    std::vector<double> vectorSetMagnitudes(const std::vector<Eigen::Vector3d>& v);
    Eigen::MatrixXd vectorRot3ToYprMatrix(const std::vector<gtsam::Rot3>& R);
    std::vector<gtsam::Rot3> imuOrientation(const imu& myImu); // pull out stored quaternion as a vector<Rot3>
    gtsam::Vector3 accel_Vector3(const imu& myImu, const int& idx);
    gtsam::Vector3 mags_Vector3(const imu& myImu, const int& idx);
    gtsam::Vector3 gyros_Vector3(const imu& myImu, const int& idx);
    Eigen::MatrixXd gyroMatrix(const imu& myImu);
    Eigen::MatrixXd accelMatrix(const imu& myImu);
    double median(std::vector<double> v);
    uint nearestIdxToVal(std::vector<double> v, double val);
    void saveMatrixToFile(const gtsam::Matrix& A, const std::string &s, const std::string& filename);
    void writeEigenMatrixToCsvFile(const std::string& name, const Eigen::MatrixXd& matrix, const Eigen::IOFormat& CSVFormat=Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n"));
    void printErrorsInGraphByFactorType(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& vals);
    Eigen::MatrixXd vectorRot3ToFlattedEigenMatrixXd(const std::vector<gtsam::Rot3>& R);

    template<typename factorType> void removeAllFactorsExceptTypeInPlace(gtsam::NonlinearFactorGraph& graph, bool verbose){
        // known bug: this segsegv's if you include MagPose3Factors... not sure why. It works fine for removing gtsam prior factors, at least.
        // this should remove any factors which inherit from type gtsam::Factor except the factorType specified.
        uint origGraphSize=graph.size(); double beginTic=clock();
        // now in a while loop, loop until no changes are made to the graph
        bool didGraphSizeChange=true; uint iterations=0;
        while(didGraphSizeChange) {
            uint graphSizeAtPassStart=graph.size();
            if(verbose){std::cout<<"removeAllFactorsExceptTypeInPlace(): pass #"<<iterations<<std::endl;}
            for (auto it = graph.begin(); it != graph.end(); it++) { // it: factor iterator object
                boost::shared_ptr<gtsam::Factor> a = boost::dynamic_pointer_cast<factorType>(*it);
                if (a) { // you found a factorType
                } else { // this factor is not a factorType, erase it
                    graph.erase(it); // delete this factor from the factor graph!
                }
            }
            uint graphSizeAtPassEnd=graph.size();
            if(graphSizeAtPassStart==graphSizeAtPassEnd){ // no changes were made
                didGraphSizeChange=false;
            }
            iterations++;
            if (verbose) { std::cout << "    graph size at pass start: " << graphSizeAtPassStart <<", graph size at pass end: "<<graphSizeAtPassEnd<<std::endl; }
        }
        uint newGraphSize=graph.size();
        if(verbose){std::cout<<"complete! orig graph size: "<<origGraphSize<<", new graph size: "<<newGraphSize<<" ("<<(clock()-beginTic)/CLOCKS_PER_SEC<<" sec)"<<std::endl;}
    }

    template<typename factorType> int getNumberOfFactorByType(const gtsam::NonlinearFactorGraph& graph){
        // return number of factors of type in m_graph
        int count=0;
        for (const gtsam::NonlinearFactor::shared_ptr& factor : graph) { // you could change this to the graph from the imu pose problem to only loop over those keys, I think.
            auto a = boost::dynamic_pointer_cast<factorType>(factor);
            if (a) { // you found factor of type factorType
                count++;
            }
        }
        return count;
    }
    template<typename factorType> double getGraphErrorDueToFactorType(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& vals){
        // return the total weighted error due to these factors in graph
        int count=0;
        double error=0.0;
        for (const gtsam::NonlinearFactor::shared_ptr& factor : graph) { // you could change this to the graph from the imu pose problem to only loop over those keys, I think.
            auto a = boost::dynamic_pointer_cast<factorType>(factor);
            if (a) { // you found factor of type factorType
                error+=a->error(vals);
                count++;
            }
        }
        return error;
    }
    template<typename valueType> int getNumberOfKeysByType(const gtsam::Values& vals){
        // return number of keys of type in m_graph
        gtsam::KeyVector keyvec=((gtsam::Values)vals.filter<valueType>()).keys();
        return keyvec.size();
    }
    template<typename factorType> void printGraphErrorDueToFactorType(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& vals, std::string str=""){
        // print error in graph due to this type of component, with optional string title str
        // only prints if you have at least one of those factors in the graph
        uint numFac=getNumberOfFactorByType<factorType>(graph);
        double errFac=gtsamutils::getGraphErrorDueToFactorType<factorType>(graph, vals);
        if(numFac>0) { std::cout << str<<" (x" << numFac << ") = " << errFac << "  ("<<(100.0*errFac)/graph.error(vals)<<"%)"<<std::endl;}
    }
    template <typename factorType> void clearGtsamFactorByKey(gtsam::NonlinearFactorGraph& graph, const gtsam::Key& key, bool verbose=false){
        // finds any factorType attached to key and removes it
        uint origGraphSize=graph.size();
        if(verbose){std::cout<<"original graph size: "<<origGraphSize<<std::endl;}
        if(verbose){std::cout<<"key you're looking for: "<<key<<std::endl;}
        for(auto it=graph.begin(); it<graph.end(); it++){ // it: factor iterator object
            boost::shared_ptr<gtsam::NonlinearFactor> a = boost::dynamic_pointer_cast<factorType>(*it);
            if (a) { // you found a factorType
                for(gtsam::Key & k: a->keys()){ // loop over all keys in this factor
                    if(verbose){std::cout<<"searched key: "<<k<<std::endl;}
                    if(k==key){ // you found the key you were looking for!
                        if(verbose){std::cout<<"found key!"<<std::endl;}
                        // delete this factor from the factor graph!
                        graph.erase(it);
                        if(verbose){std::cout<<"erased factor at iterator "<<*it<<std::endl;}
                    }
                } // finish loop over keys
            }
        }
        uint newGraphSize=graph.size();
        if(verbose){std::cout<<"new graph size: "<<newGraphSize<<std::endl;}
    }

    /* ************************************************************************************************************** */
    /* ------ extensions to gtsam's numericalDerivative implementations to support 7 and 8-factor derivatives ------- */
    /* ************************************************************************************************************** */

    // a quick helper struct to get the appropriate fixed sized matrix from two value types
    namespace internal {
        template<class Y, class X=double>
        struct FixedSizeMatrix {
            typedef Eigen::Matrix<double,gtsam::traits<Y>::dimension, gtsam::traits<X>::dimension> type;
        };
    }

    // --- 7-argument numerical derivatives --- //
    // 7 argument derivative, argument #1
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, int N = gtsam::traits<X1>::dimension>
    typename internal::FixedSizeMatrix<Y,X1>::type numericalDerivative71(
            std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&)> h, const X1& x1,
            const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, double delta = 1e-5) {
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<Y>::structure_category>::value),
                                 "Template argument Y must be a manifold type.");
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<X1>::structure_category>::value),
                                 "Template argument X1 must be a manifold type.");
        return gtsam::numericalDerivative11<Y, X1, N>(std::bind(h, std::placeholders::_1, std::cref(x2), std::cref(x3), std::cref(x4), std::cref(x5), std::cref(x6), std::cref(x7)), x1, delta);
    } // ^ this one should always call gtsam::numericalDerivative11<>. Otherwise the numbers should all match.
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7>
    inline typename internal::FixedSizeMatrix<Y,X1>::type numericalDerivative71(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&),
                                                                                const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, double delta = 1e-5) {
        return numericalDerivative71<Y, X1, X2, X3, X4, X5, X6, X7>(std::bind(h, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7), x1, x2, x3, x4, x5, x6, x7);
    }

    // 7 argument derivative, argument #2
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, int N = gtsam::traits<X2>::dimension>
    typename internal::FixedSizeMatrix<Y,X2>::type numericalDerivative72(
            std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&)> h, const X1& x1,
            const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, double delta = 1e-5) {
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<Y>::structure_category>::value),
                                 "Template argument Y must be a manifold type.");
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<X2>::structure_category>::value),
                                 "Template argument X2 must be a manifold type.");
        return gtsam::numericalDerivative11<Y, X2, N>(std::bind(h, std::cref(x1), std::placeholders::_1, std::cref(x3), std::cref(x4), std::cref(x5), std::cref(x6), std::cref(x7)), x2, delta);
    } // ^ this one should always call gtsam::numericalDerivative11<>. Otherwise the numbers should all match.
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7>
    inline typename internal::FixedSizeMatrix<Y,X2>::type numericalDerivative72(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&),
                                                                                const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, double delta = 1e-5) {
        return numericalDerivative72<Y, X1, X2, X3, X4, X5, X6, X7>(std::bind(h, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7), x1, x2, x3, x4, x5, x6, x7);
    }

    // 7 argument derivative, argument #3
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, int N = gtsam::traits<X3>::dimension>
    typename internal::FixedSizeMatrix<Y,X3>::type numericalDerivative73(
            std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&)> h, const X1& x1,
            const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, double delta = 1e-5) {
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<Y>::structure_category>::value),
                                 "Template argument Y must be a manifold type.");
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<X3>::structure_category>::value),
                                 "Template argument X3 must be a manifold type.");
        return gtsam::numericalDerivative11<Y, X3, N>(std::bind(h, std::cref(x1), std::cref(x2), std::placeholders::_1, std::cref(x4), std::cref(x5), std::cref(x6), std::cref(x7)), x3, delta);
    } // ^ this one should always call gtsam::numericalDerivative11<>. Otherwise the numbers should all match.
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7>
    inline typename internal::FixedSizeMatrix<Y,X3>::type numericalDerivative73(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&),
                                                                                const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, double delta = 1e-5) {
        return numericalDerivative73<Y, X1, X2, X3, X4, X5, X6, X7>(std::bind(h, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7), x1, x2, x3, x4, x5, x6, x7);
    }

    // 7 argument derivative, argument #4
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, int N = gtsam::traits<X4>::dimension>
    typename internal::FixedSizeMatrix<Y,X4>::type numericalDerivative74(
            std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&)> h, const X1& x1,
            const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, double delta = 1e-5) {
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<Y>::structure_category>::value),
                                 "Template argument Y must be a manifold type.");
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<X4>::structure_category>::value),
                                 "Template argument X4 must be a manifold type.");
        return gtsam::numericalDerivative11<Y, X4, N>(std::bind(h, std::cref(x1), std::cref(x2), std::cref(x3), std::placeholders::_1, std::cref(x5), std::cref(x6), std::cref(x7)), x4, delta);
    } // ^ this one should always call gtsam::numericalDerivative11<>. Otherwise the numbers should all match.
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7>
    inline typename internal::FixedSizeMatrix<Y,X4>::type numericalDerivative74(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&),
                                                                                const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, double delta = 1e-5) {
        return numericalDerivative74<Y, X1, X2, X3, X4, X5, X6, X7>(std::bind(h, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7), x1, x2, x3, x4, x5, x6, x7);
    }

    // 7 argument derivative, argument #5
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, int N = gtsam::traits<X5>::dimension>
    typename internal::FixedSizeMatrix<Y,X5>::type numericalDerivative75(
            std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&)> h, const X1& x1,
            const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, double delta = 1e-5) {
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<Y>::structure_category>::value),
                                 "Template argument Y must be a manifold type.");
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<X5>::structure_category>::value),
                                 "Template argument X5 must be a manifold type.");
        return gtsam::numericalDerivative11<Y, X5, N>(std::bind(h, std::cref(x1), std::cref(x2), std::cref(x3), std::cref(x4), std::placeholders::_1, std::cref(x6), std::cref(x7)), x5, delta);
    } // ^ this one should always call gtsam::numericalDerivative11<>. Otherwise the numbers should all match.
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7>
    inline typename internal::FixedSizeMatrix<Y,X5>::type numericalDerivative75(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&),
                                                                                const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, double delta = 1e-5) {
        return numericalDerivative75<Y, X1, X2, X3, X4, X5, X6, X7>(std::bind(h, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7), x1, x2, x3, x4, x5, x6, x7);
    }

    // 7 argument derivative, argument #6
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, int N = gtsam::traits<X6>::dimension>
    typename internal::FixedSizeMatrix<Y,X6>::type numericalDerivative76(
            std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&)> h, const X1& x1,
            const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, double delta = 1e-5) {
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<Y>::structure_category>::value),
                                 "Template argument Y must be a manifold type.");
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<X6>::structure_category>::value),
                                 "Template argument X6 must be a manifold type.");
        return gtsam::numericalDerivative11<Y, X6, N>(std::bind(h, std::cref(x1), std::cref(x2), std::cref(x3), std::cref(x4), std::cref(x5), std::placeholders::_1, std::cref(x7)), x6, delta);
    } // ^ this one should always call gtsam::numericalDerivative11<>. Otherwise the numbers should all match.
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7>
    inline typename internal::FixedSizeMatrix<Y,X6>::type numericalDerivative76(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&),
                                                                                const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, double delta = 1e-5) {
        return numericalDerivative76<Y, X1, X2, X3, X4, X5, X6, X7>(std::bind(h, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7), x1, x2, x3, x4, x5, x6, x7);
    }

    // 7 argument derivative, argument #7
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, int N = gtsam::traits<X7>::dimension>
    typename internal::FixedSizeMatrix<Y,X7>::type numericalDerivative77(
            std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&)> h, const X1& x1,
            const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, double delta = 1e-5) {
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<Y>::structure_category>::value),
                                 "Template argument Y must be a manifold type.");
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<X7>::structure_category>::value),
                                 "Template argument X7 must be a manifold type.");
        return gtsam::numericalDerivative11<Y, X7, N>(std::bind(h, std::cref(x1), std::cref(x2), std::cref(x3), std::cref(x4), std::cref(x5), std::cref(x6), std::placeholders::_1), x7, delta);
    } // ^ this one should always call gtsam::numericalDerivative11<>. Otherwise the numbers should all match.
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7>
    inline typename internal::FixedSizeMatrix<Y,X7>::type numericalDerivative77(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&),
                                                                                const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, double delta = 1e-5) {
        return numericalDerivative77<Y, X1, X2, X3, X4, X5, X6, X7>(std::bind(h, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7), x1, x2, x3, x4, x5, x6, x7);
    }

// --- end numericalDerivative7

// --- 8-argument numerical derivatives --- //
    // 8 argument derivative, argument #1
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, class X8, int N = gtsam::traits<X1>::dimension>
    typename internal::FixedSizeMatrix<Y,X1>::type numericalDerivative81(
            std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&, const X8&)> h, const X1& x1,
            const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, const X8& x8, double delta = 1e-5) {
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<Y>::structure_category>::value),
                                 "Template argument Y must be a manifold type.");
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<X1>::structure_category>::value),
                                 "Template argument X1 must be a manifold type.");
        return gtsam::numericalDerivative11<Y, X1, N>(std::bind(h, std::placeholders::_1, std::cref(x2), std::cref(x3), std::cref(x4), std::cref(x5), std::cref(x6), std::cref(x7), std::cref(x8)), x1, delta);
    } // ^ this one should always call gtsam::numericalDerivative11<>. Otherwise the numbers should all match.
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, class X8>
    inline typename internal::FixedSizeMatrix<Y,X1>::type numericalDerivative81(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&, const X8&),
                                                                                const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, const X8& x8, double delta = 1e-5) {
        return numericalDerivative81<Y, X1, X2, X3, X4, X5, X6, X7>(std::bind(h, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8), x1, x2, x3, x4, x5, x6, x7, x8);
    }

    // 8 argument derivative, argument #2
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, class X8, int N = gtsam::traits<X2>::dimension>
    typename internal::FixedSizeMatrix<Y,X2>::type numericalDerivative82(
            std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&, const X8&)> h, const X1& x1,
            const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, const X8& x8, double delta = 1e-5) {
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<Y>::structure_category>::value),
                                 "Template argument Y must be a manifold type.");
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<X2>::structure_category>::value),
                                 "Template argument X2 must be a manifold type.");
        return gtsam::numericalDerivative11<Y, X2, N>(std::bind(h, std::cref(x1), std::placeholders::_1, std::cref(x3), std::cref(x4), std::cref(x5), std::cref(x6), std::cref(x7), std::cref(x8)), x2, delta);
    } // ^ this one should always call gtsam::numericalDerivative11<>. Otherwise the numbers should all match.
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, class X8>
    inline typename internal::FixedSizeMatrix<Y,X2>::type numericalDerivative82(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&, const X8&),
                                                                                const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, const X8& x8, double delta = 1e-5) {
        return numericalDerivative82<Y, X1, X2, X3, X4, X5, X6, X7>(std::bind(h, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8), x1, x2, x3, x4, x5, x6, x7, x8);
    }

    // 8 argument derivative, argument #3
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, class X8, int N = gtsam::traits<X3>::dimension>
    typename internal::FixedSizeMatrix<Y,X3>::type numericalDerivative83(
            std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&, const X8&)> h, const X1& x1,
            const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, const X8& x8, double delta = 1e-5) {
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<Y>::structure_category>::value),
                                 "Template argument Y must be a manifold type.");
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<X3>::structure_category>::value),
                                 "Template argument X3 must be a manifold type.");
        return gtsam::numericalDerivative11<Y, X3, N>(std::bind(h, std::cref(x1), std::cref(x2), std::placeholders::_1, std::cref(x4), std::cref(x5), std::cref(x6), std::cref(x7), std::cref(x8)), x3, delta);
    } // ^ this one should always call gtsam::numericalDerivative11<>. Otherwise the numbers should all match.
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, class X8>
    inline typename internal::FixedSizeMatrix<Y,X3>::type numericalDerivative83(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&, const X8&),
                                                                                const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, const X8& x8, double delta = 1e-5) {
        return numericalDerivative83<Y, X1, X2, X3, X4, X5, X6, X7>(std::bind(h, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8), x1, x2, x3, x4, x5, x6, x7, x8);
    }

    // 8 argument derivative, argument #4
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, class X8, int N = gtsam::traits<X4>::dimension>
    typename internal::FixedSizeMatrix<Y,X4>::type numericalDerivative84(
            std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&, const X8&)> h, const X1& x1,
            const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, const X8& x8, double delta = 1e-5) {
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<Y>::structure_category>::value),
                                 "Template argument Y must be a manifold type.");
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<X4>::structure_category>::value),
                                 "Template argument X4 must be a manifold type.");
        return gtsam::numericalDerivative11<Y, X4, N>(std::bind(h, std::cref(x1), std::cref(x2), std::cref(x3), std::placeholders::_1, std::cref(x5), std::cref(x6), std::cref(x7), std::cref(x8)), x4, delta);
    } // ^ this one should always call gtsam::numericalDerivative11<>. Otherwise the numbers should all match.
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, class X8>
    inline typename internal::FixedSizeMatrix<Y,X4>::type numericalDerivative84(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&, const X8&),
                                                                                const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, const X8& x8, double delta = 1e-5) {
        return numericalDerivative84<Y, X1, X2, X3, X4, X5, X6, X7>(std::bind(h, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8), x1, x2, x3, x4, x5, x6, x7, x8);
    }

    // 8 argument derivative, argument #5
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, class X8, int N = gtsam::traits<X5>::dimension>
    typename internal::FixedSizeMatrix<Y,X5>::type numericalDerivative85(
            std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&, const X8&)> h, const X1& x1,
            const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, const X8& x8, double delta = 1e-5) {
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<Y>::structure_category>::value),
                                 "Template argument Y must be a manifold type.");
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<X5>::structure_category>::value),
                                 "Template argument X5 must be a manifold type.");
        return gtsam::numericalDerivative11<Y, X5, N>(std::bind(h, std::cref(x1), std::cref(x2), std::cref(x3), std::cref(x4), std::placeholders::_1, std::cref(x6), std::cref(x7), std::cref(x8)), x5, delta);
    } // ^ this one should always call gtsam::numericalDerivative11<>. Otherwise the numbers should all match.
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, class X8>
    inline typename internal::FixedSizeMatrix<Y,X5>::type numericalDerivative85(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&, const X8&),
                                                                                const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, const X8& x8, double delta = 1e-5) {
        return numericalDerivative85<Y, X1, X2, X3, X4, X5, X6, X7>(std::bind(h, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8), x1, x2, x3, x4, x5, x6, x7, x8);
    }

    // 8 argument derivative, argument #6
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, class X8, int N = gtsam::traits<X6>::dimension>
    typename internal::FixedSizeMatrix<Y,X6>::type numericalDerivative86(
            std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&, const X8&)> h, const X1& x1,
            const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, const X8& x8, double delta = 1e-5) {
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<Y>::structure_category>::value),
                                 "Template argument Y must be a manifold type.");
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<X6>::structure_category>::value),
                                 "Template argument X6 must be a manifold type.");
        return gtsam::numericalDerivative11<Y, X6, N>(std::bind(h, std::cref(x1), std::cref(x2), std::cref(x3), std::cref(x4), std::cref(x5), std::placeholders::_1, std::cref(x7), std::cref(x8)), x6, delta);
    } // ^ this one should always call gtsam::numericalDerivative11<>. Otherwise the numbers should all match.
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, class X8>
    inline typename internal::FixedSizeMatrix<Y,X6>::type numericalDerivative86(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&, const X8&),
                                                                                const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, const X8& x8, double delta = 1e-5) {
        return numericalDerivative86<Y, X1, X2, X3, X4, X5, X6, X7>(std::bind(h, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8), x1, x2, x3, x4, x5, x6, x7, x8);
    }

    // 8 argument derivative, argument #7
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, class X8, int N = gtsam::traits<X7>::dimension>
    typename internal::FixedSizeMatrix<Y,X7>::type numericalDerivative87(
            std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&, const X8&)> h, const X1& x1,
            const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, const X8& x8, double delta = 1e-5) {
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<Y>::structure_category>::value),
                                 "Template argument Y must be a manifold type.");
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<X7>::structure_category>::value),
                                 "Template argument X7 must be a manifold type.");
        return gtsam::numericalDerivative11<Y, X7, N>(std::bind(h, std::cref(x1), std::cref(x2), std::cref(x3), std::cref(x4), std::cref(x5), std::cref(x6), std::placeholders::_1, std::cref(x8)), x7, delta);
    } // ^ this one should always call gtsam::numericalDerivative11<>. Otherwise the numbers should all match.
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, class X8>
    inline typename internal::FixedSizeMatrix<Y,X7>::type numericalDerivative87(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&, const X8&),
                                                                                const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, const X8& x8, double delta = 1e-5) {
        return numericalDerivative87<Y, X1, X2, X3, X4, X5, X6, X7>(std::bind(h, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8), x1, x2, x3, x4, x5, x6, x7, x8);
    }

    // 8 argument derivative, argument #8
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, class X8, int N = gtsam::traits<X8>::dimension>
    typename internal::FixedSizeMatrix<Y,X8>::type numericalDerivative88(
            std::function<Y(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&, const X8&)> h, const X1& x1,
            const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, const X8& x8, double delta = 1e-5) {
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<Y>::structure_category>::value),
                                 "Template argument Y must be a manifold type.");
        BOOST_STATIC_ASSERT_MSG( (boost::is_base_of<gtsam::manifold_tag, typename gtsam::traits<X8>::structure_category>::value),
                                 "Template argument X8 must be a manifold type.");
        return gtsam::numericalDerivative11<Y, X8, N>(std::bind(h, std::cref(x1), std::cref(x2), std::cref(x3), std::cref(x4), std::cref(x5), std::cref(x6), std::cref(x7), std::placeholders::_1), x8, delta);
    } // ^ this one should always call gtsam::numericalDerivative11<>. Otherwise the numbers should all match.
    template<class Y, class X1, class X2, class X3, class X4, class X5, class X6, class X7, class X8>
    inline typename internal::FixedSizeMatrix<Y,X8>::type numericalDerivative88(Y (*h)(const X1&, const X2&, const X3&, const X4&, const X5&, const X6&, const X7&, const X8&),
                                                                                const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6, const X7& x7, const X8& x8, double delta = 1e-5) {
        return numericalDerivative88<Y, X1, X2, X3, X4, X5, X6, X7>(std::bind(h, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8), x1, x2, x3, x4, x5, x6, x7, x8);
    }
    // --- end numericalDerivative8 ---
    /* ************************************************************************************************************** */
    /* ************************************************************************************************************** */
    /* ************************************************************************************************************** */
}