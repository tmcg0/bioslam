// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

#include "testutils.h"

namespace testutils
{

    std::string getTestDataDir(){
        // assumes test data is at bioslamroot/test/data
        boost::filesystem::path testdatadir=boost::filesystem::path(getBioslamRootDir()+"/test/data");
        return testdatadir.string();
    }

    void runtime_assert(bool condition){
        // implementation of assert() that throws a runtime error if failed. not optimized out by release builds.
        if(!condition){
            throw std::runtime_error("testutils::runtime_assert(): assertion failed.");
        }
    }

    std::string getBioslamRootDir(){
        // iterates down to current path until it finds directory including 'bioslam'
        boost::filesystem::path bioslamroot;
        bool foundBioslamRoot=false;
        for(auto& part : boost::filesystem::current_path()) {
            bioslamroot /= part.c_str();
            if((part.string()).find("bioslam") != std::string::npos){ // you found it. exit.
                foundBioslamRoot=true;
                break;
            }
        }
        if(!foundBioslamRoot){
            throw std::runtime_error("EXCEPTION: could not bioslam root directory! (searched for directory in tree named 'bioslam')");
        }
        return bioslamroot.string();
    }

    std::string getTestDataFile(const std::string& filename){
        // before returning string to full path of file, checks to make sure it exists
        std::string proposedFile=getTestDataDir()+"/"+filename;
        if(boost::filesystem::exists(proposedFile)){//file exists, all is well
            return proposedFile;
        }else{ // bad
            throw std::runtime_error("Error: file \""+filename+"\" does not exist.");
        }
    }

    gtsam::Pose3 randomPose3(){
        return gtsam::Pose3(randomRot3(),randomPoint3());
    }

    gtsam::Rot3 randomRot3(){
        Eigen::Vector4d randQuat1=randomQuat();
        return gtsam::Rot3(randQuat1[0],randQuat1[1],randQuat1[2],randQuat1[3]);
    }

    gtsam::Point3 randomPoint3(){
        return {dRand(),dRand(),dRand()};
    }

    gtsam::Unit3 randomUnit3(){
        return gtsam::Unit3(randomPoint3());
    }

    double dRand(double lower_bound, double upper_bound){
        std::random_device r;
        std::uniform_real_distribution<double> unif(lower_bound,upper_bound);
        std::default_random_engine re(r());
        double a_random_double = unif(re);
        return a_random_double;
    }

    Eigen::Vector4d randomQuat(){
        // generate a random quaternion
        double qr=dRand(-1.0,1.0);
        double qi=dRand(-1.0,1.0);
        double qj=dRand(-1.0,1.0);
        double qk=dRand(-1.0,1.0);
        Eigen::Vector4d q; q<<qr,qi,qj,qk;
        q.normalize();
        return q;
    }

    Eigen::Vector4d randomDiffQuat(){
        // generate a random differential quaternion (near unit quaternion)
        double qr=dRand(0.95,1.0);
        double qi=dRand(-0.15,0.15);
        double qj=dRand(-0.15,0.15);
        double qk=dRand(-0.15,0.15);
        Eigen::Vector4d q; q<<qr,qi,qj,qk;
        q.normalize();
        return q;
    }

    gtsam::Vector3 randomVector3(){
        return {dRand(),dRand(),dRand()};
    }
} // namespace