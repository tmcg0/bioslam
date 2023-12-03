// -------------------------------------------------------------------- //
//    (c) Copyright 2021 Massachusetts Institute of Technology          //
//    Author: Tim McGrath                                               //
//    All rights reserved. See LICENSE file for license information.    //
// -------------------------------------------------------------------- //

// a simple class to hold IMU data and various methods
#include "imu/imu.h"
#include <exception>
#include <regex>
// hdf5 includes
#include "H5Cpp.h"
// highfive includes
#include <highfive/H5File.hpp>
#include <highfive/H5DataSet.hpp>

namespace h5=HighFive;

// forward declare helper functions
imudata readSingleImuDataFromApdmOpalH5FileByLabel(const std::string& filestr, const std::string& label);
bool is_apdm_h5_version5(const std::string& filestr);
uint imuSensorStrToInt(const std::string& str);
std::string get_sensor_label_from_apdm_v5_by_sensor_number(const std::string& filename, const std::string& sensorNumber);
std::vector<std::string> getAllImuLabelsInDataFile(const std::string& filestr);


imu::imu(const std::string& filePath, const std::string& labelName){
    imudata myImuData=readSingleImuDataFromApdmOpalH5FileByLabel(filePath, labelName);
    this->ax=myImuData.ax; this->ay=myImuData.ay; this->az=myImuData.az;
    this->gx=myImuData.gx; this->gy=myImuData.gy; this->gz=myImuData.gz;
    this->mx=myImuData.mx; this->my=myImuData.my; this->mz=myImuData.mz;
    this->relTimeSec=myImuData.relTimeSec;
    this->label=myImuData.label;
    this->id=myImuData.id;
    if(myImuData.qx.size()>0){ // if orientation exists
        this->qs=myImuData.qs; this->qx=myImuData.qx; this->qy=myImuData.qy; this->qz=myImuData.qz;
    }
    // create vector<double> to hold UTC time in seconds
    std::vector<double> unixTimeUtc(myImuData.unixTimeUtcMicrosec.size());
    for(uint k=0; k<myImuData.unixTimeUtcMicrosec.size(); ++k){
        unixTimeUtc[k]=myImuData.unixTimeUtcMicrosec[k]/1.0e6;
    }
    this->unixTimeUtc=unixTimeUtc;
} // end constructor

std::map<std::string,imu> imu::getImuMapFromDataFile(const std::string& filestr){
    // this static method constructs an imu map where the keys are the label std::strings of the imu
    std::vector<std::string> allLabels=getAllImuLabelsInDataFile(filestr);
    std::map<std::string,imu> ImuMap;
    for(auto & label : allLabels){ // now create map
        imu tempImu(filestr,label);
        ImuMap.insert(std::pair<std::string,imu>(label,tempImu));
    }
    return ImuMap;
}

unsigned long imu::length() const {
    return this->ax.size();
}

void imu::printLabelsInFile(const std::string& datafilestr){
    // prints all imu labels from given file
    std::vector<std::string> allLabels=getAllImuLabelsInDataFile(datafilestr);
    for(uint i=0;i<allLabels.size();i++){ // now create map
        std::cout<<"imu label "<<i<<": "<<allLabels[i]<<std::endl;
    }
}

void imu::verifyDataPackaging(bool verbose){
    // verify that the data is good. all same size.
    uint testLength=this->length(); // let's hope they're all this size
    if(this->ax.size()!=testLength){throw std::runtime_error("inconsistent size.");}
    if(this->ay.size()!=testLength){throw std::runtime_error("inconsistent size.");}
    if(this->az.size()!=testLength){throw std::runtime_error("inconsistent size.");}
    if(this->gx.size()!=testLength){throw std::runtime_error("inconsistent size.");}
    if(this->gy.size()!=testLength){throw std::runtime_error("inconsistent size.");}
    if(this->gz.size()!=testLength){throw std::runtime_error("inconsistent size.");}
    if(this->mx.size()!=testLength){throw std::runtime_error("inconsistent size.");}
    if(this->my.size()!=testLength){throw std::runtime_error("inconsistent size.");}
    if(this->mz.size()!=testLength){throw std::runtime_error("inconsistent size.");}
    if(this->relTimeSec.size()!=testLength){throw std::runtime_error("inconsistent size.");}
    // now test quaternion
    if(this->qs.size()!=testLength){throw std::runtime_error("inconsistent size.");}
    if(this->qx.size()!=testLength){throw std::runtime_error("inconsistent size.");}
    if(this->qy.size()!=testLength){throw std::runtime_error("inconsistent size.");}
    if(this->qz.size()!=testLength){throw std::runtime_error("inconsistent size.");}
    if(verbose){
        std::cout<<"everything looks good with this imu."<<std::endl;
    }
}

gtsam::Vector3 imu::accelVec(const uint idx) const {
    return {this->ax[idx],this->ay[idx],this->az[idx]};
}

gtsam::Vector3 imu::magVec(const uint idx) const {
    return {this->mx[idx],this->my[idx],this->mz[idx]};
}

gtsam::Vector3 imu::gyroVec(const uint idx) const {
    return {this->gx[idx],this->gy[idx],this->gz[idx]};
}

std::vector<std::vector<double>> imu::quaternion() const {
    std::vector<std::vector<double>> q(this->length());
    for(uint i=0; i<this->length();i++){
        q[i]={qs[i],qx[i],qy[i],qz[i]};
    }
    return q;
}

Eigen::MatrixXd imu::gyroMat() const {
    Eigen::MatrixXd gyros(this->length(),3);
    for(uint k=0; k<this->length();k++){
        gyros(k,0)=this->gx[k]; gyros(k,1)=this->gy[k]; gyros(k,2)=this->gz[k];
    }
    return gyros;
}
Eigen::MatrixXd imu::accelMat() const {
    Eigen::MatrixXd accels(this->length(),3);
    for(uint k=0; k<this->length();k++){
        accels(k,0)=this->ax[k]; accels(k,1)=this->ay[k]; accels(k,2)=this->az[k];
    }
    return accels;
}

imu imu::cutImuByIdx(uint startIdx, uint stopIdx){
    // cut imu data down and return the chopped imu--note that this copies the imu so the original input imu is not affected
    imu newImu=imu(*this); // copy
    // now go through the imu data fields and chop down by indexes in the new imu
    if(newImu.gx.size()>stopIdx){newImu.gx=slice(newImu.gx,startIdx,stopIdx);}
    if(newImu.gy.size()>stopIdx){newImu.gy=slice(newImu.gy,startIdx,stopIdx);}
    if(newImu.gz.size()>stopIdx){newImu.gz=slice(newImu.gz,startIdx,stopIdx);}
    if(newImu.ax.size()>stopIdx){newImu.ax=slice(newImu.ax,startIdx,stopIdx);}
    if(newImu.ay.size()>stopIdx){newImu.ay=slice(newImu.ay,startIdx,stopIdx);}
    if(newImu.az.size()>stopIdx){newImu.az=slice(newImu.az,startIdx,stopIdx);}
    if(newImu.mx.size()>stopIdx){newImu.mx=slice(newImu.mx,startIdx,stopIdx);}
    if(newImu.my.size()>stopIdx){newImu.my=slice(newImu.my,startIdx,stopIdx);}
    if(newImu.mz.size()>stopIdx){newImu.mz=slice(newImu.mz,startIdx,stopIdx);}
    if(newImu.unixTimeUtc.size()>stopIdx){newImu.unixTimeUtc=slice(newImu.unixTimeUtc,startIdx,stopIdx);}
    if(newImu.relTimeSec.size()>stopIdx){newImu.relTimeSec=slice(newImu.relTimeSec,startIdx,stopIdx);}
    return newImu;
}

// --- helper functions --- //
imudata readSingleImuDataFromApdmOpalH5FileByLabel(const std::string& filestr, const std::string& label){
    if (!is_apdm_h5_version5(filestr)){
        throw std::runtime_error("this is not a valid v5 apdm .h5 file");
    }
    // for h5 file structure for APDM see: http://share.apdm.com/documentation/TechnicalGuide.pdf
    h5::File file(filestr, h5::File::ReadOnly);
    h5::Group rootGroup=file.getGroup("/");
    h5::Group sensorsGroup=rootGroup.getGroup("Sensors");
    h5::Group processedGroup=rootGroup.getGroup("Processed");
    std::vector<std::string> availSensorsStr=sensorsGroup.listObjectNames();
    // now loop through availSensors and construct imu objects and write data
    bool sensorFoundByLabel=false;
    for(auto & currentLabel : availSensorsStr){
        // check if label is correct
        if (get_sensor_label_from_apdm_v5_by_sensor_number(filestr, currentLabel)==label){ // you've found the correct label, continue
            h5::Group currentSensorGroup=sensorsGroup.getGroup(currentLabel);
            // read time. Warning: sometimes it's a double and sometimes it's a int.
            //    looks like this is actually dependent on the version from ADPM. I had two files, both were v5 h5 files, but one was encoded as unit and one as double...
            std::cerr.setstate(std::ios_base::failbit); // turn off error reporting for this line
            std::vector<double> unixTimeUtcMicroseconds=readDataFromDataset<double>(currentSensorGroup.getDataSet("Time"));
            std::cerr.clear(); // turn on error reporting
            // read sensor measurements
            std::vector<std::vector<double>> dataAccel=readDataFromDataset<std::vector<double>>(currentSensorGroup.getDataSet("Accelerometer"));
            std::vector<std::vector<double>> dataGyro=readDataFromDataset<std::vector<double>>(currentSensorGroup.getDataSet("Gyroscope"));
            std::vector<std::vector<double>> dataMag=readDataFromDataset<std::vector<double>>(currentSensorGroup.getDataSet("Magnetometer"));
            // now loop through and set data
            uint vecLen=dataAccel.size();
            std::vector<double> t(vecLen), ax(vecLen), ay(vecLen), az(vecLen), gx(vecLen), gy(vecLen), gz(vecLen), mx(vecLen), my(vecLen), mz(vecLen);
            // todo: pull out quat into 2d data and set it to vectors for qs, qx, qy, qz
            for (uint j=0;j<vecLen;j++) {
                ax[j] = dataAccel[j][0]; ay[j] = dataAccel[j][1]; az[j] = dataAccel[j][2];
                gx[j] = dataGyro[j][0]; gy[j] = dataGyro[j][1]; gz[j] = dataGyro[j][2];
                mx[j] = dataMag[j][0]; my[j] = dataMag[j][1]; mz[j] = dataMag[j][2];
                t[j] = (unixTimeUtcMicroseconds[j] - unixTimeUtcMicroseconds[0]) / 1.0e6; // convert from unix time to relative time vector in seconds
            }// for loop to store data
            // now also pull out quaternion, if exists
            std::vector<std::vector<double>> q;
            bool quatExists=false;
            if(processedGroup.getGroup(currentLabel).exist("Orientation")){
                q=readDataFromDataset<std::vector<double>>(processedGroup.getGroup(currentLabel).getDataSet("Orientation"));
                quatExists=true;
            }
            // now loop over and set qs, qx, qy, qz
            std::vector<double> qs(vecLen), qx(vecLen), qy(vecLen), qz(vecLen);
            if(quatExists) {
                for (uint k = 0; k < vecLen; k++) {
                    qs[k] = q[k][0];
                    qx[k] = q[k][1];
                    qy[k] = q[k][2];
                    qz[k] = q[k][3];
                }
            }
            //std::vector<Eigen::Vector4d> qAPDM=rotutils::VectorVectorDoubleToVectorEigenVector(qAPDM0);
            //std::vector<gtsam::Rot3> orientation_Rot3=rotutils::QuaternionVectorToRot3Vector(qAPDM); // q[NWU->L]
            // remember: Eigen::Quaternion stores scalar component last
            // now store data in an imudata struct
            imudata dataout;
            dataout.ax=ax; dataout.ay=ay; dataout.az=az;
            dataout.gx=gx; dataout.gy=gy; dataout.gz=gz;
            dataout.mx=mx; dataout.my=my; dataout.mz=mz;
            dataout.qs=qs; dataout.qy=qy; dataout.qx=qx; dataout.qz=qz;
            dataout.relTimeSec=t; dataout.unixTimeUtcMicrosec=unixTimeUtcMicroseconds;
            if(quatExists){dataout.qs=qs; dataout.qy=qy; dataout.qx=qx; dataout.qz=qz;}
            //dataout.orientation=orientation_Rot3;
            dataout.label=get_sensor_label_from_apdm_v5_by_sensor_number(filestr, currentLabel);
            dataout.id=imuSensorStrToInt(currentLabel);
            sensorFoundByLabel=true;
            return dataout;
        } // if is the right label
    } // finished loop over sensors
    if (!sensorFoundByLabel){
        throw std::runtime_error("ERROR: sensor not found!");
    }
    return imudata(); // should never get here.
} // end function

bool is_apdm_h5_version5(const std::string& filestr){
    // is this a version 5 apdm file?
    try{
        h5::File file(filestr, h5::File::ReadOnly);
        h5::Group rootGroup=file.getGroup("/");
        std::vector<std::string> rootAttrs=rootGroup.listAttributeNames();
        h5::Attribute attr=rootGroup.getAttribute(rootAttrs[0]);
        // read format version. Warning: APDM tried to store this as a unit, but HighFive throws an error if you try to read it as anything other than double.
            //    not sure if this is an .h5 format or a HighFive or an APDM problem.
        std::cerr.setstate(std::ios_base::failbit); // turn off error reporting for this line
        double fileFormatVer;
        attr.read(fileFormatVer);
        std::cerr.clear(); // turn on error reporting
        return fileFormatVer == 5.0;
    }catch(std::exception& e){}
    return false;
} // end func

uint imuSensorStrToInt(const std::string& str){
    // takes in example string, like XI-0023412 and converts it to int
    std::string numeric = std::regex_replace(str, std::regex(R"([\D])"), "");
    uint id=std::stoi(numeric);
    return id;
}

std::string get_sensor_label_from_apdm_v5_by_sensor_number(const std::string& filename, const std::string& sensorNumber){
    H5::H5File file(filename, H5F_ACC_RDONLY);
    H5::Group sensorConfigGroup = file.openGroup("/Sensors/"+sensorNumber+"/Configuration/");
    if (sensorConfigGroup.attrExists("Label 0")){
        H5::Attribute attr_date = sensorConfigGroup.openAttribute("Label 0");
        H5::StrType stype = attr_date.getStrType();
        std::string date_str;
        attr_date.read(stype, date_str);
        return date_str;
    }else{
        throw std::runtime_error("ERROR: label 'Label 0' not found");
    }
}

double imu::getDeltaT() const {
    // loop through and find average delta T (seconds)
    std::vector<double> timeDiff(this->length()-1);
    for(uint i=0; i<timeDiff.size(); i++){
        timeDiff[i]=this->relTimeSec[i+1]-this->relTimeSec[i];
    }
    double averageDeltaT = accumulate( timeDiff.begin(), timeDiff.end(), 0.0)/timeDiff.size();
    return averageDeltaT;
}

std::vector<std::string> getAllImuLabelsInDataFile(const std::string& filestr){
    std::vector<std::string> allLabels;
    if (is_apdm_h5_version5(filestr)){
        // you have a version 5 APDM data file. we'll use this.
        h5::File file(filestr, h5::File::ReadOnly);
        h5::Group rootGroup=file.getGroup("/");
        h5::Group sensorsGroup=rootGroup.getGroup("Sensors");
        std::vector<std::string> availSensorsStr=sensorsGroup.listObjectNames();
        // now loop through availSensors and construct imu objects and write data
        for(uint i=0;i<availSensorsStr.size();i++){
            h5::Group currentSensorGroup=sensorsGroup.getGroup(availSensorsStr[i]);
            std::vector<std::string> sensorAttributes=currentSensorGroup.getGroup("Configuration").listAttributeNames();
            std::string lbl=get_sensor_label_from_apdm_v5_by_sensor_number(filestr, availSensorsStr[i]);
            allLabels.push_back(lbl);
        } // loop over sensors
        return allLabels;
    } else{
        std::cerr<<"ERROR: unrecognized data file type"<<std::endl;
        std::vector<std::string> returnErr={"ERROR"};
        return returnErr;
    }
}