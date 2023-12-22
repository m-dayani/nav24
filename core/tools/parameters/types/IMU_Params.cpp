//
// Created by root on 5/13/21.
//

#include "IMU_Params.hpp"

using namespace std;

namespace NAV24 {

    IMU_Params::IMU_Params() :
            missParams(false), freq(100.f), Na(0.f), Naw(0.f), Ng(0.f), Ngw(0.f), sf(1.f)
    {}

    IMU_Params::IMU_Params(const cv::FileStorage &fSettings) : IMU_Params() {

        this->read(fSettings["IMU"]);
    }

    void IMU_Params::write(cv::FileStorage &fs) const {

        fs << "IMU" << "{";

        YamlParserCV::writeReal(fs, "Frequency", freq);
        YamlParserCV::writeReal(fs, "NoiseGyro", Ng);
        YamlParserCV::writeReal(fs, "NoiseAcc", Na);
        YamlParserCV::writeReal(fs, "GyroWalk", Ngw);
        YamlParserCV::writeReal(fs, "AccWalk", Naw);

        fs << "}";
    }

    void IMU_Params::read(const cv::FileNode &imuNode) {

        freq = YamlParserCV::readReal<float>(imuNode, "Frequency", freq);

        Ng = YamlParserCV::readReal<float>(imuNode, "NoiseGyro", Ng);

        Na = YamlParserCV::readReal<float>(imuNode, "NoiseAcc", Na);

        Ngw = YamlParserCV::readReal<float>(imuNode, "GyroWalk", Ngw);

        Naw = YamlParserCV::readReal<float>(imuNode, "AccWalk", Naw);
    }

    std::string IMU_Params::printStr(const std::string& prefix) {

        ostringstream oss;

        oss << prefix << "Frequency: " << freq << endl;
        oss << prefix << "Gyro Noise: " << Ng << endl;
        oss << prefix << "Gyro Random Walk: " << Ngw << endl;
        oss << prefix << "Accelerometer Noise: " << Na << endl;
        oss << prefix << "Accelerometer Random Walk: " << Naw << endl;

        return oss.str();
    }

} // NAV24
