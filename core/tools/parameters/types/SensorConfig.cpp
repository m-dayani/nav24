//
// Created by root on 5/10/21.
//

#include "SensorConfig.hpp"

using namespace std;

namespace NAV24 {

    SensorConfig::SensorConfig(const cv::FileStorage &fs) : SensorConfig() {

        this->read(fs);
    }

    SensorConfig::SensorConfig(const cv::FileNode &fn) : SensorConfig() {

        this->read(fn);
    }

    string SensorConfig::mapConfig(const SensorConfigEnum dsConfig) {

        switch (dsConfig) {
            case SensorConfig::MONOCULAR:
                return "mono_im";
            case SensorConfig::IMU_MONOCULAR:
                return "mono_im_imu";
            case SensorConfig::EVENT_ONLY:
                return "mono_ev";
            case SensorConfig::EVENT_MONO:
                return "mono_ev_im";
            case SensorConfig::EVENT_IMU:
                return "mono_ev_imu";
            case SensorConfig::EVENT_IMU_MONO:
                return "mono_ev_im_imu";
            default:
                return "idle";
        }
    }

    SensorConfig::SensorConfigEnum SensorConfig::mapConfig(const string &dsConfig) {

        if (dsConfig == "mono_im")
            return SensorConfig::MONOCULAR;
        else if (dsConfig == "mono_im_imu")
            return SensorConfig::IMU_MONOCULAR;
        else if (dsConfig == "mono_ev")
            return SensorConfig::EVENT_ONLY;
        else if (dsConfig == "mono_ev_im")
            return SensorConfig::EVENT_MONO;
        else if (dsConfig == "mono_ev_imu")
            return SensorConfig::EVENT_IMU;
        else if (dsConfig == "mono_ev_im_imu")
            return SensorConfig::EVENT_IMU_MONO;
        else
            return SensorConfig::IDLE;
    }

    string SensorConfig::toDsStr() const {
        return SensorConfig::mapConfig(mSensor);
    }

    /*System::eSensor System::convertSensorConfig(EORB_SLAM::MyDataTypes::SensorConfig sConf) {

        switch (sConf) {
            case EORB_SLAM::MyDataTypes::MONOCULAR:
                return System::MONOCULAR;
            case EORB_SLAM::MyDataTypes::STEREO:
                return System::STEREO;
            case EORB_SLAM::MyDataTypes::IMU_MONOCULAR:
                return System::IMU_MONOCULAR;
            case EORB_SLAM::MyDataTypes::IMU_STEREO:
                return System::IMU_STEREO;
            case EORB_SLAM::MyDataTypes::RGBD:
                return System::RGBD;
            case EORB_SLAM::MyDataTypes::EVENT_ONLY:
                return System::EVENT_ONLY;
            case EORB_SLAM::MyDataTypes::EVENT_IMU:
                return System::EVENT_IMU;
            case EORB_SLAM::MyDataTypes::EVENT_MONO:
                return System::EVENT_MONO;
            case EORB_SLAM::MyDataTypes::EVENT_IMU_MONO:
                return System::EVENT_IMU_MONO;
            default:
                return System::IDLE;
        }
    }*/

    string SensorConfig::toStr() const {

        switch (mSensor) {
            case SensorConfig::MONOCULAR:
                return "Monocular";
            case SensorConfig::STEREO:
                return "Stereo";
            case SensorConfig::IMU_MONOCULAR:
                return "Mono-Inertial";
            case SensorConfig::IMU_STEREO:
                return "Stereo-Inertial";
            case SensorConfig::RGBD:
                return "RGBD";
            case SensorConfig::EVENT_ONLY:
                return "Event";
            case SensorConfig::EVENT_IMU:
                return "Event-Inertial";
            case SensorConfig::EVENT_MONO:
                return "Event-Image";
            case SensorConfig::EVENT_IMU_MONO:
                return "Event-Image-Inertial";
            default:
                return "Idle";
        }
    }

    bool SensorConfig::isMonocular() const {
        return mSensor == MONOCULAR || mSensor == IMU_MONOCULAR || mSensor == EVENT_ONLY ||
               mSensor == EVENT_MONO || mSensor == EVENT_IMU_MONO || mSensor == EVENT_IMU;
    }

    bool SensorConfig::isStereo() const {
        return mSensor == STEREO || mSensor == IMU_STEREO;
    }

    bool SensorConfig::isRGBD() const {
        return mSensor == RGBD;
    }

    bool SensorConfig::isImage() const {
        return mSensor == MONOCULAR || mSensor == IMU_MONOCULAR || mSensor == STEREO || mSensor == IMU_STEREO ||
               mSensor == RGBD || mSensor == EVENT_MONO || mSensor == EVENT_IMU_MONO;
    }

    bool SensorConfig::isEvent() const {
        return mSensor == EVENT_ONLY || mSensor == EVENT_IMU ||
               mSensor == EVENT_MONO || mSensor == EVENT_IMU_MONO;
    }

    bool SensorConfig::isInertial() const {
        return mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO ||
               mSensor == EVENT_IMU || mSensor == EVENT_IMU_MONO;
    }

    bool SensorConfig::isEventOnly() const {
        return mSensor == EVENT_ONLY;
    }

    bool SensorConfig::isImageOnly() const {
        return mSensor == MONOCULAR || mSensor == IMU_MONOCULAR || mSensor == STEREO || mSensor == IMU_STEREO ||
               mSensor == RGBD;
    }

    bool SensorConfig::operator==(SensorConfig::SensorConfigEnum sConf) {
        return this->mSensor == sConf;
    }

    bool SensorConfig::operator!=(SensorConfig::SensorConfigEnum sConf) {
        return this->mSensor != sConf;
    }

    void SensorConfig::write(cv::FileStorage &fs) const {

        fs << "sensorConfig" << mapConfig(mSensor);
    }

    void SensorConfig::read(const cv::FileNode &fn) {

        string dsConf = YamlParserCV::readString(fn, "sensorConfig",
                                                 SensorConfig::mapConfig(SensorConfig::IDLE));
        mSensor = mapConfig(dsConf);
    }

    void SensorConfig::read(const cv::FileStorage &fs) {

        string dsConf = YamlParserCV::readString(fs, "sensorConfig",
                                                 SensorConfig::mapConfig(SensorConfig::IDLE));
        mSensor = mapConfig(dsConf);
    }

    std::string SensorConfig::printStr(const string &prefix) {
        return prefix + "Sensor Configuration: " + this->toStr() + "\n";
    }


} // NAV24
