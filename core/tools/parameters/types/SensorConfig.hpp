//
// Created by root on 5/10/21.
//

#ifndef NAV24_SENSORCONFIG_HPP
#define NAV24_SENSORCONFIG_HPP

#include <memory>
#include <string>

#include "Parameter.hpp"
#include "YamlParserCV.hpp"


namespace NAV24 {

    class SensorConfig : public Parameter {
    public:
        inline static const std::string TAG{"SensorConfig"};

        enum SensorConfigEnum {
            MONOCULAR = 0,
            STEREO = 1,
            RGBD = 2,
            IMU_MONOCULAR = 3,
            IMU_STEREO = 4,
            EVENT_ONLY = 5,
            EVENT_MONO = 6,
            EVENT_IMU = 7,
            EVENT_IMU_MONO = 8,
            IDLE = 9
        };

        SensorConfig() : mSensor(IDLE) {}
        explicit SensorConfig(SensorConfigEnum sConf) : mSensor(sConf) {}
        explicit SensorConfig(const cv::FileStorage& fs);
        explicit SensorConfig(const cv::FileNode& dsNode);

        static SensorConfigEnum mapConfig(const std::string &dsConfig);
        static std::string mapConfig(SensorConfigEnum dsConfig);

        // Convert Sensor config. types
        std::string toStr() const;
        std::string toDsStr() const;

        void write(cv::FileStorage& fs) const override;
        void read(const cv::FileNode& node) override;
        void read(const cv::FileStorage& fs);
        std::string printStr(const std::string& prefix = "") override;

        // Sensor configurations:
        // 3 camera config.
        bool isMonocular() const;
        bool isStereo() const;
        bool isRGBD() const;
        // 3 types of sensors
        bool isImage() const;
        bool isEvent() const;
        bool isInertial() const;
        bool isEventOnly() const;
        bool isImageOnly() const;

        bool operator==(SensorConfigEnum sConf);
        bool operator!=(SensorConfigEnum sConf);

        SensorConfigEnum getConfig() const { return this->mSensor; }
        void setConfig(SensorConfigEnum sConf) { this->mSensor = sConf; }

    private:
        SensorConfigEnum mSensor;

    };

    typedef std::shared_ptr<SensorConfig> SensorConfigPtr;

    //These write and read functions are required by OpenCV
    static void write(cv::FileStorage& fs, const std::string&, const SensorConfig& sConf) {

        sConf.write(fs);
    }
    static void read(const cv::FileNode& node, SensorConfig& sConf, const SensorConfig& default_value = SensorConfig()) {

        if(node.empty())
            sConf.setConfig(default_value.getConfig());
        else
            sConf.read(node);
    }

} // NAV24


#endif //NAV24_SENSORCONFIG_HPP
