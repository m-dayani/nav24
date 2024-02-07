//
// Created by masoud on 2/6/24.
//

#include "Sensor.hpp"

#include <glog/logging.h>

using namespace std;

namespace NAV24 {

#define PARAM_KEY_SENSOR_NAME "name"
#define PARAM_KEY_SENSOR_INTERFACE "interface"
#define PARAM_KEY_SENSOR_IF_TYPE "type"
#define PARAM_KEY_SENSOR_IF_TARGET "target"
#define PARAM_KEY_SENSOR_IF_PORT "port"

    void Sensor::receive(const NAV24::MsgPtr &msg) {

        if (!msg) {
            DLOG(WARNING) << "Sensor::receive, null parameter detected, abort\n";
            return;
        }

        if (dynamic_pointer_cast<MsgConfig>(msg)) {
            this->loadParams(msg);
        }
    }

    void Sensor::loadParams(const MsgPtr &msg) {

        if (!msg) {
            DLOG(WARNING) << "Sensor::loadParams, null parameter detected, abort\n";
            return;
        }

        auto pMsgConfig = dynamic_pointer_cast<MsgConfig>(msg);
        if (!pMsgConfig) {
            DLOG(WARNING) << "Sensor::loadParams, wrong message type, abort\n";
            return;
        }

        auto pSensorParams = pMsgConfig->getConfig();
        if (pSensorParams) {
            // Sensor name
            auto pSensorName = find_param<ParamType<string>>(PARAM_KEY_SENSOR_NAME, pSensorParams);
            if (pSensorName) {
                mName = pSensorName->getValue();
            }

            // Sensor interface
            auto pSensorInterface = pSensorParams->read(PARAM_KEY_SENSOR_INTERFACE);
            if (pSensorInterface) {
                SensorInterface::InterfaceType ifType = SensorInterface::DEFAULT;
                string ifTarget;
                int ifPort = 0;

                auto pSensorIfType = find_param<ParamType<string>>(PARAM_KEY_SENSOR_IF_TYPE, pSensorInterface);
                if (pSensorIfType) {
                    string sensorType = pSensorIfType->getValue();
                    if (sensorType == "mixed") {
                        ifType = SensorInterface::MIXED;
                    }
                    else if (sensorType == "storage") {
                        ifType = SensorInterface::STORAGE;
                    }
                    else if (sensorType == "stream") {
                        ifType = SensorInterface::STREAM;
                    }
                }

                auto pSensorIfTarget = find_param<ParamType<string>>(PARAM_KEY_SENSOR_IF_TARGET, pSensorInterface);
                if (pSensorIfTarget) {
                    ifTarget = pSensorIfTarget->getValue();
                }

                auto pSensorIfPort = find_param<ParamType<int>>(PARAM_KEY_SENSOR_IF_PORT, pSensorInterface);
                if (pSensorIfPort) {
                    ifPort = pSensorIfPort->getValue();
                }

                mpInterface = make_shared<SensorInterface>(ifType, ifTarget, ifPort);
            }
        }
    }

    std::string Sensor::printStr(const string &prefix) const {

        ostringstream oss;

        oss << prefix << "Sensor Name: " << mName << "\n";
        oss << prefix << "Interface: " << mpInterface->printStr() << "\n";

        return oss.str();
    }

    std::string SensorInterface::printStr(const string &prefix) const {

        ostringstream oss;

        oss << prefix << "Type: " << interfaceType << "\n";
        oss << prefix << "Target: " << target << "\n";
        oss << prefix << "Port: " << port << "\n";

        return oss.str();
    }
}