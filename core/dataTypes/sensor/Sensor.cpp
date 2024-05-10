//
// Created by masoud on 2/6/24.
//

#include <iostream>
#include <glog/logging.h>

#include "Sensor.hpp"
#include "System.hpp"
#include "ParameterBlueprint.h"


using namespace std;


namespace NAV24 {

    Sensor::Sensor(const ChannelPtr &pChannel) : MsgCallback(pChannel), mpInterface() {
        DLOG(INFO) << "Sensor::Sensor, Constructor\n";
    }

    void Sensor::receive(const NAV24::MsgPtr &msg) {

        if (!msg) {
            DLOG(WARNING) << "Sensor::receive, null parameter detected, abort\n";
            return;
        }

        if (dynamic_pointer_cast<MsgConfig>(msg)) {
            this->setup(msg);
            return;
        }

        const int action = msg->getTargetId();

        if (msg->getTopic() == Sensor::TOPIC) {
            switch (action) {
                case FCN_SEN_STOP_PLAY:
                    this->stop();
                    break;
                case FCN_SEN_START_PLAY:
                    this->run();
                    break;
                case FCN_SEN_GET_NEXT:
                    this->getNext(msg);
                    break;
                case FCN_SEN_RESET:
                    this->reset();
                    break;
                case FCN_SEN_PRINT: {
                    string strStat = this->printStr("");
                    // todo: send a return message to sender
                    cout << strStat << endl;
                }
                    break;
                default:
                    DLOG(INFO) << "Sensor::receive, action is not supported: " << action << "\n";
                    break;
            }
        }

        if (action == FCN_SYS_STOP) {
            this->stop();
        }
    }

    void Sensor::setup(const MsgPtr &msg) {

        if (!msg) {
            DLOG(WARNING) << "Sensor::setup, null parameter detected, abort\n";
            return;
        }

        auto pMsgConfig = dynamic_pointer_cast<MsgConfig>(msg);
        if (!pMsgConfig) {
            DLOG(WARNING) << "Sensor::setup, wrong message type, abort\n";
            return;
        }

        auto pSensorParams = pMsgConfig->getConfig();
        if (pSensorParams) {
            // Sensor name
            auto pSensorName = find_param<ParamType<string>>(PKEY_NAME, pSensorParams);
            if (pSensorName) {
                mName = pSensorName->getValue();
            }

            // Sensor interface
            auto pSensorInterface = pSensorParams->read(PKEY_INTERFACE);
            if (pSensorInterface) {
                SensorInterface::InterfaceType ifType = SensorInterface::DEFAULT;
                string ifTarget;
                int ifPort = 0;

                auto pSensorIfType = find_param<ParamType<string>>(PKEY_IF_TYPE, pSensorInterface);
                if (pSensorIfType) {
                    string sensorType = pSensorIfType->getValue();
                    if (sensorType == "mixed") {
                        ifType = SensorInterface::MIXED;
                    }
                    else if (sensorType == "offline") {
                        ifType = SensorInterface::OFFLINE;
                    }
                    else if (sensorType == "stream") {
                        ifType = SensorInterface::STREAM;
                    }
                }

                auto pSensorIfTarget = find_param<ParamType<string>>(PKEY_IF_TARGET, pSensorInterface);
                if (pSensorIfTarget) {
                    ifTarget = pSensorIfTarget->getValue();
                }

                auto pSensorIfPort = find_param<ParamType<int>>(PKEY_IF_PORT, pSensorInterface);
                if (pSensorIfPort) {
                    ifPort = pSensorIfPort->getValue();
                }

                mpInterface = make_shared<SensorInterface>(ifType, ifTarget, ifPort);
            }
        }
    }

    std::string Sensor::printStr(const string &prefix) const {

        ostringstream oss;

        string pref = " ";
        if (!prefix.empty()) {
            pref = prefix[0];
            pref += pref;
        }
        oss << prefix << "Sensor Name: " << mName << "\n";
        oss << prefix << "Interface: \n" << mpInterface->printStr(pref);

        return oss.str();
    }

    /*void Sensor::handleRequest(const MsgPtr &reqMsg) {}

    void Sensor::run() {}

    void Sensor::stop() {
        MsgCallback::stop();
    }*/

    std::string SensorInterface::printStr(const string &prefix) const {

        ostringstream oss;

        oss << prefix << "Type: " << interfaceType << "\n";
        oss << prefix << "Target: " << target << "\n";
        oss << prefix << "Port: " << port << "\n";

        return oss.str();
    }
}