//
// Created by root on 12/21/23.
//

#include "Interface.hpp"
#include "ParameterBlueprint.h"

using namespace std;

namespace NAV24 {

    SensorInterface::SensorInterface(const ParamPtr &pSensorParams) : SensorInterface() {

        if (pSensorParams) {

            // Sensor interface
            auto pSensorInterface = pSensorParams->read(PKEY_INTERFACE);
            if (pSensorInterface) {
                SensorInterface::InterfaceType ifType = SensorInterface::DEFAULT;
                string ifTarget;
                int ifPort = 0;

                auto pSensorIfType = find_param<ParamType<string>>(PKEY_IF_TYPE, pSensorInterface);
                if (pSensorIfType) {
                    typeStr = pSensorIfType->getValue();
                    if (typeStr == "mixed") {
                        interfaceType = SensorInterface::MIXED;
                    }
                    else if (typeStr == "offline") {
                        interfaceType = SensorInterface::OFFLINE;
                    }
                    else if (typeStr == "stream") {
                        interfaceType = SensorInterface::STREAM;
                    }
                }

                auto pSensorIfTarget = find_param<ParamType<string>>(PKEY_IF_TARGET, pSensorInterface);
                if (pSensorIfTarget) {
                    target = pSensorIfTarget->getValue();
                }

                auto pSensorIfPort = find_param<ParamType<int>>(PKEY_IF_PORT, pSensorInterface);
                if (pSensorIfPort) {
                    port = pSensorIfPort->getValue();
                }

//                mpInterface = make_shared<SensorInterface>(ifType, ifTarget, ifPort);
            }
        }
    }

} // NAV24