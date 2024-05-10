//
// Created by root on 12/21/23.
//

#ifndef NAV24_INTERFACE_HPP
#define NAV24_INTERFACE_HPP

#include <memory>
#include <string>

#include "Message.hpp"


namespace NAV24 {

#define IC_DEF_PORT

    struct SensorInterface {
        enum InterfaceType {
            DEFAULT,
            OFFLINE,
            STREAM,
            MIXED
        };

        SensorInterface() : interfaceType(DEFAULT), target(), port(IC_DEF_PORT) {}
        SensorInterface(InterfaceType intType, std::string target_, int port_) :
                interfaceType(intType), target(std::move(target_)), port(port_) {}

        [[nodiscard]] std::string printStr(const std::string& prefix = "") const;

        InterfaceType interfaceType;
        std::string target;
        int port;
    };
} // NAV24

#endif //NAV24_INTERFACE_HPP
