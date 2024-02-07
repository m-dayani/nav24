//
// Created by masoud on 2/6/24.
//

#ifndef NAV24_SENSOR_HPP
#define NAV24_SENSOR_HPP

#include <string>
#include <utility>
#include <vector>
#include <memory>

#include "Message.hpp"
#include "Channel.hpp"


namespace NAV24 {

    struct SensorInterface {
        enum InterfaceType {
            DEFAULT,
            STORAGE,
            STREAM,
            MIXED
        };

        SensorInterface() : interfaceType(DEFAULT), target(), port(0) {}
        SensorInterface(InterfaceType intType, std::string target_, int port_) :
            interfaceType(intType), target(std::move(target_)), port(port_) {}

        [[nodiscard]] std::string printStr(const std::string& prefix = "") const;

        InterfaceType interfaceType;
        std::string target;
        int port;
    };

    class Sensor : public MsgCallback {
    public:
        Sensor() : mName(), mpInterface(), mpChannel() {}
        explicit Sensor(ChannelPtr pChannel) : mName(), mpInterface(), mpChannel(std::move(pChannel)) {}

        // All sensors are nodes and at least handle config requests
        void receive(const MsgPtr &msg) override;

    protected:
        virtual void loadParams(const MsgPtr &msg);
        [[nodiscard]] virtual std::string printStr(const std::string& prefix) const;

        // All sensors have a name and interface
        std::string mName;
        std::shared_ptr<SensorInterface> mpInterface;

        // All sensors have means of communication with other modules
        ChannelPtr mpChannel;
    };

}   //NAV24

#endif //NAV24_SENSOR_HPP
