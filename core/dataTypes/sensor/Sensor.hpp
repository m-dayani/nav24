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

//#define FCN_SEN_REQ 1
#define FCN_SEN_PRINT 3
#define FCN_SEN_STOP_PLAY 5
#define FCN_SEN_START_PLAY 6
#define FCN_SEN_GET_NEXT 8
#define FCN_SEN_RESET 10

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
        inline static const std::string TOPIC = "Sensor";

        Sensor() : mName(), mpInterface(), mbIsStopped(true), mpChannel() {}
        explicit Sensor(ChannelPtr pChannel) :
                mName(), mpInterface(), mbIsStopped(true), mpChannel(std::move(pChannel)) {}

        // All sensors are nodes and at least handle config requests
        void receive(const MsgPtr &msg) override;

    protected:
        virtual void loadParams(const MsgPtr &msg);

        virtual void getNext(MsgPtr pReq) = 0;
        virtual void play() = 0;

        virtual void reset() = 0;

        [[nodiscard]] virtual std::string printStr(const std::string& prefix) const;

        // All sensors have a name and interface
        std::string mName;
        std::shared_ptr<SensorInterface> mpInterface;

        bool mbIsStopped;

        // All sensors have means of communication with other modules
        ChannelPtr mpChannel;
    };

}   //NAV24

#endif //NAV24_SENSOR_HPP
