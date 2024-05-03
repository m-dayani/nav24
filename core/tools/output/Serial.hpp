//
// Created by masoud on 4/30/24.
//

#ifndef NAV24_SERIAL_HPP
#define NAV24_SERIAL_HPP

#include <serial/serial.h>

#include "Output.hpp"
#include "Channel.hpp"
#include "Sensor.hpp"


namespace NAV24 {

#define SER_DEF_BAUD 9600

#define FCN_SER_OPEN 6
#define FCN_SER_READ 11
#define FCN_SER_WRITE 3

    class Serial : public Output {
    public:
        inline static const std::string TOPIC = "Serial";

        explicit Serial(ChannelPtr  pChannel);

        void receive(const MsgPtr &msg) override;

    protected:
        void initialize(const MsgPtr& msg);

        std::string mName;
        ChannelPtr mpChannel;
        std::shared_ptr<SensorInterface> mpInterface;
        std::shared_ptr<serial::Serial> mpSerial;
    };
} // NAV24

#endif //NAV24_SERIAL_HPP
