//
// Created by masoud on 4/30/24.
//

#ifndef NAV24_SERIAL_HPP
#define NAV24_SERIAL_HPP

#include <queue>
#include <serial/serial.h>

#include "Output.hpp"
#include "Interface.hpp"
#include "Sensor.hpp"


namespace NAV24 {

#define SER_DEF_BAUD 9600
#define SER_DEF_PORT "/dev/ttyUSB0"

#define FCN_SER_OPEN 6
#define FCN_SER_READ 11
#define FCN_SER_WRITE 3

    class Serial : public Output {
    public:
        inline static const std::string TOPIC = "Serial";

        explicit Serial(const ChannelPtr&  pChannel);

        void receive(const MsgPtr &msg) override;

    protected:
        void setup(const MsgPtr& msg) override;

        void run() override;
        void requestStop(const std::string &channel) override;
        void stop() override;

    protected:
        std::shared_ptr<serial::Serial> mpSerial;

        std::queue<std::string> mReadBuffer;
        std::mutex mMtxReadBuff;
        std::queue<std::string> mWriteBuffer;
        std::mutex mMtxWriteBuff;
    };
} // NAV24

#endif //NAV24_SERIAL_HPP
