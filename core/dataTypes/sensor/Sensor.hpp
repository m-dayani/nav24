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
#include "Interface.hpp"


namespace NAV24 {

#define FCN_SEN_PRINT 3
#define FCN_SEN_STOP_PLAY 5
#define FCN_SEN_START_PLAY 6
#define FCN_SEN_GET_NEXT 8
#define FCN_SEN_RESET 10
#define FCN_SEN_CONFIG 12

#define TAG_SEN_MX_OFFLINE "offline"
#define TAG_SEN_MX_STREAM "stream"
#define TAG_SEN_MX_BOTH "both"


    class Sensor : public MsgCallback {
    public:
        inline static const std::string TOPIC = "Sensor";

        Sensor() : mpInterface() {}
        explicit Sensor(const ChannelPtr& pChannel);

        // All sensors are nodes and at least handle config requests
        void receive(const MsgPtr &msg) override;

    protected:
        void setup(const MsgPtr &msg) override;
        //void handleRequest(const MsgPtr &reqMsg) override;

        virtual void getNext(MsgPtr pReq) = 0;
        //void run() override;
        //void stop() override;

        virtual void reset() = 0;

        [[nodiscard]] virtual std::string printStr(const std::string& prefix) const;

    protected:
        // All sensors have a name and interface
        // All sensors have means of communication with other modules
        std::shared_ptr<SensorInterface> mpInterface;
    };

}   //NAV24

#endif //NAV24_SENSOR_HPP
