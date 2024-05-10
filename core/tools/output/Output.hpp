//
// Created by masoud on 4/30/24.
//

#ifndef NAV24_OUTPUT_HPP
#define NAV24_OUTPUT_HPP

#include "Message.hpp"
#include "Interface.hpp"
#include "Sensor.hpp"


namespace NAV24 {
    class Output : public MsgCallback {
    public:
        inline static const std::string TOPIC = "Output";

        explicit Output(const ChannelPtr&  pChannel);

        void receive(const MsgPtr &msg) override;

        static std::shared_ptr<Output> getNewInstance(const ParamPtr& pParam, const ChannelPtr& pChannel);
    protected:
        void setup(const MsgPtr& msg) override;
        void handleRequest(const MsgPtr& msg) override;

        //virtual void run() = 0;
        virtual void requestStop(const std::string& channel) = 0;

    protected:
        std::shared_ptr<SensorInterface> mpInterface;
    };
    typedef std::shared_ptr<Output> OutputPtr;
} // NAV24

#endif //NAV24_OUTPUT_HPP
