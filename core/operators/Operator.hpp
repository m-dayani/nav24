//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_OPERATOR_HPP
#define NAV24_OPERATOR_HPP

#include <string>

#include "Message.hpp"


namespace NAV24::OP {

    class Operator : public MsgCallback {
    public:
        inline static const std::string TOPIC = "OP::Operator";

        Operator() : MsgCallback() {}
        explicit Operator(const ChannelPtr& pChannel) : MsgCallback(pChannel) {}

        void receive(const MsgPtr &msg) override;

    protected:
        void setup(const MsgPtr &configMsg) override;

        void handleRequest(const MsgPtr &reqMsg) override;

        void run() override;

    };

} // NAV24::OP

#endif //NAV24_OPERATOR_HPP
