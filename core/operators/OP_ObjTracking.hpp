//
// Created by masoud on 5/1/24.
//

#ifndef NAV24_OP_OBJTRACKING_HPP
#define NAV24_OP_OBJTRACKING_HPP

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Message.hpp"
#include "Interface.hpp"
#include "Operator.hpp"
#include "Sensor.hpp"


namespace NAV24::OP {

#define FCN_OBJ_TR_RUN 4
#define FCN_OBJ_TR_STOP 6

    class ObjTracking : public MsgCallback, public Operator {
    public:
        inline static const std::string TOPIC = "ObjTracking";

        ObjTracking() : MsgCallback(), Operator(), mpChannel() {}
        explicit ObjTracking(ChannelPtr  pChannel) : MsgCallback(), Operator(), mpChannel(std::move(pChannel)) {}

        static cv::Point2f find_center(const cv::Rect2f& rect);

    protected:
        void setup(const MsgPtr &configMsg) override;
        void handleRequest(const MsgPtr &reqMsg) override;
        void run() override;

    protected:
        ChannelPtr mpChannel;
    };
} // NAV24::OP

#endif //NAV24_OP_OBJTRACKING_HPP
