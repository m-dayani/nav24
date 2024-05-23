//
// Created by masoud on 5/15/24.
//

#ifndef NAV24_FE_OBJTRACKINGROS_HPP
#define NAV24_FE_OBJTRACKINGROS_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>

#include "FE_ObjTracking.hpp"


namespace NAV24::FE {

    class ObjTrackingRos : public ObjTracking {
    public:
        ObjTrackingRos(const ChannelPtr& pChannel);

        void receive(const MsgPtr &msg) override;

        void coordsCallback(const std_msgs::String::ConstPtr& msg);

    protected:
        void handleImageMsg(const MsgPtr &msg) override;

    private:
        image_transport::Publisher imgPublisher;
        ros::Subscriber bboxListener;
    };

} // NAV24::FE

#endif //NAV24_FE_OBJTRACKINGROS_HPP
