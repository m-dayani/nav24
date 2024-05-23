//
// Created by masoud on 5/15/24.
//

#include <iostream>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "FE_ObjTrackingRos.hpp"
#include "System.hpp"

using namespace std;

namespace NAV24::FE {

    ObjTrackingRos::ObjTrackingRos(const NAV24::ChannelPtr &pChannel) : ObjTracking(pChannel) {

    }

    void ObjTrackingRos::receive(const MsgPtr &msg) {
        ObjTracking::receive(msg);

        if (msg) {
            if (std::dynamic_pointer_cast<MsgType<ros::NodeHandle>>(msg)) {
                auto nh = std::dynamic_pointer_cast<MsgType<ros::NodeHandle>>(msg)->getData();

                bboxListener = nh.subscribe("/obj/coords", 10, &ObjTrackingRos::coordsCallback, this);

                image_transport::ImageTransport it(nh);
                imgPublisher = it.advertise("/camera/image", 1);
            }
        }
    }

    void ObjTrackingRos::handleImageMsg(const MsgPtr &msg) {

        if (msg && std::dynamic_pointer_cast<MsgSensorData>(msg)) {
            auto pImage = std::dynamic_pointer_cast<MsgSensorData>(msg)->getData();
            if (pImage && std::dynamic_pointer_cast<ImageTs>(pImage)) {
                auto pImg = std::dynamic_pointer_cast<ImageTs>(pImage);
                cv::Mat image = pImg->mImage.clone();

                // make the image message
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

                // publish
                imgPublisher.publish(msg);
                // do this in the main()
                //ros::spinOnce();
            }
        }

        // call the rest of the program
        ObjTracking::handleImageMsg(msg);
    }

    void ObjTrackingRos::coordsCallback(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO("I heard: [%s]", msg->data.c_str());
        //std::cout << "I heard: " << msg->data.c_str() << std::endl;
    }

} // NAV24::FE