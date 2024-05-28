//
// Created by masoud on 5/15/24.
//


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include "FE_ObjTrackingRos.hpp"
#include "System.hpp"
#include "Point2D.hpp"

using namespace std;

namespace NAV24 { namespace FE {

    ObjTrackingRos::ObjTrackingRos(const NAV24::ChannelPtr &pChannel) : ObjTracking(pChannel) {

        mpYoloDetector = make_shared<OP::ObjTrYoloOnnx>(pChannel);
    }

    void ObjTrackingRos::receive(const MsgPtr &msg) {
        ObjTracking::receive(msg);

        if (msg) {
            if (std::dynamic_pointer_cast<MsgType<ros::NodeHandle>>(msg)) {
                auto nh = std::dynamic_pointer_cast<MsgType<ros::NodeHandle>>(msg)->getData();

                bboxListener = nh.subscribe("/obj/coords", 10, &ObjTrackingRos::coordsCallback, this);

                image_transport::ImageTransport it(nh);
                imgPublisher = it.advertise("/camera/image", 1);

                if (!mbInitialized) {
                    this->setup(msg);
                }
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
                sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
                imgMsg->header.stamp.fromNSec(static_cast<uint64_t>(pImg->mTimeStamp));
                imgMsg->header.frame_id = pImg->mPath;

                // publish
                imgPublisher.publish(imgMsg);
                // do this in the main()
                //ros::spinOnce();
            }
        }

        // call the rest of the program
        ObjTracking::handleImageMsg(msg);
    }

    void ObjTrackingRos::coordsCallback(const std_msgs::String::ConstPtr& msg) {
        //ROS_INFO("I heard: [%s]", msg->data.c_str());
        //std::cout << "I heard: " << msg->data.c_str() << std::endl;
        long ts_img = -1;
        string the_rest;
        istringstream iss{msg->data};
        iss >> ts_img;
        getline(iss, the_rest);
        // todo: do something with ts
        auto t1 = chrono::time_point_cast<chrono::nanoseconds>(chrono::system_clock::now());
        double t_diff = static_cast<double>(t1.time_since_epoch().count() - ts_img) * 1e-9;
        //cout << "time diff: " << t_diff << "\n";
        //this->updateBboxAndLastPoint(the_rest);
        shared_ptr<OB::BBox> pBbox = make_shared<OB::BBox>();
        pBbox->updateBboxAndLastPoint(the_rest);

        // let the top level front-end handle this
        auto pMsgObs = make_shared<MsgType<OB::ObsTimed>>(ID_CH_FE,
                make_pair(ts_img, pBbox), ObjTracking::TOPIC);
        this->receive(pMsgObs);
        //this->correctObservation(make_pair(t_diff, pBbox));
        mTsYoloUpdate = ts_img;
        //ROS_INFO("(%f, %f, %f, %f)", mYoloDet.x, mYoloDet.y, mYoloDet.width, mYoloDet.height);
    }

} } // NAV24::FE