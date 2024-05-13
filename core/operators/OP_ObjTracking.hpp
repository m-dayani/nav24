//
// Created by masoud on 5/1/24.
//

#ifndef NAV24_OP_OBJTRACKING_HPP
#define NAV24_OP_OBJTRACKING_HPP

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <queue>

#include "Message.hpp"
#include "Interface.hpp"
#include "Operator.hpp"
#include "Sensor.hpp"
#include "Image.hpp"


namespace NAV24::OP {

#define NAME_OBJ_TR_CV "tracker-cv"
#define NAME_OBJ_TR_YOLO "tracker-yolo-onnx"

#define FCN_OBJ_TR_RUN 4
#define FCN_OBJ_TR_STOP 6

    class ObjTracking : public MsgCallback, public Operator {
    public:
        inline static const std::string TOPIC = "ObjTracking";

        ObjTracking();
        explicit ObjTracking(const ChannelPtr& pChannel);

        static std::shared_ptr<ObjTracking> createTracker(const std::string& name, const ChannelPtr& pChannel);

        static cv::Point2f find_center(const cv::Rect2f& rect);

    protected:
        void setup(const MsgPtr &configMsg) override;
        void handleRequest(const MsgPtr &reqMsg) override;
        void run() override;

        // Initialize internal tracker objects
        virtual void init(const MsgPtr& msg);
        virtual void update(const ImagePtr& pImage);

    protected:
        std::queue<ImagePtr> mqpImages;
        std::mutex mMtxImgQ;
        std::mutex mMtxImg;

        cv::Rect2d bbox;
        double mLastTs;
    };
} // NAV24::OP

#endif //NAV24_OP_OBJTRACKING_HPP
