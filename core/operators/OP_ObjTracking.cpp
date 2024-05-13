//
// Created by masoud on 5/1/24.
//

//using namespace std;

#include "OP_ObjTracking.hpp"
#include "Message.hpp"

namespace NAV24::OP {

    ObjTracking::ObjTracking() : MsgCallback(), Operator(),
        mqpImages(), mMtxImgQ(), mMtxImg(), bbox(), mLastTs(-1.0) {}

    ObjTracking::ObjTracking(const ChannelPtr& pChannel) : MsgCallback(pChannel), Operator(),
        mqpImages(), mMtxImgQ(), mMtxImg(), bbox(), mLastTs(-1.0) {}

    void ObjTracking::setup(const MsgPtr &configMsg) {

    }

    void ObjTracking::handleRequest(const MsgPtr &reqMsg) {

    }

    void ObjTracking::run() {

        while(true) {
            mMtxImgQ.lock();
            ImagePtr pImage;
            if (!mqpImages.empty()) {
                pImage = mqpImages.front();
                mqpImages.pop();
            }
            mMtxImgQ.unlock();

            this->update(pImage);

            if (this->isStopped()) {
                break;
            }
        }
    }

    cv::Point2f ObjTracking::find_center(const cv::Rect2f &rect) {

        return {rect.x + rect.width * 0.5f, rect.y + rect.height * 0.5f};
    }

    void ObjTracking::init(const MsgPtr &msg) {

    }

    void ObjTracking::update(const ImagePtr &pImage) {

    }

    std::shared_ptr<ObjTracking> ObjTracking::createTracker(const std::string &name, const ChannelPtr &pChannel) {
        return std::shared_ptr<ObjTracking>();
    }
} // NAV24::OP
