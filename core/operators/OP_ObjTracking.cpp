//
// Created by masoud on 5/1/24.
//

//using namespace std;

#include "OP_ObjTracking.hpp"
#include "Message.hpp"

namespace NAV24::OP {

    void ObjTracking::setup(const MsgPtr &configMsg) {

    }

    void ObjTracking::handleRequest(const MsgPtr &reqMsg) {

    }

    void ObjTracking::run() {

    }

    cv::Point2f ObjTracking::find_center(const cv::Rect2f &rect) {

        return {rect.x + rect.width * 0.5f, rect.y + rect.height * 0.5f};
    }
} // NAV24::OP
