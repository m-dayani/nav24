//
// Created by masoud on 2/11/24.
//

#include "FrontEnd.hpp"


namespace NAV24::FE {
    int FrontEnd::mMapCnt = 0;
    int FrontEnd::mTrajCnt = 0;

    FrontEnd::FrontEnd(const ChannelPtr& pChannel) : MsgCallback(pChannel) {}

    void FrontEnd::handleRequest(const MsgPtr &reqMsg) {

    }

    void FrontEnd::run() {

    }
} // NAV24::FE