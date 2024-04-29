//
// Created by masoud on 2/11/24.
//

#include "FrontEnd.hpp"

#include <utility>

namespace NAV24::FE {
    int FrontEnd::mMapCnt = 0;
    int FrontEnd::mTrajCnt = 0;

    FrontEnd::FrontEnd(ChannelPtr pChannel) : mpChannel(std::move(pChannel)) {}
} // NAV24::FE