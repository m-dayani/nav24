//
// Created by masoud on 2/11/24.
//

#include "FrontEnd.hpp"

#include <utility>

namespace NAV24::FE {
    FrontEnd::FrontEnd(ChannelPtr pChannel) : mpChannel(std::move(pChannel)) {}
} // NAV24::FE