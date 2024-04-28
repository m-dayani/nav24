//
// Created by masoud on 2/11/24.
//

#include "Atlas.hpp"

namespace NAV24 {

    Atlas::Atlas(ChannelPtr pChannel) : mpChannel(std::move(pChannel)) {

    }

    void Atlas::receive(const MsgPtr &msg) {

    }


} // NAV24