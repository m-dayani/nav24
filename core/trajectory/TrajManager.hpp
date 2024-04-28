//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_TRAJMANAGER_HPP
#define NAV24_TRAJMANAGER_HPP

#include <memory>

#include "Message.hpp"
#include "Channel.hpp"


namespace NAV24 {

    class TrajManager : public MsgCallback {
    public:
        void receive(const MsgPtr &msg) override;

    protected:
        ChannelPtr mpChannel;
    };
    typedef std::shared_ptr<TrajManager> TrajManagerPtr;

} // NAV24

#endif //NAV24_TRAJMANAGER_HPP
