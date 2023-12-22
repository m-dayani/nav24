//
// Created by root on 12/21/23.
//

#ifndef NAV24_CHANNEL_HPP
#define NAV24_CHANNEL_HPP

#include <memory>
#include <string>

#include "Message.hpp"


namespace NAV24 {

    class Channel;
    typedef std::shared_ptr<Channel> ChannelPtr;
    class Channel {
    public:
        virtual void publish(const MsgPtr& message) = 0;
        //virtual void receive(const Message& message) = 0; -> this is implemented by the request class
        virtual void registerChannel(const MsgCbPtr& callback, const std::string& topic) = 0;
        virtual void unregisterChannel(const MsgCbPtr& callback, const std::string& topic) = 0;
    };

}

#endif //NAV24_CHANNEL_HPP
