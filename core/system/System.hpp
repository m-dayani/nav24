//
// Created by root on 12/21/23.
//

#ifndef NAV24_SYSTEM_HPP
#define NAV24_SYSTEM_HPP

#include <string>
#include <vector>
#include <map>

#include "Channel.hpp"


namespace NAV24 {

    class System : public Channel, public MsgCallback {
    public:
        System(const std::string& settings);

        void publish(const MsgPtr &message) override;

        void registerChannel(const MsgCbPtr& callback, const std::string &topic) override;

        void unregisterChannel(const MsgCbPtr& callback, const std::string &topic) override;

        void receive(const MsgPtr &msg) override;

    protected:
        std::map<std::string, std::vector<MsgCbPtr>> mmChannels;
    };

}

#endif //NAV24_SYSTEM_HPP
