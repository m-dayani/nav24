//
// Created by root on 12/21/23.
//

#include "System.hpp"

using namespace std;

namespace NAV24 {

    System::System(const std::string &settings) : mmChannels{} {

    }

    void System::publish(const MsgPtr &message) {

        for (auto channel : mmChannels[message->getTopic()]) {
            channel->receive(message);
        }
    }

    void System::registerChannel(const MsgCbPtr &callback, const string &topic) {

        if (mmChannels.count(topic) <= 0) {
            mmChannels[topic] = vector<MsgCbPtr>();
        }
        mmChannels[topic].push_back(callback);
    }

    void System::unregisterChannel(const MsgCbPtr &callback, const string &topic) {

    }

    void System::receive(const MsgPtr &msg) {

    }


}