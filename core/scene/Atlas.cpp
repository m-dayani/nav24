//
// Created by masoud on 2/11/24.
//

#include <glog/logging.h>

#include "Atlas.hpp"

using namespace std;

namespace NAV24 {

    Atlas::Atlas(const ChannelPtr& pChannel) : MsgCallback(pChannel), mWorlds() {}

    void Atlas::receive(const MsgPtr &msg) {

        if (msg && msg->getTopic() == Atlas::TOPIC) {
            switch (msg->getTargetId()) {
                case FCN_MAP_CREATE:
                    this->createMap(msg);
                    break;
                case FCN_MAP_ADD_WO:
                    this->addWorldObjects(msg);
                    break;
                default:
                    DLOG(WARNING) << "Atlas::receive, unsupported action\n";
                    break;
            }
        }
    }

    void Atlas::createMap(const MsgPtr &msg) {

        if (msg) {
            string mapName = msg->getMessage();
            MapPtr pMap = make_shared<Map>(mapName);
            mWorlds.insert(make_pair(mapName, pMap));
            mActiveWorld = mapName;
        }
    }

    void Atlas::addWorldObjects(const MsgPtr &msg) {

        if (msg) {
            auto msgData = dynamic_pointer_cast<MsgType<vector<WO::WoPtr>>>(msg);
            if (msgData) {
                string mapName = msg->getMessage();
                if (mapName.empty()) {
                    mapName = mActiveWorld;
                }
                if (mWorlds.count(mapName) <= 0) {
                    DLOG(WARNING) << "Atlas::addWorldObjects, could not find map: " << mapName << "\n";
                    return;
                }
                auto pMap = mWorlds[mapName];
                auto vWobj = msgData->getData();
                for (const auto& wobj : vWobj) {
                    pMap->addWorldObject(wobj);
                }
            }
        }
    }

    void Atlas::setup(const MsgPtr &configMsg) {

    }

    void Atlas::handleRequest(const MsgPtr &reqMsg) {

    }

    void Atlas::run() {

    }


} // NAV24