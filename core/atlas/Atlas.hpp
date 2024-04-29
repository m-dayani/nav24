//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_ATLAS_HPP
#define NAV24_ATLAS_HPP

#include <memory>
#include <map>

#include "Message.hpp"
#include "Channel.hpp"
#include "Map.hpp"

namespace NAV24 {

#define FCN_MAP_CREATE 2
#define FCN_MAP_ADD_WO 5

    class Atlas : public MsgCallback {
    public:
        inline static const std::string TOPIC = "Atlas";

        explicit Atlas(ChannelPtr pChannel);
        void receive(const MsgPtr &msg) override;

    protected:
        void createMap(const MsgPtr &msg);
        void addWorldObjects(const MsgPtr& msg);

    protected:
        ChannelPtr mpChannel;

        std::map<std::string, MapPtr> mWorlds;
        std::string mActiveWorld;
    };
    typedef std::shared_ptr<Atlas> AtlasPtr;

} // NAV24

#endif //NAV24_ATLAS_HPP
