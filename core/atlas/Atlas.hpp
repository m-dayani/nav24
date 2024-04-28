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

    class Atlas : public MsgCallback {
    public:
        explicit Atlas(ChannelPtr pChannel);
        void receive(const MsgPtr &msg) override;

    protected:
        ChannelPtr mpChannel;

        std::map<std::string, Map> mWorlds;
        std::string mActiveWorld;
    };
    typedef std::shared_ptr<Atlas> AtlasPtr;

} // NAV24

#endif //NAV24_ATLAS_HPP
