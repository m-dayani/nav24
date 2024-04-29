//
// Created by root on 12/21/23.
//

#include "Message.hpp"
#include "Map.hpp"


namespace NAV24 {

    inline std::string MsgConfig::toString()  {

        if (mConfig)
            return mConfig->printStr("");
        return "";
    }

    //std::string MsgType<MapPtr>::toString();
}