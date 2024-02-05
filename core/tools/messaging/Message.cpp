//
// Created by root on 12/21/23.
//

#include "Message.hpp"


namespace NAV24 {

    inline std::string MsgConfig::toString()  {

        if (mConfig)
            return mConfig->printStr("");
        return "";
    }

}