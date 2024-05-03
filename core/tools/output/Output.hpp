//
// Created by masoud on 4/30/24.
//

#ifndef NAV24_OUTPUT_HPP
#define NAV24_OUTPUT_HPP

#include "Message.hpp"


namespace NAV24 {
    class Output : public MsgCallback {
    public:
        inline static const std::string TOPIC = "Output";
    };
    typedef std::shared_ptr<Output> OutputPtr;
} // NAV24

#endif //NAV24_OUTPUT_HPP
