//
// Created by masoud on 5/1/24.
//

#ifndef NAV24_OP_OBJTRACKING_HPP
#define NAV24_OP_OBJTRACKING_HPP

#include <memory>
#include <string>
#include <vector>

#include "Message.hpp"
#include "Channel.hpp"
#include "Operator.hpp"
#include "Sensor.hpp"


namespace NAV24::OP {
    class ObjTracking : public MsgCallback, public Operator {

    };
} // NAV24::OP

#endif //NAV24_OP_OBJTRACKING_HPP
