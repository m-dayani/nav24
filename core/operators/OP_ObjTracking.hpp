//
// Created by masoud on 5/1/24.
//

#ifndef NAV24_OP_OBJTRACKING_HPP
#define NAV24_OP_OBJTRACKING_HPP

#include <memory>
#include <string>
#include <vector>

#include "Message.hpp"
#include "Interface.hpp"
#include "Operator.hpp"
#include "Sensor.hpp"


namespace NAV24::OP {

#define FCN_OBJ_TR_RUN 4
#define FCN_OBJ_TR_STOP 6

    class ObjTracking : public MsgCallback, public Operator {
    public:
        inline static const std::string TOPIC = "ObjTracking";
    protected:
        void setup(const MsgPtr &configMsg) override;

        void handleRequest(const MsgPtr &reqMsg) override;

        void run() override;
    };
} // NAV24::OP

#endif //NAV24_OP_OBJTRACKING_HPP
