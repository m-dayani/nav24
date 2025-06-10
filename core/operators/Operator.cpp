//
// Created by masoud on 2/11/24.
//

#include <memory>

#include "Operator.hpp"


namespace NAV24::OP {
    void Operator::receive(const MsgPtr &msg) {
        if (msg) {
            if (std::dynamic_pointer_cast<MsgConfig>(msg)) {
                this->setup(msg);
            }
        }
    }

    void Operator::setup(const MsgPtr &) {

    }

    void Operator::handleRequest(const MsgPtr &) {

    }

    void Operator::run() {

    }
} // NAV24::OP