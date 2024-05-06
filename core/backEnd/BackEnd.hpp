//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_BACKEND_HPP
#define NAV24_BACKEND_HPP

#include "Message.hpp"
#include "Problem.hpp"

namespace NAV24::BE {

    class BackEnd : public MsgCallback {
    public:
        virtual void solve(const ProblemPtr& problem) = 0;

        void receive(const MsgPtr &msg) override;

    protected:
        void setup(const MsgPtr &configMsg) override;

        void handleRequest(const MsgPtr &reqMsg) override;

        void run() override;
    };

} // NAV24::BE

#endif //NAV24_BACKEND_HPP
