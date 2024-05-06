//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_FRONTEND_HPP
#define NAV24_FRONTEND_HPP

#include "Message.hpp"
#include "Interface.hpp"

namespace NAV24::FE {

    class FrontEnd : public MsgCallback {
    public:
        inline static const std::string TOPIC = "FrontEnd";
        explicit FrontEnd(ChannelPtr  pChannel);

    protected:
        //virtual void setup(const MsgPtr& msg) = 0;
        void handleRequest(const MsgPtr &reqMsg) override;

        void run() override;


    protected:
        ChannelPtr mpChannel;

        static int mMapCnt;
        static int mTrajCnt;
    };

} // NAV24::FE

#endif //NAV24_FRONTEND_HPP
