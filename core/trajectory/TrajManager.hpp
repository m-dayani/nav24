//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_TRAJMANAGER_HPP
#define NAV24_TRAJMANAGER_HPP

#include <memory>

#include "Message.hpp"
#include "Interface.hpp"
#include "Trajectory.hpp"


namespace NAV24 {

#define FCN_TRJ_CREATE 3
#define FCN_TRJ_POSE_ADD 4

    class TrajManager : public MsgCallback {
    public:
        inline static const std::string TOPIC = "TrajManager";

        void receive(const MsgPtr &msg) override;

    protected:
        void createTrajectory(const MsgPtr &msg);
        void addPose(const MsgPtr &msg);

        void setup(const MsgPtr &configMsg) override;

        void handleRequest(const MsgPtr &reqMsg) override;

        void run() override;

    protected:
        ChannelPtr mpChannel;

        std::map<std::string, TrajPtr> mmpTrajectory;
        std::string mActiveTraj;
    };
    typedef std::shared_ptr<TrajManager> TrajManagerPtr;

} // NAV24

#endif //NAV24_TRAJMANAGER_HPP
