//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_TRAJMANAGER_HPP
#define NAV24_TRAJMANAGER_HPP

#include <memory>
#include <list>

#include "Message.hpp"
#include "Interface.hpp"
#include "Trajectory.hpp"
#include "Operator.hpp"
#include "OP_KfManagerSimple.hpp"


namespace NAV24 {

#define FCN_TRJ_CREATE 3
#define FCN_TRJ_POSE_ADD 4

    class TrajManager : public MsgCallback {
    public:
        inline static const std::string TOPIC = "TrajManager";

        explicit TrajManager(const ChannelPtr& pChannel);

        void receive(const MsgPtr &msg) override;

    protected:
        void createTrajectory(const std::string &msg);
        void insertPose(const PosePtr &pPose);
        void addPosesToQueue(const std::vector<PosePtr>& vpPose);

        void setup(const MsgPtr &configMsg) override;

        void handleRequest(const MsgPtr &reqMsg) override;

        void run() override;

    protected:
        std::list<PosePtr> mPoseQueue;
        std::mutex mPoseQueueLock;

        std::map<std::string, TrajPtr> mmpTrajectory;
        std::string mActiveTraj;

        std::shared_ptr<OP::KfManagerSimple> mpKfManager;
    };
    typedef std::shared_ptr<TrajManager> TrajManagerPtr;

} // NAV24

#endif //NAV24_TRAJMANAGER_HPP
