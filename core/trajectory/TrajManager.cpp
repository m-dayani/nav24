//
// Created by masoud on 2/11/24.
//

#include <glog/logging.h>

#include "TrajManager.hpp"
#include "Pose.hpp"

using namespace std;

namespace NAV24 {

    void TrajManager::receive(const MsgPtr &msg) {

        if (msg && msg->getTopic() == TrajManager::TOPIC) {

            switch (msg->getTargetId()) {
                case FCN_TRJ_CREATE:
                    this->createTrajectory(msg);
                    break;
                case FCN_TRJ_POSE_ADD:
                    this->addPose(msg);
                    break;
                default:
                    DLOG(WARNING) << "TrajManager::receive, message action is not supported\n";
                    break;
            }
        }
    }

    void TrajManager::createTrajectory(const MsgPtr &msg) {

        if (msg) {
            string trajName = msg->getMessage();
            auto pTraj = make_shared<Trajectory>(trajName);
            mmpTrajectory.insert(make_pair(trajName, pTraj));
            mActiveTraj = trajName;
        }
    }

    void TrajManager::addPose(const MsgPtr &msg) {

        if (msg) {
            string trajName = msg->getMessage();
            auto msgAddOne = dynamic_pointer_cast<MsgType<PosePtr>>(msg);
            if (msgAddOne) {
                if (mmpTrajectory.count(trajName) > 0) {
                    auto pTraj = mmpTrajectory[trajName];
                    pTraj->addPose(msgAddOne->getData());
                }
            }
        }
    }

    void TrajManager::setup(const MsgPtr &configMsg) {

    }

    void TrajManager::handleRequest(const MsgPtr &reqMsg) {

    }

    void TrajManager::run() {

    }
} // NAV24