//
// Created by masoud on 2/11/24.
//

#include "Trajectory.hpp"

using namespace std;


namespace NAV24 {

    void Trajectory::addPose(const PosePtr &pose) {
        if (pose) {
            mPoseChainLock.lock();
            mspPoseChain.insert(pose);
            mPoseChainLock.unlock();
        }
    }

    void Trajectory::cleanup() {
        // remove all invalid poses from the list
        set<PosePtr> poseChain;
        mPoseChainLock.lock();
        if (!mspPoseChain.empty()) {
            for (const auto& pPose : mspPoseChain) {
                if (pPose->isValid()) {
                    poseChain.insert(pPose);
                }
            }
        }
        mspPoseChain = poseChain;
        mPoseChainLock.unlock();
    }
} // NAV24