//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_TRAJECTORY_HPP
#define NAV24_TRAJECTORY_HPP

#include <memory>
#include <utility>

#include "trajectory/pose/Pose.hpp"


namespace NAV24 {

#define DEF_TRJ_NAME "trj0"

    class Trajectory {
    public:
        explicit Trajectory(std::string traj) : mName(std::move(traj)), mspPoseChain(), mPoseChainLock() {}

        void addPose(const PosePtr& pose);

        void cleanup();

        int getNumPose() {
            int nPose = 0;
            mPoseChainLock.lock();
            if (!mspPoseChain.empty()) {
                nPose = static_cast<int>(mspPoseChain.size());
            }
            mPoseChainLock.unlock();
            return nPose;
        }

    protected:
        std::string mName;
        // todo: or use a ts map instead of vector?
        std::set<PosePtr> mspPoseChain;
        std::mutex mPoseChainLock;
        PosePtr pFirstPose;
        std::shared_ptr<float> mGlobalScale;

        // relations between trajectories: t_offset, T_w1w0 (time, space)
    };
    typedef std::shared_ptr<Trajectory> TrajPtr;

} // NAV24

#endif //NAV24_TRAJECTORY_HPP
