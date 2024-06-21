//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_TRAJECTORY_HPP
#define NAV24_TRAJECTORY_HPP

#include <memory>
#include <utility>

#include "trajectory/pose/Pose.hpp"


namespace NAV24 {

    class Trajectory {
    public:
        explicit Trajectory(std::string traj) : mName(std::move(traj)), mvpPoseChain() {}

        void addPose(const PosePtr& pose);

    protected:
        std::string mName;
        std::vector<PosePtr> mvpPoseChain;
    };
    typedef std::shared_ptr<Trajectory> TrajPtr;

} // NAV24

#endif //NAV24_TRAJECTORY_HPP
