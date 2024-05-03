//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_TRAJECTORY_HPP
#define NAV24_TRAJECTORY_HPP

#include <memory>

#include "Pose.hpp"


namespace NAV24 {

    class Trajectory {
    public:
        explicit Trajectory(const std::string& traj) : mName(), mvpPoseChain() {}

        void addPose(const PosePtr& pose);

    protected:
        std::string mName;
        std::vector<PosePtr> mvpPoseChain;
    };
    typedef std::shared_ptr<Trajectory> TrajPtr;

} // NAV24

#endif //NAV24_TRAJECTORY_HPP
