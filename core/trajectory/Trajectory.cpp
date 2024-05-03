//
// Created by masoud on 2/11/24.
//

#include "Trajectory.hpp"

namespace NAV24 {

    void Trajectory::addPose(const PosePtr &pose) {
        if (pose) {
            mvpPoseChain.push_back(pose);
        }
    }
} // NAV24