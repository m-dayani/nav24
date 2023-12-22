//
// Created by root on 5/18/21.
//

#ifndef NAV24_POSEHOOK_H
#define NAV24_POSEHOOK_H

#include <memory>
#include <utility>

#include "Pose.hpp"

namespace NAV24 {

    class PoseHook {
    public:
        explicit PoseHook(PoseQueuePtr  pPoseQueue) : mpqPose(std::move(pPoseQueue)) {}

        void dispatch(const PosePtr& pPose) { mpqPose->push(pPose); }

    protected:
        PoseQueuePtr mpqPose;
    };

    typedef std::shared_ptr<PoseHook> PoseHookPtr;

} // NAV24

#endif //NAV24_POSEHOOK_H
