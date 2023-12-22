//
// Created by root on 5/18/21.
//

#ifndef NAV24_IMU_HOOK_H
#define NAV24_IMU_HOOK_H

#include <memory>
#include <utility>

#include "IMU.hpp"
#include "SharedQueue.hpp"


namespace NAV24 {

    class IMU_Hook {
    public:
        explicit IMU_Hook(IMU_QueuePtr  pImuQueue) : mpqIMU_Data(std::move(pImuQueue)) {}

        void dispatch(const IMU_DataPtr& pImuData) { mpqIMU_Data->push(pImuData); }

    protected:
        IMU_QueuePtr mpqIMU_Data;
    };

    typedef std::shared_ptr<IMU_Hook> IMU_HookPtr;

} // NAV24


#endif //NAV24_IMU_HOOK_H
