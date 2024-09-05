//
// Created by masoud on 6/21/24.
//
// Note since it's a performance bottleneck to send/receive
// messages from modules (e.g. millions of map points),
// it is better to use flags to mark changes on distributed pointer
// objects, to be changed locally

#ifndef NAV24_SMARTOBJECT_HPP
#define NAV24_SMARTOBJECT_HPP

#include <mutex>

namespace NAV24 {
    class SmartObject {
    public:
        SmartObject() : mbValid(true), mbVisible(true), mbOptFixed(false), mbOptLockState(false), mbOptIgnore(false),
            mMtxLockState(), mMtxOptFixed(), mMtxOptIgnore(), mMtxValid(), mMtxVisible() {}

        [[nodiscard]] virtual bool isValid() const {
            return mbValid;
        }

        virtual void setValid(bool bValid) {
            mMtxValid.lock();
            SmartObject::mbValid = bValid;
            mMtxValid.unlock();
        }

        [[nodiscard]] virtual bool isVisible() const {
            return mbVisible;
        }

        virtual void setVisible(bool bVisible) {
            mMtxVisible.lock();
            SmartObject::mbVisible = bVisible;
            mMtxVisible.unlock();
        }

        [[nodiscard]] bool isOptLockState() const {
            return mbOptLockState;
        }

        void setOptLockState(bool optLockState) {
            mMtxLockState.lock();
            SmartObject::mbOptLockState = optLockState;
            mMtxLockState.unlock();
        }

        [[nodiscard]] bool isOptFixed() const {
            return mbOptFixed;
        }

        void setOptFixed(bool optFixed) {
            mMtxOptFixed.lock();
            SmartObject::mbOptFixed = optFixed;
            mMtxOptFixed.unlock();
        }

        [[nodiscard]] bool isOptIgnore() const {
            return mbOptIgnore;
        }

        void setOptIgnore(bool optIgnore) {
            mMtxOptIgnore.lock();
            SmartObject::mbOptIgnore = optIgnore;
            mMtxOptIgnore.unlock();
        }

    protected:
        // Controls validity of a frame, observation, or world object
        bool mbValid;
        std::mutex mMtxValid;
        // Controls the visibility of visual vars
        bool mbVisible;
        std::mutex mMtxVisible;
        // Flags for optimization
        // Lock the state of the variable
        bool mbOptLockState;
        std::mutex mMtxLockState;
        // Fixed optimization vertices
        bool mbOptFixed;
        std::mutex mMtxOptFixed;
        // Ignore this variable for an optimization problem
        bool mbOptIgnore;
        std::mutex mMtxOptIgnore;
    };
} // NAV24

#endif //NAV24_SMARTOBJECT_HPP
