//
// Created by masoud on 2/11/24.
//

#include <glog/logging.h>

#include "TrajManager.hpp"
#include "System.hpp"
#include "OP_KfManagerSimple.hpp"
#include "ParameterBlueprint.h"
//#include "trajectory/pose/Pose.hpp"

using namespace std;

namespace NAV24 {

    TrajManager::TrajManager(const ChannelPtr &pChannel) : MsgCallback(pChannel),
            mPoseQueue(), mPoseQueueLock(), mmpTrajectory(), mpKfManager(nullptr) {

        // load and setup keyframe manager
        auto pKfManager = make_shared<OP::KfManagerSimple>(mpChannel);
        mpChannel->registerChannel(ID_CH_OP, pKfManager);
//        string msgKey = string(PARAM_OP) + string(DEF_KFMS_NAME);
//        auto fp = [pKfManager](auto && PH1) { pKfManager->receive(std::forward<decltype(PH1)>(PH1)); };
//        auto msgGetParams = make_shared<MsgRequest>(ID_CH_PARAMS, fp,
//                                                    ParameterServer::TOPIC, FCN_PS_REQ, msgKey);
//        mpChannel->send(msgGetParams);

        mpKfManager = pKfManager;
    }

    void TrajManager::receive(const MsgPtr &msg) {

        if (msg) {
            if (msg->getTopic() == TrajManager::TOPIC) {
                switch (msg->getTargetId()) {
                    case FCN_TRJ_CREATE:
                        this->createTrajectory(msg->getMessage());
                        break;
//                    case FCN_TRJ_POSE_ADD:
//                        this->insertPose(msg);
//                        break;
                    default:
                        DLOG(WARNING) << "TrajManager::receive, message action is not supported\n";
                        break;
                }
            }

            if (dynamic_pointer_cast<MsgType<PosePtr>>(msg)) {
                this->addPosesToQueue({ dynamic_pointer_cast<MsgType<PosePtr>>(msg)->getData() });
            }
            if (dynamic_pointer_cast<MsgType<vector<PosePtr>>>(msg)) {
                this->addPosesToQueue(dynamic_pointer_cast<MsgType<vector<PosePtr>>>(msg)->getData());
            }
            if (dynamic_pointer_cast<MsgConfig>(msg)) {
                this->setup(msg);
            }
            if (dynamic_pointer_cast<MsgRequest>(msg)) {
                this->handleRequest(msg);
            }
            if (msg->getTargetId() == FCN_SYS_STOP) {
                this->stop();
            }
        }
    }

    void TrajManager::createTrajectory(const string &trajName) {

        auto pTraj = make_shared<Trajectory>(trajName);
        mmpTrajectory.insert(make_pair(trajName, pTraj));
        mActiveTraj = trajName;
    }

    void TrajManager::insertPose(const PosePtr &pPose) {

        if (pPose) {
            string trajName = pPose->getName();
            shared_ptr<Trajectory> pTraj;
            if (trajName.empty() || !mmpTrajectory.contains(trajName)) {
                pPose->setName(mActiveTraj);
                pTraj = mmpTrajectory[mActiveTraj];
            }
            else {
                pTraj = mmpTrajectory[trajName];
            }
            if (pTraj) {
                pTraj->addPose(pPose);
            }
        }
    }

    void TrajManager::setup(const MsgPtr &) {

    }

    void TrajManager::handleRequest(const MsgPtr &msg) {

        if (msg && dynamic_pointer_cast<MsgRequest>(msg)) {
            auto msgReq = dynamic_pointer_cast<MsgRequest>(msg);
            auto senderCb = msgReq->getCallbackFun();
            if (senderCb) {
                if (msg->getTargetId() == FCN_SYS_RUN) {
                    auto pThread = make_shared<thread>(&TrajManager::run, this);
                    auto msgRes = make_shared<MsgType<shared_ptr<thread>>>(ID_CH_SYS, pThread,
                                                                           System::TOPIC);
                    senderCb(msgRes);
                }
            }
        }

    }

    void TrajManager::run() {

        DLOG(INFO) << "TrajManager::run, started main loop\n";

        list<PosePtr> lpPose;

        while (!this->isStopped()) {

            mPoseQueueLock.lock();
            lpPose = mPoseQueue;
            mPoseQueueLock.unlock();

            PosePtr pPose = nullptr;
            if (!lpPose.empty()) {
                pPose = lpPose.front();
            }

            if (pPose) {
//                cout << "Found a pose: " << pPose->getTimestamp() << endl;
                // insert pose to its trajectory
                this->insertPose(pPose);

                // manage frames (keyframe addition, deletion, ...)
                mpKfManager->checkPose(pPose);

                // merge pose groups

                // pose optimization

                // ...
            }

            mPoseQueueLock.lock();
            if (!mPoseQueue.empty() && mPoseQueue.front() == pPose) {
                mPoseQueue.pop_front();
            }
            mPoseQueueLock.unlock();
        }

        DLOG(INFO) << "TrajManager::run, stopped\n";
    }

    void TrajManager::addPosesToQueue(const vector <PosePtr> &vpPose) {

        mPoseQueueLock.lock();
        for (const auto& pPose : vpPose) {
            if (pPose) {
                mPoseQueue.push_back(pPose);
            }
        }
        mPoseQueueLock.unlock();
    }

} // NAV24