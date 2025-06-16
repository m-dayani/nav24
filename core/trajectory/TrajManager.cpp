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
            mPoseQueue(), mPoseQueueLock(), mmpTrajectory(), mpKfManager(nullptr), mmpTrans() {

        // load and setup keyframe manager
        auto pKfManager = make_shared<OP::KfManagerSimple>(mpChannel);
        mpChannel->registerChannel(ID_CH_OP, pKfManager);
//        string msgKey = string(PARAM_OP) + string(DEF_KFMS_NAME);
//        auto fp = [pKfManager](auto && PH1) { pKfManager->receive(std::forward<decltype(PH1)>(PH1)); };
//        auto msgGetParams = make_shared<MsgRequest>(ID_CH_PARAMS, fp,
//                                                    ParameterServer::TOPIC, FCN_PS_REQ, msgKey);
//        mpChannel->send(msgGetParams);

        mpKfManager = pKfManager;

        this->loadRelations();
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
            if (dynamic_pointer_cast<MsgType<FramePtr>>(msg)) {
                auto pFrame = dynamic_pointer_cast<MsgType<FramePtr>>(msg)->getData();
                if (pFrame) {
                    if (pFrame->getPose()) {
                        this->addPosesToQueue({pFrame->getPose()});
                    }
                }
            }
            if (dynamic_pointer_cast<MsgType<vector<FramePtr>>>(msg)) {
                auto vpFrame = dynamic_pointer_cast<MsgType<vector<FramePtr>>>(msg)->getData();
                vector<PosePtr> vpPose;
                vpPose.reserve(vpFrame.size());
                for (const auto& pFrame: vpFrame) {
                    if (pFrame && pFrame->getPose()) {
                        vpPose.push_back(pFrame->getPose());
                    }
                }
                this->addPosesToQueue(vpPose);
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
            if (trajName.empty()) {
                trajName = DEF_TRJ_NAME;
            }
            shared_ptr<Trajectory> pTraj;
            if (!mmpTrajectory.contains(trajName)) {
                this->createTrajectory(trajName);
            }
            pTraj = mmpTrajectory[trajName];
            if (pTraj) {
                pTraj->addPose(pPose);
            }
        }
    }

    void TrajManager::setup(const MsgPtr &msg) {

        if (msg && dynamic_pointer_cast<MsgConfig>(msg)) {

            auto pParam = dynamic_pointer_cast<MsgConfig>(msg)->getConfig();
            if (pParam && pParam->getName() == PKEY_POSE_REL) {
                for (const auto& relParamPair : pParam->getAllChildren()) {
                    auto pRelParam = relParamPair.second.lock();
                    if (pRelParam) {
                        auto pTrans = TF::PoseSE3::getTrans(pRelParam);
                        if (pTrans) {
                            mmpTrans.insert(make_pair(pTrans->getName(), pTrans));
                        }
                    }
                }
            }
        }
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

                int action = msg->getTargetId();
                string msgStr = msg->getMessage();
                if (action == FCN_GET_TRANS) {
                    if (mmpTrans.count(msgStr) > 0) {

                        auto pTrans = mmpTrans[msgStr];
                        auto msgTrans = make_shared<MsgType<PosePtr>>(DEF_CAT, pTrans, msg->getTopic());
                        senderCb(msgTrans);
                    }
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

            // Pose cleanup
            for (const auto& pTrajInfo : mmpTrajectory) {
                if (pTrajInfo.second) {
                    pTrajInfo.second->cleanup();
                }
            }

            mPoseQueueLock.lock();
            if (!mPoseQueue.empty() && mPoseQueue.front() == pPose) {
                mPoseQueue.pop_front();
            }
            mPoseQueueLock.unlock();

            stringstream trajInfo;
            for (const auto& traj : mmpTrajectory) {
                if (traj.second) {
                    trajInfo << "(" << traj.first << ", " << traj.second->getNumPose() << "), ";
                }
            }
            DLOG_EVERY_N(INFO, 10000) << "TrajManager::run, Trajectory Info, (name, num poses): "
                                     << trajInfo.str() << "\n";
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

    void TrajManager::loadRelations() {

        auto fp = [this](auto && PH1) {
            receive(std::forward<decltype(PH1)>(PH1));
        };
        auto msgGetParams = make_shared<MsgRequest>(ID_CH_PARAMS, fp,
                                                    ParameterServer::TOPIC, FCN_PS_REQ, PARAM_REL);
        mpChannel->send(msgGetParams);
    }

} // NAV24