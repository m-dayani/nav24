//
// Created by masoud on 2/11/24.
//

#include <glog/logging.h>

#include "Atlas.hpp"
#include "System.hpp"


using namespace std;

namespace NAV24 {

    Atlas::Atlas(const ChannelPtr& pChannel) : MsgCallback(pChannel),
            mWorlds(), mvpFrameBuffer(), mFrameBuffLock(), mvpKeyframes() {

        mpMpManager = make_shared<OP::MapPointManager>(mpChannel);
        mpChannel->registerChannel(ID_CH_OP, mpMpManager);
        mpVprDbow2 = make_shared<OP::VPR_DBoW2>(mpChannel);
        mpChannel->registerChannel(ID_CH_OP, mpVprDbow2);
    }

    void Atlas::receive(const MsgPtr &msg) {

        if (msg) {
            if (msg->getTopic() == Atlas::TOPIC) {
                switch (msg->getTargetId()) {
                    case FCN_MAP_CREATE:
                        this->createMap(msg);
                        break;
                    case FCN_MAP_ADD_WO:
                        this->addWorldObjects(msg);
                        break;
                    default:
                        DLOG(WARNING) << "Atlas::receive, unsupported action\n";
                        break;
                }
            }
            if (dynamic_pointer_cast<MsgType<FramePtr>>(msg)) {
                auto pFrame = dynamic_pointer_cast<MsgType<FramePtr>>(msg)->getData();
                if (pFrame) {
                    this->insertFrame(pFrame);
                }
            }
            if (dynamic_pointer_cast<MsgRequest>(msg)) {
                this->handleRequest(msg);
            }
            if (msg->getTargetId() == FCN_SYS_STOP) {
                this->stop();
            }
        }
    }

    void Atlas::createMap(const MsgPtr &msg) {

        if (msg) {
            string mapName = msg->getMessage();
            MapPtr pMap = make_shared<Map>(mapName);
            mWorlds.insert(make_pair(mapName, pMap));
            mActiveWorld = mapName;
        }
    }

    void Atlas::addWorldObjects(const MsgPtr &msg) {

        if (msg) {
            auto msgData = dynamic_pointer_cast<MsgType<vector<WO::WoPtr>>>(msg);
            if (msgData) {
                string mapName = msg->getMessage();
                if (mapName.empty()) {
                    mapName = mActiveWorld;
                }
                if (mWorlds.count(mapName) <= 0) {
                    DLOG(WARNING) << "Atlas::addWorldObjects, could not find map: " << mapName << "\n";
                    return;
                }
                auto pMap = mWorlds[mapName];
                auto vWobj = msgData->getData();
                for (const auto& wobj : vWobj) {
                    pMap->addWorldObject(wobj);
                }
            }
        }
    }

    void Atlas::setup(const MsgPtr &) {

    }

    void Atlas::handleRequest(const MsgPtr &msg) {
        if (msg && dynamic_pointer_cast<MsgRequest>(msg)) {
            auto msgReq = dynamic_pointer_cast<MsgRequest>(msg);
            auto senderCb = msgReq->getCallbackFun();
            if (senderCb) {
                if (msg->getTargetId() == FCN_SYS_RUN) {
                    auto pThread = make_shared<thread>(&Atlas::run, this);
                    auto msgRes = make_shared<MsgType<shared_ptr<thread>>>(ID_CH_SYS, pThread,
                                                                           System::TOPIC);
                    senderCb(msgRes);
                }
            }
        }
    }

    void Atlas::run() {

        DLOG(INFO) << "Atlas::run, started the main loop\n";

        while (!this->isStopped()) {

            // process frames to find key frames
            this->processFrames();

            // manage world objects (map points)
            if (!mvpKeyframes.empty()) {

                // compute DBoW feature vector for the last frame
                for (const auto& pKF : mvpKeyframes) {
                    if (dynamic_pointer_cast<FrameMonoOS>(pKF)) {
                        if (!dynamic_pointer_cast<FrameMonoOS>(pKF)->isInitDBoW2()) {
                            mpVprDbow2->computeBowInfo(mvpKeyframes.back());
                        }
                    }
                }

                // create new map points
                std::vector<WO::WoPtr> vpPoints3d;
                mpMpManager->checkNewKeyFrame(mvpKeyframes.back(), vpPoints3d);

                // todo: process and publish map points
            }

            // manage maps

            // associate visual cues, maps, world objects

            // optimization
        }

        DLOG(INFO) << "Atlas::run, exit the main loop\n";
    }

    void Atlas::insertFrame(const FramePtr &pFrame) {

        if (pFrame) {
            mFrameBuffLock.lock();
            mvpFrameBuffer.push_back(pFrame);
            mFrameBuffLock.unlock();
        }
    }

    void Atlas::processFrames() {

        size_t kf_idx = 0;
        mFrameBuffLock.lock();

        for (size_t i = 0; i < mvpFrameBuffer.size(); i++) {
            const auto& pFrame = mvpFrameBuffer[i];
            if (pFrame && pFrame->getLevel() > 0) {
                mvpKeyframes.push_back(pFrame);
                kf_idx = i + 1;
            }
        }

        // remove redundant frames
        if (kf_idx != 0 || mvpFrameBuffer.size() >= ATLAS_DEF_FRAME_BUFF_SIZE) {
            if (kf_idx == 0) {
                kf_idx = 1;
            }
            vector<FramePtr> newFrames;
            for (size_t i = kf_idx; i < mvpFrameBuffer.size(); i++) {
                newFrames.push_back(mvpFrameBuffer[i]);
            }
            mvpFrameBuffer = newFrames;
        }

        mFrameBuffLock.unlock();
    }


} // NAV24