//
// Created by masoud on 4/7/25.
//

#include <glog/logging.h>

#include "FE_MappingMonoV.hpp"
#include "Image.hpp"
#include "Frame.hpp"
#include "ParameterServer.hpp"
#include "System.hpp"


using namespace std;


namespace NAV24::FE {

    MappingMonoV::MappingMonoV(const ChannelPtr &pChannel) : FrontEnd(pChannel),
            mmPose(), mPoseMapLock(), mmImage(), mImageMapLock(), T_bc0(), mpLastFrame(), mvpAllFrames() {

        // initialize all required operators
        mpOrbDetector = make_shared<OP::FtDtOrbSlam>(mpChannel);
        mpChannel->registerChannel(ID_CH_OP, mpOrbDetector);
        mpOrbMatcher = make_shared<OP::FtAssocOrbSlam>(mpChannel);
        mpChannel->registerChannel(ID_CH_OP, mpOrbMatcher);
    }

    void MappingMonoV::run() {

        while (!this->isStopped()) {

            this->processImage();
        }
    }

    void MappingMonoV::setup(const MsgPtr &msg) {

        if (msg && dynamic_pointer_cast<MsgConfig>(msg)) {
            DLOG(INFO) << "MappingMonoV::setup called\n";
        }
    }

    void MappingMonoV::receive(const MsgPtr &msg) {

        if (msg) {

            if (dynamic_pointer_cast<MsgSensorData>(msg)) {
                auto pImage = dynamic_pointer_cast<MsgSensorData>(msg)->getData();
                auto pImageTs = dynamic_pointer_cast<ImageTs>(pImage);
                if (pImageTs) {
                    mImageMapLock.lock();
                    mmImage.insert(make_pair(pImageTs->mTimeStamp, pImageTs));
                    mImageMapLock.unlock();

                    this->processImage();
                }
            }
            if (dynamic_pointer_cast<MsgType<PosePtr>>(msg) && msg->getTopic() != FrontEnd::TOPIC) {
                auto pPose = dynamic_pointer_cast<MsgType<PosePtr>>(msg)->getData();
                mPoseMapLock.lock();
                mmPose.insert(make_pair(pPose->getTimestamp(), pPose));
                mPoseMapLock.unlock();
            }
            if (dynamic_pointer_cast<MsgType<PosePtr>>(msg) && msg->getTopic() == FrontEnd::TOPIC) {
                // Load pose relations
                auto pPose = dynamic_pointer_cast<MsgType<PosePtr>>(msg)->getData();
                if (pPose && pPose->getName() == "T_bc0") {
                    T_bc0 = pPose;
                }
            }
        }

    }

    void MappingMonoV::processImage() {

        this->createNewFrame();

        // extract features and match them
        // triangulate map points
    }

    void MappingMonoV::createNewFrame() {

        // grab frames and poses
        double imgTs = -1.0;
        ImagePtr pImage;
        mImageMapLock.lock();
        if (!mmImage.empty()) {
            auto tsImage = *(mmImage.begin());
            imgTs = tsImage.first;
            pImage = tsImage.second;
        }
        mImageMapLock.unlock();

        double poseTs = -1.0;
        PosePtr pPose;
        if (imgTs >= 0) {
            double maxPoseTs = -1.0, minPoseTs = -1.0;
            mPoseMapLock.lock();
            for (const auto& tsPose : mmPose) {
                double currPoseTs = tsPose.first;
                if (currPoseTs > imgTs) {
                    maxPoseTs = currPoseTs;
                    break;
                }
                minPoseTs = currPoseTs;
            }
            if (maxPoseTs >= 0) {
                if (minPoseTs >= 0) {
                    poseTs = (abs(maxPoseTs - imgTs) < abs(minPoseTs - imgTs)) ? maxPoseTs : minPoseTs;
                }
                else {
                    poseTs = maxPoseTs;
                }
            }
            if (poseTs >= 0) {
                pPose = mmPose[poseTs];
            }
            mPoseMapLock.unlock();
        }

        // link them to create a full MonoV frame
        if (pPose && pImage) {
            // This is based on EuRoC poses
            // GT poses are stored like: T_wb, and we have T_bc0 from calib -> T_wc0 = T_wb * T_bc0
            Eigen::Matrix4d T_wc0 = pPose->getPose();
            if (T_bc0) {
                T_wc0 = pPose->getPose() * T_bc0->getPose();
            }
            auto pNewPose = make_shared<TF::PoseSE3>(pPose->getTimestamp(), T_wc0, FE_DEF_WORLD_NAME);
            FramePtr pFrame = make_shared<FrameMonoOS>(imgTs, pNewPose, vector<OB::ObsPtr>(), pImage);
            pNewPose->setFrame(pFrame);
            pFrame->setPrevFrame(mpLastFrame);
            if (mpLastFrame) {
                mpLastFrame->setNextFrame(pFrame);
            }

            // publish the new pose
//            auto pPoseMsg = make_shared<MsgType<PosePtr>>(ID_TP_SDATA, pNewPose, FrontEnd::TOPIC);
//            mpChannel->publish(pPoseMsg);

            auto pFrameMsg = make_shared<MsgType<FramePtr>>(ID_TP_SDATA, pFrame, FrontEnd::TOPIC);
            mpChannel->publish(pFrameMsg);

            mpLastFrame = pFrame;

            // must store all frames, otherwise, they are destroyed when go out of scope
//            pFrame->simplify();
            mvpAllFrames.push_back(pFrame);
        }

        // clean up
        if (pPose && pImage) {
            mImageMapLock.lock();
            mmImage.erase(imgTs);
            mImageMapLock.unlock();
            mPoseMapLock.lock();
            auto iter = mmPose.begin(), iter_end = mmPose.find(poseTs);
            for (; iter != iter_end; iter++) {
                iter->second->setValid(false);
//                mmPose.erase(iter);
            }
            mmPose.erase(mmPose.begin(), iter_end);
            mPoseMapLock.unlock();
        }
    }


} // NAV24::FE

