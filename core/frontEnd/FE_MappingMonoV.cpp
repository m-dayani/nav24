//
// Created by masoud on 4/7/25.
//

#include "FE_MappingMonoV.hpp"
#include "Image.hpp"
#include "Frame.hpp"
#include "ParameterServer.hpp"


using namespace std;


namespace NAV24::FE {

    MappingMonoV::MappingMonoV(const ChannelPtr &pChannel) : FrontEnd(pChannel),
            mmPose(), mPoseMapLock(), mmImage(), mImageMapLock() {

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

    void MappingMonoV::setup(const MsgPtr &) {
        // Get operator parameters
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
            auto pNewPose = make_shared<TF::PoseSE3>(pPose->getTimestamp(), pPose->getPose(), "world0");
            FramePtr pFrame = make_shared<FrameImgMono>(imgTs, pNewPose, vector<OB::ObsPtr>(), pImage);
            // publish the new pose
            auto pPoseMsg = make_shared<MsgType<PosePtr>>(ID_TP_SDATA, pNewPose, FrontEnd::TOPIC);
            mpChannel->publish(pPoseMsg);
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

