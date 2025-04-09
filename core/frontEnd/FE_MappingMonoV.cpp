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

        mpOrbMatcher = make_shared<OP::FtAssocOrbSlam>();
    }

    void MappingMonoV::run() {

        while (!this->isStopped()) {

            this->createNewFrame();

            // extract features and match them
            // triangulate map points
        }
    }

    void MappingMonoV::setup(const MsgPtr &msg) {

        // Get operator parameters
        if (msg && dynamic_pointer_cast<MsgConfig>(msg)) {
            auto mpTempParam = dynamic_pointer_cast<MsgConfig>(msg)->getConfig();
            if (mpTempParam && mpTempParam->getName() == "OP") {
                for (const auto &pOpParamPair: mpTempParam->getAllChildren()) {
                    string key = pOpParamPair.first;
                    auto pOpParam = pOpParamPair.second.lock();

                    mpOrbDetector = OP::FtDt::create(pOpParam, mpChannel);
                    mpOrbDetector->scaleNumFeatures(5.f);
                }
            }
        }
    }

    void MappingMonoV::receive(const MsgPtr &msg) {

        if (msg) {

            if (dynamic_pointer_cast<MsgType<ImagePtr>>(msg)) {
                auto pImage = dynamic_pointer_cast<MsgType<ImagePtr>>(msg)->getData();
                auto pImageTs = dynamic_pointer_cast<ImageTs>(pImage);
                if (pImageTs) {
                    mImageMapLock.lock();
                    mmImage.insert(make_pair(pImageTs->mTimeStamp, pImageTs));
                    mImageMapLock.unlock();
                }
            }
            if (dynamic_pointer_cast<MsgType<PosePtr>>(msg)) {
                auto pPose = dynamic_pointer_cast<MsgType<PosePtr>>(msg)->getData();
                mPoseMapLock.lock();
                mmPose.insert(make_pair(pPose->getTimestamp(), pPose));
                mPoseMapLock.unlock();
            }
        }

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
            FramePtr pFrame = make_shared<FrameImgMono>(imgTs, pPose, vector<OB::ObsPtr>(), pImage);
        }

        // clean up
        if (pPose && pImage) {
            mImageMapLock.lock();
            mmImage.erase(imgTs);
            mImageMapLock.unlock();
            mPoseMapLock.lock();
            auto iter = mmPose.find(poseTs);
            mmPose.erase(mmPose.begin(), iter);
            mPoseMapLock.unlock();
        }
    }


} // NAV24::FE

