//
// Created by masoud on 8/29/24.
//

/*
 * todo: Future Directions
 * 1. Better optimization: account for other vtx signals (ignore, inValid, ...)
 *      count number of fixed/non-fixed and decide how to formulate the problem (e.g. schur decomposition)
 *      if a problem is ill-conditioned, abort optimization
 * 2. Separate Backend Loop and send problems through message passing
 * 3. Add logic for keyframe/map point insertion (ORB-SLAM Local Mapping thread)
 * 4. Visualization & broadcasting vars to other components
 */

#include <glog/logging.h>
#include "opencv2/core.hpp"
#ifdef HAVE_OPENCV_XFEATURES2D
#include "opencv2/xfeatures2d.hpp"
#endif

#include "FE_SlamMonoV.hpp"
#include "Image.hpp"
#include "Output.hpp"
#include "Atlas.hpp"
#include "TrajManager.hpp"
#include "ParameterServer.hpp"
#include "ParameterBlueprint.h"
#include "Visualization.hpp"
#include "Camera.hpp"
#include "Problem.hpp"
#include "BE_GraphOptim.hpp"

using namespace std;

namespace NAV24::FE {

    SlamMonoV::SlamMonoV(const ChannelPtr &pChannel) : FrontEnd(pChannel),
        mpOrbDetector(), mpTempParam(), mbInitialized(false), mbMapInitialized(false),
        mpFirstFrame(), mpLastFrame(), mpCurrFrame(), mmpFrames(),
        mpFtTracks(), mpOrbMatcher(), mpCalib(), mvpLocalMapPoints(), mvpTrackedFrames()  {

        mpOrbMatcher = make_shared<OP::FtAssocOrbSlam>();
        mpFtTracks = make_shared<OB::FeatureTracks>();
    }

    void SlamMonoV::handleRequest(const MsgPtr &reqMsg) {
        FrontEnd::handleRequest(reqMsg);
    }

    void SlamMonoV::run() {
        FrontEnd::run();
    }

    void SlamMonoV::receive(const MsgPtr &msg) {

        if (msg) {
            if (dynamic_pointer_cast<MsgSensorData>(msg)) {
                this->handleImageMsg(msg);
            }
            if (dynamic_pointer_cast<MsgType<CalibPtr>>(msg)) {
                mpCalib = dynamic_pointer_cast<MsgType<CalibPtr>>(msg)->getData();
                mpMapInit = make_shared<OP::MapInitializer>(mpCalib);
            }
            if (dynamic_pointer_cast<MsgConfig>(msg)) {
                // can receive messages from parameter server and main
                auto msgConfig = dynamic_pointer_cast<MsgConfig>(msg);
                mpTempParam = msgConfig->getConfig();
                if (!mpTempParam && !mbInitialized) {
                    this->setup(msg);
                }
            }
        }
    }

    void SlamMonoV::handleImageMsg(const MsgPtr &msg) {

        if (dynamic_pointer_cast<MsgSensorData>(msg)) {
            auto msgSensor = dynamic_pointer_cast<MsgSensorData>(msg);
            auto sensorData = msgSensor->getData();

            if (sensorData && dynamic_pointer_cast<ImageTs>(sensorData)) {

                auto pImage = dynamic_pointer_cast<ImageTs>(sensorData);

                if (!pImage || pImage->mImage.empty()) {
                    DLOG(WARNING) << "ObjTracking::handleImageMsg, empty image detected\n";
                    return;
                }

                cv::Mat img = pImage->mImage.clone();
                cv::Mat gray = img.clone();
                if (gray.channels() == 3) {
                    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
                }

                if (!mpDummyGrid) {
                    vector<OB::ObsPtr> vpObs;
                    mpDummyGrid = make_shared<OB::FeatureGrid>(vpObs);
                    vector<float> imgBounds = mpCalib->computeImageBounds(img);
                    mpDummyGrid->setImageBounds(cv::Size(img.cols, img.rows), imgBounds);
                }

                // Extract features
                mpCurrFrame = make_shared<FrameMonoGrid>(pImage->mTimeStamp, nullptr, vector<OB::ObsPtr>(), pImage);
//                mmpFrames.insert(make_pair(mpCurrFrame->getId(), mpCurrFrame));
                if (mpOrbDetector) {
                    mpOrbDetector->detect(mpCurrFrame);
                }

                if (!mpFirstFrame) {
                    mpFirstFrame = mpCurrFrame;
                }

                // Undistort features
                if (mpCalib) {
                    mpCurrFrame->setObservations(mpCalib->undistort(mpCurrFrame->getObservations()));
                }

                // Match Features
                if (mpOrbMatcher && mpFirstFrame && mpFirstFrame != mpCurrFrame) {
                    mpOrbMatcher->match(mpFirstFrame, mpCurrFrame);
//                    cout << "N matches: " << nMch << endl;
                }

                if (!mbMapInitialized) {

                    // Initialize map
                    if (mpMapInit) {
                        mvpLocalMapPoints.clear();
                        bool resMapInit = mpMapInit->reconstruct(mpFirstFrame, mpCurrFrame,mvpLocalMapPoints);
                        if (resMapInit) {
                            mvpKeyFrames.clear();
                            mvpKeyFrames.push_back(mpFirstFrame);
                            mvpKeyFrames.push_back(mpCurrFrame);
                            mbMapInitialized = true;
                            mpOrbDetector->scaleNumFeatures(0.2f);
                            mpFirstFrame = mpCurrFrame;
                            mvpTrackedFrames.clear();
                            mvpTrackedFrames.push_back(mpFirstFrame);
                        }
                    }
                }
                else {
                    // Track local map
                    connectMatchedFrames(mvpTrackedFrames.back(), mpCurrFrame);

                    auto pLastPose = mpLastFrame->getPose();
                    mpCurrFrame->setPose(make_shared<TF::PoseSE3>(pLastPose->getRef(),
                                                                  pLastPose->getTarget(),
                                                                  pLastPose->getTimestamp(),
                                                                  pLastPose->getPose()));
                    // Optimize for pose
                    mvpTrackedFrames.push_back(mpCurrFrame);
                    mvpTrackedFrames[0]->setOptFixed(true);
                    mvpTrackedFrames.back()->setOptFixed(false);
                    for (auto& pObs : mvpTrackedFrames[0]->getObservations()) {
                        if (pObs && pObs->getWorldObject()) {
                            pObs->getWorldObject()->setOptFixed(true);
                        }
                    }
                    auto pProblem = make_shared<PR_VBA>(mvpTrackedFrames, mpCalib);
                    pProblem->setNumIter(5);
                    // Publish to the BA backend
                    auto pVbaSolver = make_shared<BE::GraphOptim>();
                    pVbaSolver->solve(pProblem);


                }

                mpLastFrame = mpCurrFrame;

                if (OB::MatchedObs::getNumMatches(mpCurrFrame) < 50 || mvpTrackedFrames.size() > 10) {
                    // tracking lost
                    mpFirstFrame = mpCurrFrame;
                    mbMapInitialized = false;
                    mpOrbDetector->scaleNumFeatures(5.f);
                }

                cv::Mat imgShow;

                // Draw detected features
                imgShow = img.clone();
                Visualization::drawKeyPoints(imgShow, mpCurrFrame);
//                Visualization::drawMatchedTracks(imgShow, mpFtTracks);

                // Show Image
                if (!imgShow.empty()) {
                    auto pImageMatch = make_shared<ImageTs>(imgShow, pImage->mTimeStamp, "");
                    auto msgImShow = make_shared<MsgSensorData>(ID_TP_OUTPUT, pImageMatch,
                                                                Output::TOPIC);
                    mpChannel->publish(msgImShow);
                }

                dynamic_pointer_cast<FrameImgMono>(mpCurrFrame)->deleteCvImage();
            }
        }
    }

    void SlamMonoV::setup(const MsgPtr &configMsg) {

        // Create a tracking map
        // Never store a local map or anything else (leave this to each manager)
        mMapName = "world0";
        auto msgCreateMap = make_shared<Message>(ID_CH_ATLAS, Atlas::TOPIC,
                                                 FCN_MAP_CREATE, mMapName);
        mpChannel->send(msgCreateMap);

        // Create a point trajectory (fixed camera)
        mTrajectory = "cam0";
        auto msgCreateTraj = make_shared<Message>(ID_CH_TRAJECTORY, TrajManager::TOPIC,
                                                  FCN_TRJ_CREATE, mTrajectory);
        mpChannel->send(msgCreateTraj);

        // Request world0:cam0 relation from system
//        auto msgGetRel = make_shared<MsgRequest>(ID_CH_SYS, shared_from_this(),
//                                                 System::TOPIC,FCN_GET_TRANS, "world0:cam0");
//        mpChannel->send(msgGetRel);

        // Add it to point trajectory
//        auto msgAddPose = make_shared<MsgType<PosePtr>>(ID_CH_TRAJECTORY, mpTwc,
//                                                        TrajManager::TOPIC,FCN_TRJ_POSE_ADD, "cam0");
//        mpChannel->send(msgAddPose);

        // Init Operators
        this->initOperators();

        // Load camera's calib parameters
        auto msgReqCalib = make_shared<MsgRequest>(ID_CH_SENSORS, shared_from_this(),
                                                   Sensor::TOPIC, FCN_CAM_GET_CALIB);
        mpChannel->send(msgReqCalib);

        mbInitialized = true;
    }

    void SlamMonoV::initOperators() {

        // Get operator parameters
        MsgPtr msgOpParams = make_shared<MsgRequest>(ID_CH_PARAMS, shared_from_this(), ParameterServer::TOPIC,
                                                     FCN_PS_REQ, string(PARAM_OP));
        mpChannel->send(msgOpParams);
        if (mpTempParam && mpTempParam->getName() == "OP") {
            for (const auto& pOpParamPair : mpTempParam->getAllChildren()) {
                string key = pOpParamPair.first;
                auto pOpParam = pOpParamPair.second.lock();

                mpOrbDetector = OP::FtDt::create(pOpParam, mpChannel);
                mpOrbDetector->scaleNumFeatures(5.f);
            }
        }

    }

    void SlamMonoV::connectMatchedFrames(FramePtr &pFrame1, FramePtr &pFrame2) {

        if (!pFrame2 || !pFrame1) {
            return;
        }

        vector<int> vMatches12{};
        int nMatches = 0;
        OB::MatchedObs::getMatches(pFrame2, vMatches12, nMatches);
        if (nMatches <= 0) {
            return;
        }

        auto vpObs1 = pFrame1->getObservations();
        auto vpObs2 = pFrame2->getObservations();

        // connect matches through map points
        for (size_t i1 = 0; i1 < vMatches12.size(); i1++) {
            int i2 = vMatches12[i1];
            if (i2 < 0) {
                continue;
            }
            if (i1 >= vpObs1.size() || i2 >= vpObs2.size()) {
                continue;
            }
            auto pObs1 = vpObs1[i1];
            auto pObs2 = vpObs2[i2];
            if (pObs1 && pObs1->getWorldObject() && pObs2) {
                pObs1->getWorldObject()->addObservation(pObs2);
            }
        }
    }
}