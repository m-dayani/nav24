//
// Created by masoud on 8/29/24.
//

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

using namespace std;

namespace NAV24::FE {

    SlamMonoV::SlamMonoV(const ChannelPtr &pChannel) : FrontEnd(pChannel),
        mpOrbDetector(), mpTempParam(), mbInitialized(false),
        mpFirstFrame(), mpLastFrame(), mpCurrFrame(),
        mpFtTracks(), mpOrbMatcher(), mpCalib() {

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
                cv::Mat gray;
//                cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

//                if (!mpDummyGrid) {
//                    vector<OB::ObsPtr> vpObs;
//                    mpDummyGrid = make_shared<OB::FeatureGrid>(vpObs);
//                    vector<float> imgBounds = mpCalib->computeImageBounds(img);
//                    mpDummyGrid->setImageBounds(cv::Size(img.cols, img.rows), imgBounds);
//                }

                // Extract features
                mpCurrFrame = make_shared<FrameMonoGrid>(pImage->mTimeStamp, nullptr, vector<OB::ObsPtr>(), pImage);
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
                if (mpOrbMatcher && mpLastFrame && mpLastFrame != mpCurrFrame) {
                    int nMch = mpOrbMatcher->match(mpLastFrame, mpCurrFrame, mpFtTracks);
//                    cout << "N matches: " << nMch << endl;
                }

                mpLastFrame = mpCurrFrame;

                cv::Mat imgShow;

                // Draw detected features
                imgShow = img.clone();
//                Visualization::drawKeyPoints(imgShow, mpCurrFrame);
                Visualization::drawMatchedTracks(imgShow, mpFtTracks);

                // Show Image
                if (!imgShow.empty()) {
                    auto pImageMatch = make_shared<ImageTs>(imgShow, pImage->mTimeStamp, "");
                    auto msgImShow = make_shared<MsgSensorData>(ID_TP_OUTPUT, pImageMatch,
                                                                Output::TOPIC);
                    mpChannel->publish(msgImShow);
                }
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
            }
        }

    }
}