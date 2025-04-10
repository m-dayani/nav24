//
// Created by masoud on 2/22/25.
//

#include <functional>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <glog/logging.h>

#include "System.hpp"
#include "FE_InferenceNav.hpp"
#include "Image.hpp"
#include "Sensor.hpp"
#include "Atlas.hpp"
#include "TrajManager.hpp"
#include "Serial.hpp"
#include "ParameterBlueprint.h"
#include "Camera.hpp"
#include "Point3D.hpp"

using namespace std;


namespace NAV24::FE {

#define FRAME_BUFF_MAX_SIZE 100

    InferenceNav::InferenceNav(const ChannelPtr &pChannel) : FrontEnd(pChannel), mbInitialized(false),
            mMapName(), mTrajectory(), mpTempParam(), mvpParamHolder(), mvpThTrackers(),
            mmpFrameBuffer() {

        //mpYoloDetector = make_shared<OP::ObjTrYoloOnnx>(pChannel);
        //mpObjTracker = make_shared<OP::ObjTrackingCv>(pChannel);

        //mHwc = Eigen::Matrix3d::Identity();
    }

    void InferenceNav::receive(const MsgPtr &msg) {

        if (msg) {

            if (dynamic_pointer_cast<MsgSensorData>(msg)) {
                this->handleImageMsg(msg);
            }
            if (dynamic_pointer_cast<MsgType<OB::ObsTimed>>(msg)) {
                //DLOG(INFO) << "FE::ObjTracking::receive, received correction message\n";
                auto msgPtObs = dynamic_pointer_cast<MsgType<OB::ObsTimed>>(msg);
                // correct observations
                this->correctObservation(msgPtObs->getData());
            }
            if (dynamic_pointer_cast<MsgType<CalibPtr>>(msg)) {
                mpCalib = dynamic_pointer_cast<MsgType<CalibPtr>>(msg)->getData();
            }
            if (dynamic_pointer_cast<MsgType<shared_ptr<thread>>>(msg)) {
                auto pThMsg = dynamic_pointer_cast<MsgType<shared_ptr<thread>>>(msg);
                mvpThTrackers.push_back(pThMsg->getData());
            }
            if (dynamic_pointer_cast<MsgConfig>(msg)) {
                mpTempParam = dynamic_pointer_cast<MsgConfig>(msg)->getConfig();
            }
            if (msg->getTopic() == InferenceNav::TOPIC) {

                if (dynamic_pointer_cast<MsgConfig>(msg)) {
                    // Configure Front-End
                    auto pMsgConfig = dynamic_pointer_cast<MsgConfig>(msg);
                    ParamPtr pParams = pMsgConfig->getConfig();
                    if (pParams) {
                        // todo: Get some params like CV tracker option here
                        //auto pTrType = dynamic_pointer_cast<ParamType<string>>(pParams);
                        //mTrType = (pTrType) ? pTrType->getValue() : FE_TR_TYPE_CV_ONLY;
                    }
                    if (!mbInitialized) {
                        this->setup(msg);
                    }
                }
            }
            if (msg->getTopic() == System::TOPIC) {
                auto msgTrans = dynamic_pointer_cast<MsgType<PosePtr>>(msg);
                if (msgTrans) {
                    mpTwc = msgTrans->getData()->inverse();
//                    this->loadHomoFromPose(msgTrans->getData());

                    // show the transform
                    auto msgShowTrans = make_shared<MsgType<PosePtr>>(ID_TP_OUTPUT, mpTwc, Output::TOPIC);
                    mpChannel->publish(msgShowTrans);
                }
            }
            if (msg->getTargetId() == FCN_SYS_STOP) {
                this->stop();
            }
        }
    }

    void InferenceNav::setup(const MsgPtr &) {

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
        auto fp = [this](auto && PH1) { receive(std::forward<decltype(PH1)>(PH1)); };
        auto msgGetRel = make_shared<MsgRequest>(ID_CH_SYS, fp,
                                                 System::TOPIC,FCN_GET_TRANS, "world0:cam0");
        mpChannel->send(msgGetRel);

        // Add it to point trajectory
        auto msgAddPose = make_shared<MsgType<PosePtr>>(ID_CH_TRAJECTORY, mpTwc,
                                                        TrajManager::TOPIC,FCN_TRJ_POSE_ADD, "cam0");
        mpChannel->send(msgAddPose);

        // Init Operators
        this->initOperators();

        // Load camera's calib parameters
        auto msgReqCalib = make_shared<MsgRequest>(ID_CH_SENSORS, fp,
                                                   Sensor::TOPIC,FCN_CAM_GET_CALIB);
        mpChannel->send(msgReqCalib);

        mbInitialized = true;
    }

    void InferenceNav::stop() {
        MsgCallback::stop();
//        auto msgStop = make_shared<Message>(ID_CH_OP, FE::InferenceNav::TOPIC,
//                                            FCN_OBJ_TR_STOP);
//
//        for (const auto& pTh : mvpThTrackers) {
//            if (pTh) {
//                pTh->join();
//            }
//        }
    }

    void InferenceNav::initOperators() {

        // Get operator parameters
        auto fp = [this](auto && PH1) { receive(std::forward<decltype(PH1)>(PH1)); };
        MsgPtr msgOpParams = make_shared<MsgRequest>(ID_CH_PARAMS, fp, ParameterServer::TOPIC,
                                                     FCN_PS_REQ, string(PARAM_OP));
        mpChannel->send(msgOpParams);
        if (mpTempParam && mpTempParam->getName() == "OP") {
            for (const auto& pOpParamPair : mpTempParam->getAllChildren()) {
                string key = pOpParamPair.first;
                auto pOpParam = pOpParamPair.second.lock();

                auto pTracker = OP::ObjDet::createDetector(pOpParam, mpChannel);
                if (pTracker) {
                    mvpObjDetectors.push_back(pTracker);
                }
            }
        }
    }

    void InferenceNav::handleImageMsg(const MsgPtr &msg) {

        if (!dynamic_pointer_cast<MsgSensorData>(msg)) {
            DLOG(WARNING) << "InferenceNav::handleImageMsg, msg is not SensorData\n";
            return;
        }

        auto msgSensor = dynamic_pointer_cast<MsgSensorData>(msg);
        auto sensorData = msgSensor->getData();

        if (!sensorData || !dynamic_pointer_cast<ImageTs>(sensorData)) {
            DLOG(WARNING) << "InferenceNav::handleImageMsg, SensorData has no image msg\n";
            return;
        }

        auto pImage = dynamic_pointer_cast<ImageTs>(sensorData);

        if (!pImage || pImage->mImage.empty()) {
            DLOG(WARNING) << "InferenceNav::handleImageMsg, empty image detected\n";
            return;
        }

        cv::Mat img = pImage->mImage.clone();
        cv::Mat imgShow = img.clone();
        if (imgShow.channels() == 1) {
            cv::cvtColor(imgShow, imgShow, cv::COLOR_GRAY2BGR);
        }
        if (img.cols > 640) {
            cv::Size newSize(640, (int) (640.f / (float) img.cols * (float) img.rows));
            cv::resize(imgShow, imgShow, newSize);
        }
        auto pImgShow = make_shared<ImageTs>(imgShow, pImage->mTimeStamp, pImage->mPath);
        imgShow = pImgShow->mImage;
        auto msgImShow = make_shared<MsgSensorData>(ID_TP_SDATA, pImgShow,
                                                    Output::TOPIC);

        if (mImgSize.empty()) {
            mImgSize = cv::Size(img.cols, img.rows);
        }

        // Frame Creation
//        this->createAndInsertFrame(pImage);

        // Inference
        vector<OB::ObsPtr> vpObservations;
        for (const auto& pDetector : mvpObjDetectors) {
            pDetector->detect(pImgShow, vpObservations);
        }

        // Draw results
        for (const auto& pObs : vpObservations) {
            pObs->draw(imgShow);
        }

        // Publish results
        mpChannel->publish(msgImShow);
    }

    void InferenceNav::showResults(const ImagePtr& pImg, const cv::Point2f& lastPoint, const WO::WoPtr &Pw) {

        if (pImg && dynamic_pointer_cast<ImageTs>(pImg)) {

            auto pImage = dynamic_pointer_cast<ImageTs>(pImg);

            cv::Mat img = pImage->mImage.clone();

            auto pt3d = dynamic_pointer_cast<WO::Point3D>(Pw);
            ostringstream locStr;
            locStr << "(" << pt3d->getPoint().x << ", " << pt3d->getPoint().y << ")";

            cv::putText(img, locStr.str(), lastPoint, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(0, 255, 0), 2);
            cv::drawMarker(img, lastPoint, cv::Scalar(0, 0, 255));

            pImage = make_shared<ImageTs>(img, pImage->mTimeStamp, pImage->mPath);
            auto msgImShow = make_shared<MsgSensorData>(ID_TP_OUTPUT, pImage,
                                                        Output::TOPIC);
            mpChannel->publish(msgImShow);

            auto msgShowPw = make_shared<MsgType<WO::WoPtr>>(ID_TP_OUTPUT, Pw, Output::TOPIC);
            mpChannel->publish(msgShowPw);
        }
    }

    FramePtr InferenceNav::getLastFrame() {
        if (!mmpFrameBuffer.empty()) {
            return mmpFrameBuffer.rbegin()->second;
        }
        return nullptr;
    }

    std::shared_ptr<OB::BBox> InferenceNav::getLastObservation() {

        shared_ptr<OB::BBox> pBbox = nullptr;
        auto pFrame = this->getLastFrame();
        if (pFrame) {
            auto pObsBbox = pFrame->getObservations().back();
            if (static_pointer_cast<OB::BBox>(pObsBbox)) {
                pBbox = static_pointer_cast<OB::BBox>(pObsBbox);
            }
        }
        return pBbox;
    }

    OB::ObsPtr InferenceNav::updateObservation() {
        // in the simplest form, current observation is equal to the last observation
        return this->getLastObservation();
    }

    void InferenceNav::correctObservation(const OB::ObsTimed& obsTimed) {

        auto ts = obsTimed.first;
        auto pObs = obsTimed.second;

        if (mmpFrameBuffer.count(ts) > 0) {
            //DLOG(INFO) << "FE::ObjTracking::correctObservation, retrieved frame\n";
            // find the frame
            auto pFrame = mmpFrameBuffer[ts];
            if (pFrame) {
                // correct
                vector<OB::ObsPtr> vpObs{pObs};
                pFrame->setObservations(vpObs);
                //DLOG(INFO) << "FE::ObjTracking::correctObservation, corrected frame observation at " << ts << "\n";

                ImagePtr pImage = nullptr;
                if (static_pointer_cast<FrameImgMono>(pFrame)) {
                    pImage = static_pointer_cast<FrameImgMono>(pFrame)->getImage();
                }

//                this->processObservation(obsTimed, pImage);
            }
        }
    }

    void InferenceNav::createAndInsertFrame(const ImagePtr &pImg) {

        // observations are updated either from operators or EKF updates
        auto pObsCurr = updateObservation();
//        mpLastFrame = this->creatNewFrame(pImg, pObsCurr);

        std::shared_ptr<FrameImgMono> pFrame = nullptr;
        if (pImg && dynamic_pointer_cast<ImageTs>(pImg)) {
            auto ts = dynamic_pointer_cast<ImageTs>(pImg)->mTimeStamp;
            vector<OB::ObsPtr> vpObs{pObsCurr};
            pFrame = make_shared<FrameImgMono>(ts, mpTwc, vpObs, pImg);
        }
        mpLastFrame = pFrame;
        if (pFrame == nullptr) {
//            return pFrame;
            return;
        }
        if (mmpFrameBuffer.count(static_cast<long>(pFrame->getTs())) <= 0) {
            mmpFrameBuffer.insert(make_pair(pFrame->getTs(), pFrame));
        }
        if (mmpFrameBuffer.size() > FRAME_BUFF_MAX_SIZE) {
            mmpFrameBuffer.erase(mmpFrameBuffer.begin()->first);
        }

//        return pFrame;
    }

} // NAV24::FE
