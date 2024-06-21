//
// Created by masoud on 4/29/24.
//

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <glog/logging.h>

#include "System.hpp"
#include "FE_ObjTracking.hpp"
#include "Image.hpp"
#include "Sensor.hpp"
#include "Atlas.hpp"
#include "TrajManager.hpp"
#include "Serial.hpp"
#include "DataConversion.hpp"
#include "ParameterBlueprint.h"
#include "Camera.hpp"
#include "OP_ObjTrackingCv.hpp"
#include "OP_ObjTrackingYolo.hpp"
#include "Point3D.hpp"

using namespace std;


namespace NAV24::FE {

#define FRAME_BUFF_MAX_SIZE 100

    ObjTracking::ObjTracking(const ChannelPtr &pChannel) : FrontEnd(pChannel), mbInitialized(false),
        mvpParamHolder(), mMapName(), mTrajectory(), mvpThTrackers(), mbTrInit(false),
        mTsYoloUpdate(-1), mpTempParam(), mmpFrameBuffer() {

        //mpYoloDetector = make_shared<OP::ObjTrYoloOnnx>(pChannel);
        //mpObjTracker = make_shared<OP::ObjTrackingCv>(pChannel);

        //mHwc = Eigen::Matrix3d::Identity();
    }

    void ObjTracking::receive(const MsgPtr &msg) {

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
            if (msg->getTopic() == ObjTracking::TOPIC) {

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
                    this->loadHomoFromPose(msgTrans->getData());

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

    void ObjTracking::setup(const MsgPtr &msg) {

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
        auto msgGetRel = make_shared<MsgRequest>(ID_CH_SYS, shared_from_this(),
                                                 System::TOPIC,FCN_GET_TRANS, "world0:cam0");
        mpChannel->send(msgGetRel);

        // Add it to point trajectory
        auto msgAddPose = make_shared<MsgType<PosePtr>>(ID_CH_TRAJECTORY, mpTwc,
                TrajManager::TOPIC,FCN_TRJ_POSE_ADD, "cam0");
        mpChannel->send(msgAddPose);

        // Init Operators
        this->initOperators();

        // Load camera's calib parameters
        auto msgReqCalib = make_shared<MsgRequest>(ID_CH_SENSORS, shared_from_this(),
                                                   Sensor::TOPIC,FCN_CAM_GET_CALIB);
        mpChannel->send(msgReqCalib);

        mbInitialized = true;
    }

    void ObjTracking::stop() {
        MsgCallback::stop();
        auto msgStop = make_shared<Message>(ID_CH_OP, OP::ObjTracking::TOPIC,
                                            FCN_OBJ_TR_STOP);
        if (mpObjTracker) {
            mpObjTracker->receive(msgStop);
        }
        if (mpYoloDetector) {
            mpYoloDetector->receive(msgStop);
        }
        for (const auto& pTh : mvpThTrackers) {
            if (pTh) {
                pTh->join();
            }
        }
    }

    void ObjTracking::initOperators() {

        // Get operator parameters
        MsgPtr msgOpParams = make_shared<MsgRequest>(ID_CH_PARAMS, shared_from_this(), ParameterServer::TOPIC,
                                                         FCN_PS_REQ, string(PARAM_OP));
        mpChannel->send(msgOpParams);
        if (mpTempParam && mpTempParam->getName() == "OP") {
            for (const auto& pOpParamPair : mpTempParam->getAllChildren()) {
                string key = pOpParamPair.first;
                auto pOpParam = pOpParamPair.second.lock();

                auto pTracker = OP::ObjTracking::createTracker(pOpParam, mpChannel);
                if (pTracker) {
                    string trackerName = pTracker->getName();
                    if (trackerName == OP_OTR_NAME_YOLO_ONNX || trackerName == OP_OTR_NAME_YOLO_PY) {
                        mpYoloDetector = pTracker;
                        // Register
                        mpChannel->registerPublisher(ID_TP_FE, mpYoloDetector);
                        // Run ObjTracking operator in background
                        auto msgReqRun = make_shared<MsgRequest>(ID_CH_OP, shared_from_this(),
                                                                 TOPIC, FCN_OBJ_TR_RUN);
                        mpYoloDetector->receive(msgReqRun);
                    }
                    if (trackerName == OP_OTR_NAME_CV) {
                        mpObjTracker = pTracker;
                        mpChannel->registerPublisher(ID_TP_OUTPUT, mpObjTracker);
                        auto msgReqRun = make_shared<MsgRequest>(ID_CH_OP, shared_from_this(),
                                                                 TOPIC, FCN_OBJ_TR_RUN);
                        mpObjTracker->receive(msgReqRun);
                    }
                }
            }
        }

    }

    void ObjTracking::handleImageMsg(const MsgPtr &msg) {

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
                cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

                if (mImgSize.empty()) {
                    mImgSize = cv::Size(gray.cols, gray.rows);
                }

                // Frame Creation
                this->createAndInsertFrame(pImage);

                // Initialize tracking
                this->initialize();

                // Detect and track object in images
                this->track(this->getLastFrame());
            }
        }
    }

    void ObjTracking::track(const FramePtr& pFrame) {

        auto msg = make_shared<MsgType<FramePtr>>(ID_CH_OP, pFrame, OP::ObjTracking::TOPIC);

        // Track with yolo detector
        if (mpYoloDetector) {
            mpYoloDetector->receive(msg);
        }
        // Track with main tracker
        if (mpObjTracker) {
            //msg->setTargetId(FCN_TR_CV_INIT_OBJ);
            //mpObjTracker->receive(msg);
            //mbTrInitFrame = true;
            msg->setTargetId(FCN_TR_CV_TRACK);
            mpObjTracker->receive(msg);
        }
    }

    /*Eigen::Vector3d ObjTracking::unproject(const cv::Point2f& lastPoint) {

        Eigen::Vector3d Pw;
        auto img_x = lastPoint.x, img_y = lastPoint.y;
        auto img_w = static_cast<float>(mImgSize.width), img_h = static_cast<float>(mImgSize.height);
        if (mpCalib && img_x > 0 && img_x < img_w && img_y > 0 && img_y < img_h) {

            // Get undistorted, normalized point
//                    cv::Point2f lp(lastPoint.y, lastPoint.x);
            // todo: it seems point coords are reverse -> check this
            cv::Point2f undistPt = mpCalib->undistPoint(lastPoint);
            cv::Point3f Pc(undistPt.x, undistPt.y, 1); //mpCalib->unproject(undistPt);//Pc.z = 0;
            Eigen::Vector3d Pc_eig = Converter::toVector3d(Pc);

            //auto Pc_homo = PoseSE3::euler2homo(Pc_eig);
            //auto Pw_homo = mpTwc->transform(Pc_homo);
            //Pw = PoseSE3::homo2euler(Pw_homo);
            // To get the correct Pw, you must use Homography (not Twc)
            Pw = mHwc * Pc_eig;
            Pw /= Pw[2];

//                    DLOG(INFO) << "Pt_undist: " << undistPt << "\n";
//                    DLOG(INFO) << "Pc_homo: " << Pc_homo << "\n";
//                    DLOG(INFO) << "Pw_homo: " << Pw_homo << "\n";
//                    DLOG(INFO) << "Pw: " << Pw << "\n";
//                    DLOG(INFO) << "H_1: " << mHwc << "\n";
//                    DLOG(INFO) << "Twc: " << mpTwc->getPose() << "\n";
//                    DLOG(INFO) << "-------------------------\n";
        }
        return Pw;
    }*/

    void ObjTracking::sendCoords(const WO::woPtr &Pw) {

        auto pt3d = static_pointer_cast<WO::Point3D>(Pw);
        stringstream locStr;
        locStr << " " << pt3d->getPoint().x << " " << pt3d->getPoint().y;
        auto msgSerial = make_shared<Message>(ID_TP_OUTPUT, Output::TOPIC,
                                              FCN_SER_WRITE, locStr.str());
        mpChannel->publish(msgSerial);
    }

    void ObjTracking::showResults(const ImagePtr& pImg, const cv::Point2f& lastPoint, const WO::woPtr &Pw) {

        if (pImg && dynamic_pointer_cast<ImageTs>(pImg)) {

            auto pImage = dynamic_pointer_cast<ImageTs>(pImg);

            cv::Mat img = pImage->mImage.clone();

            auto pt3d = static_pointer_cast<WO::Point3D>(Pw);
            ostringstream locStr;
            locStr << "(" << pt3d->getPoint().x << ", " << pt3d->getPoint().y << ")";

            cv::putText(img, locStr.str(), lastPoint, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(0, 255, 0), 2);
            cv::drawMarker(img, lastPoint, cv::Scalar(0, 0, 255));

            pImage = make_shared<ImageTs>(img, pImage->mTimeStamp, pImage->mPath);
            auto msgImShow = make_shared<MsgSensorData>(ID_TP_OUTPUT, pImage,
                                                        Output::TOPIC);
            mpChannel->publish(msgImShow);

            auto msgShowPw = make_shared<MsgType<WO::woPtr>>(ID_TP_OUTPUT, Pw, Output::TOPIC);
            mpChannel->publish(msgShowPw);
        }
    }

    FramePtr ObjTracking::getLastFrame() {
        if (!mmpFrameBuffer.empty()) {
            return mmpFrameBuffer.rbegin()->second;
        }
        return nullptr;
    }

    std::shared_ptr<OB::BBox> ObjTracking::getLastObservation() {

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

    OB::ObsPtr ObjTracking::updateObservation() {
        // in the simplest form, current observation is equal to the last observation
        return this->getLastObservation();
    }

    void ObjTracking::correctObservation(const OB::ObsTimed& obsTimed) {

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

                this->processObservation(obsTimed, pImage);
            }
        }
    }

    void ObjTracking::createAndInsertFrame(const ImagePtr &pImg) {

        // observations are updated either from operators or EKF updates
        auto pObsCurr = updateObservation();
        mpLastFrame = this->creatNewFrame(pImg, pObsCurr);
        this->insertFrame(mpLastFrame);
    }

    std::shared_ptr<FrameImgMono> ObjTracking::creatNewFrame(const ImagePtr &pImg, const OB::ObsPtr &pObs) {

        std::shared_ptr<FrameImgMono> pFrame = nullptr;
        if (pImg && dynamic_pointer_cast<ImageTs>(pImg)) {
            auto ts = dynamic_pointer_cast<ImageTs>(pImg)->mTimeStamp;
            vector<OB::ObsPtr> vpObs{pObs};
            pFrame = make_shared<FrameImgMono>(ts, mpTwc, vpObs, pImg);
        }
        return pFrame;
    }

    void ObjTracking::insertFrame(const FramePtr &frame) {
        if (mmpFrameBuffer.count(static_cast<long>(frame->getTs())) <= 0) {
            mmpFrameBuffer.insert(make_pair(frame->getTs(), frame));
        }
        if (mmpFrameBuffer.size() > FRAME_BUFF_MAX_SIZE) {
            mmpFrameBuffer.erase(mmpFrameBuffer.begin()->first);
        }
    }

    void ObjTracking::initialize() {

        if (mbTrInit) {
            return;
        }

        if (mpYoloDetector) {
            // if YOLO detector is defined, detect automatically
            if (mTsYoloUpdate >= 0) {
                // update dependent trackers
                if (mmpFrameBuffer.count(mTsYoloUpdate)) {
                    auto msgConf = make_shared<MsgType<FramePtr>>(ID_CH_OP,
                            mmpFrameBuffer[mTsYoloUpdate],OP::ObjTracking::TOPIC);
                    msgConf->setMessage("init");
                    mpObjTracker->receive(msgConf);
                    //mbTrInit = true;
                    //DLOG(INFO) << "FE::ObjTracking::initialize, Initialized dependent tracker at " << mTsYoloUpdate << "\n";
                    //mTsYoloUpdate = -1;
                }
            }
        }
    }

    void ObjTracking::processObservation(const OB::ObsTimed &pObsPair, const ImagePtr& pImage) {

        auto pObs = pObsPair.second;
        //long ts = pObsPair.first;

        if (!pObs || !static_pointer_cast<OB::BBox>(pObs)) {
            return;
        }

        cv::Point2f lastPoint = static_pointer_cast<OB::BBox>(pObs)->getCenter();

        // Back-project image coords to find world loc
        // create and insert a map point
        auto pPtObs = make_shared<OB::Point2D>(lastPoint.x, lastPoint.y);
        auto pWo = Camera::unproject(pPtObs, mHwc, mpCalib);
        auto pt3d = static_pointer_cast<WO::Point3D>(pWo);
        auto ptcv = pt3d->getPoint();
        pWo = make_shared<WO::Point3D>(ptcv.x/ptcv.z, ptcv.y/ptcv.z, 1);
        vector<WO::woPtr> vpPts3D = {pWo};

        pWo->setObservation(pObs);
        pObs->setWorldObject(pWo);

        auto msgAddMapPts = make_shared<MsgType<vector<WO::woPtr>>>(ID_CH_ATLAS, vpPts3D,
                                                                    Atlas::TOPIC, FCN_MAP_ADD_WO, mMapName);
        mpChannel->send(msgAddMapPts);

        // Send a coord message to serial output
        this->sendCoords(pWo);

        if (pImage) {
            // Show results
            this->showResults(pImage, lastPoint, pWo);
        }
    }

    void ObjTracking::loadHomoFromPose(const PosePtr &pPose_cw) {

        if (pPose_cw) {
            Eigen::Matrix4d Tcw = pPose_cw->getPose();
            Eigen::Matrix3d Hcw = Eigen::Matrix3d::Identity();
            Hcw.block<3, 2>(0, 0) = Tcw.block<3, 2>(0, 0);
            Hcw.block<3, 1>(0, 2) = Tcw.block<3, 1>(0, 3);
            mHwc = make_shared<TF::Trans2D>(pPose_cw->getRef(), pPose_cw->getTarget(),
                                            pPose_cw->getTimestamp(), Hcw.inverse());
        }
        else {
            mHwc = make_shared<TF::Trans2D>("c", "w", -1, Eigen::Matrix3d::Identity());
        }
    }

} // NAV24::FE