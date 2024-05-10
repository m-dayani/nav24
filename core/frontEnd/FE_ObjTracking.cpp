//
// Created by masoud on 4/29/24.
//

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <glog/logging.h>
#include <Eigen/Eigen>
#include <chrono>

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

using namespace std;


namespace NAV24::FE {

    ObjTracking::ObjTracking(const ChannelPtr &pChannel) : FrontEnd(pChannel), mbInitialized(false),
        mvpParamHolder(), mMapName(), mTrajectory(), mLastImPoint(0, 0), mMtxLastPt(), mvpThTrackers(),
        mbYoloUpdated(false), mMtxYoloDet(), mbTrInitBbox(false), mbTrInitFrame(false) {

        mpYoloDetector = make_shared<OP::ObjTrYoloOnnx>(pChannel);
        mpObjTracker = make_shared<OP::ObjTrackingCv>(pChannel);
    }

    void ObjTracking::receive(const MsgPtr &msg) {

        if (msg) {
            if (msg->getTopic() == ObjTracking::TOPIC) {

                if (dynamic_pointer_cast<MsgConfig>(msg)) {
                    // Configure Front-End
                    auto pMsgConfig = dynamic_pointer_cast<MsgConfig>(msg);
                    ParamPtr pParams = pMsgConfig->getConfig();
                    if (pParams) {
                        // todo: get some params like CV tracker option
                    }
                    if (!mbInitialized) {
                        this->setup(msg);
                    }
                }
            }
            if (dynamic_pointer_cast<MsgSensorData>(msg)) {
                this->handleImageMsg(msg);
            }
            if (msg->getTopic() == FrontEnd::TOPIC) {

                //this->handleImageMsg(msg);

                if (dynamic_pointer_cast<MsgType<cv::Point2f>>(msg)) {
                    auto msgPtObs = dynamic_pointer_cast<MsgType<cv::Point2f>>(msg);
                    mMtxLastPt.lock();
                    mLastImPoint = msgPtObs->getData();
                    mMtxLastPt.unlock();
                }
                if (dynamic_pointer_cast<MsgType<cv::Rect2f>>(msg)) {
                    auto msgRect = dynamic_pointer_cast<MsgType<cv::Rect2f>>(msg);
                    mMtxYoloDet.lock();
                    mYoloDet = msgRect->getData();
                    mMtxYoloDet.unlock();
                    mMtxLastPt.lock();
                    mLastImPoint = OP::ObjTracking::find_center(mYoloDet);
                    mMtxLastPt.unlock();
                    mbYoloUpdated = true;
                }
            }
            if (msg->getTopic() == System::TOPIC) {
                auto msgTrans = dynamic_pointer_cast<MsgType<TransPtr>>(msg);
                if (msgTrans) {
                    mpTwc = msgTrans->getData()->getPose();
                }
            }
            if (dynamic_pointer_cast<MsgType<CalibPtr>>(msg)) {
                mpCalib = dynamic_pointer_cast<MsgType<CalibPtr>>(msg)->getData();
            }
            if (dynamic_pointer_cast<MsgType<shared_ptr<thread>>>(msg)) {
                auto pThMsg = dynamic_pointer_cast<MsgType<shared_ptr<thread>>>(msg);
                mvpThTrackers.push_back(pThMsg->getData());
            }
            if (msg->getTargetId() == FCN_SYS_STOP) {
                this->stop();
            }
        }
    }

    //ParamPtr ObjTracking::getDefaultParameters(std::vector<ParamPtr> &vpParamContainer) { return {}; }

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

        // Load YOLOv8 Tracker
        this->initYoloDetector();

        // Init main tracker
        this->initMainTracker();

        // Load camera's calib parameters
        auto msgReqCalib = make_shared<MsgRequest>(ID_CH_SENSORS, shared_from_this(),
                                                   Sensor::TOPIC,FCN_CAM_GET_CALIB);
        mpChannel->send(msgReqCalib);

        mbInitialized = true;
    }

    void ObjTracking::handleImageMsg(const MsgPtr &msg) {

        if (dynamic_pointer_cast<MsgSensorData>(msg)) {
            auto msgSensor = dynamic_pointer_cast<MsgSensorData>(msg);
            auto sensorData = msgSensor->getData();

            if (sensorData && dynamic_pointer_cast<ImageTs>(sensorData)) {

                auto pImage = dynamic_pointer_cast<ImageTs>(sensorData);

                if (!pImage || pImage->mImage.empty()) {
                    DLOG(WARNING) << "CalibCamCv::handleImageMsg, empty image detected\n";
                    return;
                }

                cv::Mat img = pImage->mImage.clone();
                cv::Mat gray;
                cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

                // Detect and track object in images
                auto start = chrono::high_resolution_clock::now();
                // todo: generalize
                mpYoloDetector->receive(msg);
                auto stop = chrono::high_resolution_clock::now();
                auto duration = duration_cast<chrono::microseconds>(stop - start);
//                cout << "Time taken by YOLO obj detection: " << duration.count() << " microseconds" << endl;

                // Track with main tracker
                if (mbYoloUpdated && !mbTrInitBbox) {
                    auto msgConfig = make_shared<MsgType<cv::Rect2f>>(ID_CH_OP, mYoloDet,
                            OP::ObjTracking::TOPIC);
                    mpObjTracker->receive(msgConfig);
                    mbTrInitBbox = true;
                }
                if (!mbTrInitFrame && mbTrInitBbox) {
                    msg->setTargetId(FCN_TR_CV_INIT_OBJ);
                    mpObjTracker->receive(msg);
                    mbTrInitFrame = true;
                }
                if (mbTrInitBbox) {
                    msg->setTargetId(FCN_TR_CV_TRACK);
                    mpObjTracker->receive(msg);
                }
                if (mbYoloUpdated) {
                    mbYoloUpdated = false;
                }

                // Back-project image coords to find world loc
                mMtxLastPt.lock();
                cv::Point2f lastPoint = mLastImPoint;
                mMtxLastPt.unlock();
                Eigen::Vector3d Pw;
                auto img_x = lastPoint.x, img_y = lastPoint.y;
                auto img_w = static_cast<float>(gray.cols), img_h = static_cast<float>(gray.rows);
                if (mpCalib && img_x > 0 && img_x < img_w && img_y > 0 && img_y < img_h) {

                    // Get undistorted, normalized point
                    cv::Point2f undistPt = mpCalib->undistPoint(lastPoint);
                    cv::Point3f Pc = mpCalib->unproject(undistPt);

                    Eigen::Vector3d Pc_eig;
                    Pc_eig << Pc.x, Pc.y, Pc.z;
                    auto q_float = mpTwc->q;
                    auto t_float = mpTwc->p;
                    Eigen::Quaterniond q_eig(q_float[0], q_float[1], q_float[2], q_float[3]);
                    Eigen::Matrix3d Rwc = q_eig.toRotationMatrix();
                    Eigen::Vector3d t_wc(t_float[0], t_float[1], t_float[2]);

                    Pw = Rwc.inverse() * (Pc_eig - t_wc);
                    Pw /= Pw[2];
                }

                // Send a coord message to serial output
                stringstream locStr;
                locStr << " " << Pw.x() << " " << Pw.y();
                auto msgSerial = make_shared<Message>(ID_TP_OUTPUT, Output::TOPIC,
                                                      FCN_SER_WRITE, locStr.str());
                mpChannel->publish(msgSerial);

                // Show results
                cv::putText(img, locStr.str(), lastPoint, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                            cv::Scalar(0, 255, 0), 2);
                pImage = make_shared<ImageTs>(img, pImage->mTimeStamp, pImage->mPath);
                auto msgImShow = make_shared<MsgSensorData>(ID_TP_OUTPUT, pImage,
                                                            Output::TOPIC);
                mpChannel->publish(msgImShow);
            }
        }
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

    void ObjTracking::initYoloDetector() {

        // Retrieve required parameters and configure YOLO detector
        MsgPtr msgYoloOpParams = make_shared<MsgRequest>(ID_CH_PARAMS, mpYoloDetector, ParameterServer::TOPIC,
                                                         FCN_PS_REQ, string(PARAM_OP) + "/0");
        mpChannel->send(msgYoloOpParams);
        // todo: you normally want to address a dataset by the name in a component's interface
        MsgPtr msgYoloModelPath = make_shared<MsgRequest>(ID_CH_DS, mpYoloDetector, DataStore::TOPIC,
                                                          FCN_DS_REQ, TAG_DS_GET_PATH_MODEL);
        mpChannel->send(msgYoloModelPath);

        // Register
        //mpChannel->registerSubscriber(ID_TP_SDATA, mpYoloDetector);
        mpChannel->registerPublisher(ID_TP_FE, mpYoloDetector);

        // Run ObjTracking operator in background
        auto msgReqRun = make_shared<MsgRequest>(ID_CH_OP, shared_from_this(),
                                                 TOPIC, FCN_OBJ_TR_RUN);
        mpYoloDetector->receive(msgReqRun);
    }

    void ObjTracking::initMainTracker() {

        // todo: get this option from executable
        auto msgConfig = make_shared<Message>(ID_CH_OP, OP::ObjTracking::TOPIC,
                                              FCN_TR_CV_INIT_OPT, "2");
        mpObjTracker->receive(msgConfig);

        // Register tracker
        mpChannel->registerPublisher(ID_TP_OUTPUT, mpObjTracker);
    }

} // NAV24::FE