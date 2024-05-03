//
// Created by masoud on 4/29/24.
//

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <glog/logging.h>
#include <opencv2/calib3d.hpp>
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

using namespace std;


namespace NAV24::FE {

    ObjTracking::ObjTracking(const ChannelPtr &pChannel) : FrontEnd(pChannel), mbInitialized(false),
        mvpParamHolder(), mMapName(), mTrajectory(), mLastImPoint(0, 0) {

        mpObjTracker = make_shared<OP::ObjTrYoloOnnx>(pChannel);
    }

    void ObjTracking::receive(const MsgPtr &msg) {

        if (msg) {
            if (msg->getTopic() == ObjTracking::TOPIC) {

                if (dynamic_pointer_cast<MsgConfig>(msg)) {
                    // Configure Front-End
                    auto pMsgConfig = dynamic_pointer_cast<MsgConfig>(msg);
                    ParamPtr pParams = pMsgConfig->getConfig();
                    if (pParams) {
                    }
                    if (!mbInitialized) {
                        this->initialize();
                    }
                }
            }
            if (msg->getTopic() == FrontEnd::TOPIC) {

                this->handleImageMsg(msg);

                if (msg && dynamic_pointer_cast<MsgType<cv::Point2f>>(msg)) {
                    auto msgPtObs = dynamic_pointer_cast<MsgType<cv::Point2f>>(msg);
                    mLastImPoint = msgPtObs->getData();
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
        }
    }

    //ParamPtr ObjTracking::getDefaultParameters(std::vector<ParamPtr> &vpParamContainer) { return {}; }

    void ObjTracking::initialize() {

        // Create a tracking map
        // Never store a local map or anything else (leave this to each manager)
        mMapName = "world0";
        auto msgCreateMap = make_shared<Message>(Atlas::TOPIC, mMapName, FCN_MAP_CREATE);
        mpChannel->publish(msgCreateMap);

        // Create a point trajectory (fixed camera)
        mTrajectory = "cam0";
        auto msgCreateTraj = make_shared<Message>(TrajManager::TOPIC,mTrajectory, FCN_TRJ_CREATE);
        mpChannel->publish(msgCreateTraj);

        // Request world0:cam0 relation from system
        auto msgGetRel = make_shared<MsgRequest>(System::TOPIC, "world0:cam0",
                                                 FCN_GET_TRANS, shared_from_this());
        mpChannel->publish(msgGetRel);

        // Add it to point trajectory
        auto msgAddPose = make_shared<MsgType<PosePtr>>(TrajManager::TOPIC, "cam0",
                FCN_TRJ_POSE_ADD, mpTwc);
        mpChannel->publish(msgAddPose);

        // Load YOLOv8 Tracker
        MsgPtr msgYoloOpParams = make_shared<MsgRequest>(ParameterServer::TOPIC,string(PARAM_OP) + "/0",
                                                         FCN_PS_REQ, mpObjTracker);
        mpChannel->publish(msgYoloOpParams);
        // todo: you normally want to address a dataset by the name in a component's interface
        MsgPtr msgYoloModelPath = make_shared<MsgRequest>(DataStore::TOPIC, TAG_DS_GET_PATH_MODEL,
                                                          FCN_DS_REQ, mpObjTracker);
        mpChannel->publish(msgYoloModelPath);

        // Load camera's calib parameters
        auto msgReqCalib = make_shared<MsgRequest>(Sensor::TOPIC, "",
                                                   FCN_CAM_GET_CALIB, shared_from_this());
        mpChannel->publish(msgReqCalib);

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
                //img = cv::imread("", cv::IMREAD_UNCHANGED);
                cv::Mat gray;
                cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

                // Detect and track object in images
                auto start = chrono::high_resolution_clock::now();
                mpObjTracker->receive(msg);
                auto stop = chrono::high_resolution_clock::now();
                auto duration = duration_cast<chrono::microseconds>(stop - start);
//                cout << "Time taken by YOLO obj detection: " << duration.count() << " microseconds" << endl;

                // Back-project image coords to find world loc
                Eigen::Vector3d Pw;
                auto img_x = mLastImPoint.x, img_y = mLastImPoint.y;
                auto img_w = static_cast<float>(gray.cols), img_h = static_cast<float>(gray.rows);
                if (mpCalib && img_x > 0 && img_x < img_w && img_y > 0 && img_y < img_h) {

                    // Get undistorted, normalized point
                    cv::Point2f undistPt = mpCalib->undistPoint(mLastImPoint);
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
                auto msgSerial = make_shared<Message>(Output::TOPIC, locStr.str(), FCN_SER_WRITE);
                mpChannel->publish(msgSerial);

                // Show results
                cv::putText(img, locStr.str(), mLastImPoint, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                            cv::Scalar(0, 255, 0), 2);
                pImage = make_shared<ImageTs>(img, pImage->mTimeStamp, pImage->mPath);
                auto msgImShow = make_shared<MsgSensorData>(Output::TOPIC, pImage);
                mpChannel->publish(msgImShow);
            }
        }
    }

} // NAV24::FE