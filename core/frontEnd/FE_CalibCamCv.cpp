//
// Created by masoud on 2/11/24.
//

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <glog/logging.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#include "FE_CalibCamCv.hpp"
#include "Image.hpp"
#include "Sensor.hpp"
#include "Atlas.hpp"
#include "TrajManager.hpp"
#include "Problem.hpp"
#include "BE_CalibCamCv.hpp"
#include "ParameterServer.hpp"
#include "ParameterBlueprint.h"
#include "Calibration.hpp"
#include "Visualization.hpp"
#include "Output.hpp"

using namespace std;

namespace NAV24::FE {

#define DEF_CALIB_GRID_X 9
#define DEF_CALIB_GRID_Y 6
#define DEF_CALIB_GRID_S 1.f

#define PARAM_KEY_CALIB_CAM "CalibCamCv"
#define PARAM_KEY_GRID_SZ "grid_size"
#define PARAM_KEY_GRID_SC "grid_scale"

    CalibCamCv::CalibCamCv(const ChannelPtr &pChannel) : FrontEnd(pChannel),
            mGridSize(DEF_CALIB_GRID_X, DEF_CALIB_GRID_Y), mGridScale(DEF_CALIB_GRID_S),
            mbInitialized(false), mImageSize(), mvpParamHolder() {

        cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001);
        mpOpChBoardDetCv = make_shared<OP::OP_ChBoardDetCv>(mGridSize, criteria);
    }

    void CalibCamCv::receive(const MsgPtr &msg) {

        if (msg) {
            if (msg->getTopic() == CalibCamCv::TOPIC) {

                if (dynamic_pointer_cast<MsgConfig>(msg)) {
                    // Configure Front-End
                    auto pMsgConfig = dynamic_pointer_cast<MsgConfig>(msg);
                    ParamPtr pParams = pMsgConfig->getConfig();
                    if (pParams) {
                        auto pGridSz = find_param<ParamSeq<int>>(PARAM_KEY_GRID_SZ, pParams);
                        if (pGridSz) {
                            vector<int> vGridSize = pGridSz->getValue();
                            if (vGridSize.size() >= 2) {
                                mGridSize = cv::Size(vGridSize[0], vGridSize[1]);
                            }
                        }
                        auto pGridSc = find_param<ParamType<double>>(PARAM_KEY_GRID_SC, pParams);
                        if (pGridSc) {
                            mGridScale = static_cast<float>(pGridSc->getValue());
                        }
                    }
                    if (!mbInitialized) {
                        this->setup(msg);
                    }
                }

                int action = msg->getTargetId();
                if (action == FCN_FE_CAM_CALIB) {
                    this->calibrate();
                }
            }
            if (dynamic_pointer_cast<MsgSensorData>(msg)) {
                this->handleImageMsg(msg);
            }
            if (msg->getTargetId() == FCN_SHOW_LAST_FRAME) {
                this->drawFrame(mvpFrames.back());
            }
        }
    }

    void CalibCamCv::setup(const MsgPtr& msg) {

        // Create a calibration map
        // Never store a local map or anything else (leave this to each manager)
        mCalibMap = "calib_world" + to_string(mMapCnt++);
        auto msgCreateMap = make_shared<Message>(ID_CH_ATLAS, Atlas::TOPIC,
                                                 FCN_MAP_CREATE, mCalibMap);
        mpChannel->send(msgCreateMap);

        // Create a point trajectory (fixed camera)
        mTrajectory = "calib_traj" + to_string(mTrajCnt++);
        auto msgCreateTraj = make_shared<Message>(ID_CH_TRAJECTORY, TrajManager::TOPIC,
                                                  FCN_TRJ_CREATE, mTrajectory);
        mpChannel->send(msgCreateTraj);

        // Initialize the grid map
        for (int y = 0; y < mGridSize.height; y++) {
            for (int x = 0; x < mGridSize.width; x++) {
                auto pt3d = make_shared<WO::Point3D>((float)x * mGridScale, (float)y * mGridScale, 0);
                mvpPts3D.push_back(pt3d);
            }
        }
        // Always add parameters in bulk (for efficiency)
        auto msgAddMapPts = make_shared<MsgType<vector<WO::WoPtr>>>(ID_CH_ATLAS, mvpPts3D,
                                                                    Atlas::TOPIC, FCN_MAP_ADD_WO, mCalibMap);
        mpChannel->send(msgAddMapPts);

        mbInitialized = true;
    }

    void CalibCamCv::handleImageMsg(const MsgPtr &msg) {

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

                if (mImageSize.empty()) {
                    mImageSize = cv::Size(img.cols, img.rows);
                }

                // Detect and associate image observations
                // In this case the order of detected points determines the association
                vector<OB::ObsPtr> vpCorners{};
                bool res = mpOpChBoardDetCv->process(img, vpCorners);

                if (res) {
                    // Create and add a frame
                    //pImage->mImage = cv::Mat();
                    PosePtr pPose = nullptr;//make_shared<PoseSE3>();
                    auto pFrame = make_shared<FrameImgMono>(pImage->mTimeStamp, pPose, vpCorners, pImage);
                    mvpFrames.push_back(pFrame);
                }

                // Show results
                this->drawChessBoard(pImage, vpCorners, res);
            }
        }
    }

    void CalibCamCv::drawChessBoard(const ImagePtr& pImage, const vector<OB::ObsPtr>& vpCorners, bool res) {

        cv::Mat img = pImage->mImage.clone();

        string resStr = (res) ? "Success" : "Failure";
        cv::Scalar resColor = (res) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::putText(img, resStr, cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX,
                    1.5, resColor, 2);
        vector<cv::Point2f> vCorners;
        for (const auto& pObs : vpCorners) {
            auto point = dynamic_pointer_cast<OB::Point2D>(pObs);
            if (point) {
                auto ptcv = point->getPoint();
                vCorners.emplace_back(ptcv.x, ptcv.y);
            }
        }
        cv::drawChessboardCorners(img, mGridSize, vCorners, res);
        auto pImage2Show = make_shared<ImageTs>(img.clone(), -1, pImage->mPath);
        auto msgImage = make_shared<MsgSensorData>(ID_TP_OUTPUT, pImage2Show);

        //cv::imshow("ChessBoard", img);
        //cv::waitKey(0);
        mpChannel->publish(msgImage);
    }

    void CalibCamCv::calibrate() {

        if (mvpFrames.size() < 10) {
            DLOG(WARNING) << "CalibCamCv::calibrate, not enough frames to calibrate: " << mvpFrames.size() << "\n";
            return;
        }

        // Create optimization problem
        auto pProblem = make_shared<PR_CamCalibCV>();
        pProblem->mvpFrames = mvpFrames;
        pProblem->mvpWorldObjs = mvpPts3D;
        pProblem->mImgSize = mImageSize;
        pProblem->setSolved(false);

        // Solve the problem in the back-end
        // Normally you would send a problem message to a back-end
        auto solver = make_shared<BE::CalibCamCv>();
        solver->solve(pProblem);

        // Update the results
        if (pProblem->isSolved()) {

            // Update calibration parameters: K, D
            auto pCalibParam = Calibration::getCalibParams(pProblem->mK, pProblem->mDistCoeffs,
                                                           "radial-tangential", mvpParamHolder);
            mpCalib = make_shared<Calibration>(pCalibParam);
            auto msgCalibConf = make_shared<MsgConfig>(ID_CH_PARAMS, pCalibParam,
                                                       ParameterServer::TOPIC);
            // todo: you normally get param keys from the system
            msgCalibConf->setMessage(string(PARAM_CAM) + "/0/calib");
            msgCalibConf->setTargetId(FCN_PS_CONF);
            mpChannel->send(msgCalibConf);

            // Update camera-world trans from last frame
            auto pTransParam = TF::PoseSE3::getTransParam("world0", "cam0", 0.0,
                                                             mvpFrames.back()->getPose(), mvpParamHolder);
            auto msgTransConf = make_shared<MsgConfig>(ID_CH_PARAMS, pTransParam,
                                                       ParameterServer::TOPIC);
            msgTransConf->setMessage(string(PARAM_REL) + "/0");
            msgTransConf->setTargetId(FCN_PS_CONF);
            mpChannel->send(msgTransConf);

            this->drawPoseMap();
            // TODO: Notify components that parameters have changed
        }
    }

    ParamPtr CalibCamCv::getDefaultParameters(std::vector<ParamPtr>& vpParamContainer) {

        ParamPtr pParam = make_shared<Parameter>(PARAM_KEY_CALIB_CAM, nullptr, Parameter::NodeType::MAP_NODE);

        vector<int> grid_size = {DEF_CALIB_GRID_X, DEF_CALIB_GRID_Y};
        auto pGridSz = make_shared<ParamSeq<int>>(PARAM_KEY_GRID_SZ, pParam, grid_size);
        auto pGridSc = make_shared<ParamType<double>>(PARAM_KEY_GRID_SC, pParam, DEF_CALIB_GRID_S);

        pParam->insertChild(PARAM_KEY_GRID_SZ, pGridSz);
        pParam->insertChild(PARAM_KEY_GRID_SC, pGridSc);

        vpParamContainer.push_back(pParam);
        vpParamContainer.push_back(pGridSz);
        vpParamContainer.push_back(pGridSc);

        return pParam;
    }

    void CalibCamCv::drawFrame(const FramePtr &pFrame) {

        shared_ptr<FrameImgMono> pImgFr = static_pointer_cast<FrameImgMono>(pFrame);
        if (!pImgFr) {
            return;
        }

        cv::Mat imgShow = pImgFr->getImage()->mImage.clone();
        Visualization::projectMap(imgShow, pImgFr->getPose(), mpCalib, mvpPts3D);

        cv::imshow("Image Grid", imgShow);
        cv::waitKey(0);
    }

    void CalibCamCv::drawPoseMap() {

        auto msgSendMap = make_shared<MsgType<vector<WO::WoPtr>>>(ID_TP_OUTPUT, mvpPts3D, Output::TOPIC);
        auto msgSendPose = make_shared<MsgType<vector<FramePtr>>>(ID_TP_OUTPUT, mvpFrames, Output::TOPIC);

        mpChannel->publish(msgSendMap);
        mpChannel->publish(msgSendPose);
    }

} // NAV24::FE