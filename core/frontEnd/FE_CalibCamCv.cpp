//
// Created by masoud on 2/11/24.
//

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <glog/logging.h>

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

        cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001);
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
                        this->initialize();
                    }
                }

                int action = msg->getTargetId();
                if (action == FCN_FE_CAM_CALIB) {
                    this->calibrate();
                }
            }
            if (msg->getTopic() == FrontEnd::TOPIC) {
                this->handleImageMsg(msg);
            }
        }
    }

    void CalibCamCv::initialize() {

        // Create a calibration map
        // Never store a local map or anything else (leave this to each manager)
        mCalibMap = "calib_world" + to_string(mMapCnt++);
        auto msgCreateMap = make_shared<Message>(Atlas::TOPIC,mCalibMap,FCN_MAP_CREATE);
        mpChannel->publish(msgCreateMap);

        // Create a point trajectory (fixed camera)
        mTrajectory = "calib_traj" + to_string(mTrajCnt++);
        auto msgCreateTraj = make_shared<Message>(TrajManager::TOPIC,mTrajectory, FCN_TRJ_CREATE);
        mpChannel->publish(msgCreateTraj);

        // Initialize the grid map
        for (int y = 0; y < mGridSize.height; y++) {
            for (int x = 0; x < mGridSize.width; x++) {
                auto pt3d = make_shared<WO::Point3D>(x, y, 0);
                mvpPts3D.push_back(pt3d);
            }
        }
        // Always add parameters in bulk (for efficiency)
        auto msgAddMapPts = make_shared<MsgType<vector<WO::woPtr>>>(Atlas::TOPIC, mCalibMap,
                                                                    FCN_MAP_ADD_WO, mvpPts3D);
        mpChannel->publish(msgAddMapPts);

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
                bool res = mpOpChBoardDetCv->process(gray, vpCorners);

                if (res) {
                    // Create and add a frame
                    pImage->mImage = cv::Mat();
                    PosePtr pPose = make_shared<Pose>();
                    auto pFrame = make_shared<FrameImgMono>(pImage->mTimeStamp, pPose, vpCorners, pImage);
                    mvpFrames.push_back(pFrame);


                }

                // Show results
//                    cv::putText(img, "Successful", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX,
//                                1.5, cv::Scalar(0, 255, 0), 2);
                //cv::drawChessboardCorners(img, mGridSize, vCorners, res);
            }
        }
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
        pProblem->solved = false;

        // Solve the problem in the back-end
        // Normally you would send a problem message to a back-end
        auto solver = make_shared<BE::CalibCamCv>();
        solver->solve(pProblem);

        // Update the results
        if (pProblem->solved) {

            // Update calibration parameters: K, D
            auto pCalibParam = Calibration::getCalibParams(pProblem->mK, pProblem->mDistCoeffs,
                                                           "radial-tangential", mvpParamHolder);
            auto msgCalibConf = make_shared<MsgConfig>(ParameterServer::TOPIC, pCalibParam);
            // todo: you normally get param keys from the system
            msgCalibConf->setMessage(string(PARAM_CAM) + "/0/calib");
            msgCalibConf->setTargetId(FCN_PS_CONF);
            mpChannel->publish(msgCalibConf);

            // Update camera-world trans from last frame
            auto pTransParam = Transformation::getTransParam("world0", "cam0", 0.0,
                                                             mvpFrames.back()->getPose(), mvpParamHolder);
            auto msgTransConf = make_shared<MsgConfig>(ParameterServer::TOPIC, pTransParam);
            msgTransConf->setMessage(string(PARAM_REL) + "/0");
            msgTransConf->setTargetId(FCN_PS_CONF);
            mpChannel->publish(msgTransConf);

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

} // NAV24::FE