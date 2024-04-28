//
// Created by masoud on 2/11/24.
//

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "FE_CalibCamCv.hpp"
#include "Image.hpp"
#include "Sensor.hpp"

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
            mbInitialized(false) {}

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
            }
            if (msg->getTopic() == FrontEnd::TOPIC) {
                this->handleImageMsg(msg);
            }
        }
    }

    void CalibCamCv::initialize() {

        // TODO: Initialize the grid map

        mbInitialized = true;
    }

    void CalibCamCv::handleImageMsg(const MsgPtr &msg) {

        if (dynamic_pointer_cast<MsgSensorData>(msg)) {
            auto msgSensor = dynamic_pointer_cast<MsgSensorData>(msg);
            auto sensorData = msgSensor->getData();

            if (sensorData && dynamic_pointer_cast<ImageTs>(sensorData)) {

                auto pImage = dynamic_pointer_cast<ImageTs>(sensorData);

                if (!pImage || pImage->mImage.empty()) {
                    return;
                }

                cv::imshow("Image", pImage->mImage);
                int dly = 33;
                if (msgSensor->getTargetId() == FCN_SEN_GET_NEXT)
                    dly = 0;
                int keyVal = cv::waitKey(dly);
                if (keyVal == 'q') {
                    auto msgStop = make_shared<Message>(Sensor::TOPIC, "stop_play", FCN_SEN_STOP_PLAY);
                    mpChannel->publish(msgStop);
                }
            }
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