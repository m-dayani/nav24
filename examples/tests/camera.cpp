//
// Created by root on 2/1/24.
//

#include <iostream>

#include <glog/logging.h>
#include <opencv2/core.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/core/utility.hpp>
//#include <opencv2/highgui.hpp>

#include "ParameterServer.hpp"
#include "ParameterBlueprint.h"
#include "Calibration.hpp"
#include "Camera.hpp"
#include "DataStore.hpp"
#include "Image.hpp"


using namespace std;
using namespace NAV24;

/*class Camera {
public:
    explicit Camera(std::string img_path) : imgPath(std::move(img_path)) {}

    void play() {
        vector<cv::String> fn;
        cv::glob(imgPath, fn, false);

        for (const auto& imgFile : fn) {
            cv::Mat img = cv::imread(imgFile);
            cv::imshow("Image", img);
            cv::waitKey(33);
            //std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
        cv::destroyAllWindows();
    }

private:
    std::string imgPath;
};*/


class DummySystem : public MsgCallback, public Channel {
public:
    void handleImageMsg(const MsgPtr &msg) {

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
                    for (const auto& pCam : mvpCamera) {
                        if (pCam)
                            pCam->receive(msgStop);
                    }
                }
            }
        }
    }

    void handleConfigMsg(const MsgPtr &msg) {

        if (dynamic_pointer_cast<MsgConfig>(msg)) {
            auto pMsgConf = dynamic_pointer_cast<MsgConfig>(msg);
            mpParamCamera = pMsgConf->getConfig();
        }
    }

    void receive(const MsgPtr &msg) override {

        if (msg) {
            this->handleConfigMsg(msg);
            this->handleImageMsg(msg);
        }
    }

    void publish(const MsgPtr &msg) override {

        if (!msg) {
            DLOG(WARNING) << "DummySystem::publish, Null message detected\n";
            return;
        }
        this->handleImageMsg(msg);
    }

    void registerChannel(const MsgCbPtr &callback, const string &topic) override {

    }

    void unregisterChannel(const MsgCbPtr &callback, const string &topic) override {

    }

    ParamPtr mpParamCamera;
    vector<shared_ptr<Camera>> mvpCamera{};
};


int main([[maybe_unused]] int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    string confFile = "../../config/AUN_ARM.yaml";
    string confFile1 = "../../config/TUM_RGBD.yaml";

    shared_ptr<DummySystem> pSystem = make_shared<DummySystem>();

    // Load sample settings
    MsgPtr msgLoadSettings = make_shared<Message>(ParameterServer::TOPIC, confFile, FCN_PS_LOAD);
    shared_ptr<ParameterServer> pParamServer = make_shared<ParameterServer>(nullptr);
    pParamServer->receive(msgLoadSettings);

    // DataStore
    shared_ptr<DataStore> pDataProvider = make_shared<DataStore>(nullptr);
    auto msgGetDsParams = make_shared<MsgRequest>(ParameterServer::TOPIC, PARAM_DS0, FCN_PS_REQ, pDataProvider);
    pParamServer->receive(msgGetDsParams);

    // Normal operation

    // Calibration
    MsgReqPtr msgGetCamParams = make_shared<MsgRequest>(ParameterServer::TOPIC, PARAM_CAM, FCN_PS_REQ, pSystem);
    pParamServer->receive(msgGetCamParams);

    if (pSystem->mpParamCamera) {
        ParamPtr pParamCalib = pSystem->mpParamCamera->read("calib");
        if (pParamCalib) {
            CalibPtr pCalib = make_shared<Calibration>(pParamCalib);
            cout << pCalib->printStr() << "\n";
        }
    }

    // Camera
    auto pCamOffline = make_shared<CamOffline>(pSystem);
    auto pCamStream = make_shared<CamStream>(pSystem);
    // TODO: also test mixed camera
    pSystem->mvpCamera.push_back(pCamOffline);
    pSystem->mvpCamera.push_back(pCamStream);

    msgGetCamParams = make_shared<MsgRequest>(ParameterServer::TOPIC, PARAM_CAM, FCN_PS_REQ, pCamOffline);
    auto msgGetCamParams1 = make_shared<MsgRequest>(ParameterServer::TOPIC, PARAM_CAM, FCN_PS_REQ, pCamStream);
    MsgPtr msgImagePaths = make_shared<MsgRequest>(DataStore::TOPIC, TAG_DS_GET_PATH_IMG, FCN_DS_REQ, pCamOffline);
    pDataProvider->receive(msgImagePaths);
    pParamServer->receive(msgGetCamParams);
    pParamServer->receive(msgGetCamParams1);

    auto msgPrint = make_shared<Message>(Sensor::TOPIC, "print", FCN_SEN_PRINT);
    auto msgStartPlay = make_shared<Message>(Sensor::TOPIC, "start_play", FCN_SEN_START_PLAY);
    auto msgGetNext = make_shared<MsgRequest>(Sensor::TOPIC, "get_next", FCN_SEN_GET_NEXT, pSystem);
    auto msgReset = make_shared<Message>(Sensor::TOPIC, "reset", FCN_SEN_RESET);

    pCamOffline->receive(msgPrint);
    pCamOffline->receive(msgStartPlay);
    pCamOffline->receive(msgReset);
    pCamOffline->receive(msgGetNext);
    pCamStream->receive(msgPrint);
    pCamStream->receive(msgStartPlay);
    pCamStream->receive(msgReset);
    pCamStream->receive(msgGetNext);

    // Wrong operation

    // Calibration
    CalibPtr pCalibNull = make_shared<Calibration>(nullptr);
    cout << pCalibNull->printStr() << endl;
    if (pSystem->mpParamCamera) {
        // Wrong params
        CalibPtr pCalibWrongParams = make_shared<Calibration>(pSystem->mpParamCamera->read("interface"));
        cout << pCalibWrongParams->printStr() << endl;
    }

    return 0;
}