//
// Created by root on 2/1/24.
//

#include <iostream>
#include <utility>
//#include <thread>
//#include <chrono>

#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>

#include "ParameterServer.hpp"
#include "ParameterBlueprint.h"
#include "Calibration.hpp"
#include "Camera.hpp"


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


class DummySystem : public MsgCallback {
public:
    void receive(const MsgPtr &msg) override {

        if (msg) {
            if (dynamic_pointer_cast<MsgConfig>(msg)) {
                auto pMsgConf = dynamic_pointer_cast<MsgConfig>(msg);
                mpParamCamera = pMsgConf->getConfig();
            }
        }
    }

    ParamPtr mpParamCamera;
};


int main([[maybe_unused]] int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    string confFile = "../../config/AUN_ARM.yaml";

    shared_ptr<DummySystem> pSystem = make_shared<DummySystem>();

    // Load sample settings
    MsgPtr msgLoadSettings = make_shared<Message>(ParameterServer::TOPIC, confFile, FCN_PS_LOAD);
    shared_ptr<ParameterServer> pParamServer = make_shared<ParameterServer>(nullptr);
    pParamServer->receive(msgLoadSettings);

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
    shared_ptr<Camera> pCamera = make_shared<Camera>(nullptr);
    msgGetCamParams = make_shared<MsgRequest>(ParameterServer::TOPIC, PARAM_CAM, FCN_PS_REQ, pCamera);
    pParamServer->receive(msgGetCamParams);
    cout << pCamera->toString() << endl;

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