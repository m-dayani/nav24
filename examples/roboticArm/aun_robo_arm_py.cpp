//
// Created by masoud on 5/15/24.
//

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include <iostream>

#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <Python.h>

#include "ParameterBlueprint.h"
#include "System.hpp"
#include "FE_CalibCamCv.hpp"
#include "FE_ObjTracking.hpp"
#include "Camera.hpp"

using namespace std;
using namespace NAV24;

class ParamReceiver : public MsgCallback {
public:
    void receive(const MsgPtr &msg) override {
        if (msg && dynamic_pointer_cast<MsgConfig>(msg)) {
            auto pMsgConfig = dynamic_pointer_cast<MsgConfig>(msg);
            mpParam = pMsgConfig->getConfig();
        }
    }

protected:
    void setup(const MsgPtr &configMsg) override {}
    void handleRequest(const MsgPtr &reqMsg) override {}
    void run() override {}

public:
    ParamPtr mpParam;
};

void exec_tracking(const shared_ptr<System>& mpSystem, const string& defVideo = "") {

    // If camera is calibrated, run the object tracking front-end
    auto pFeObjTracking = make_shared<FE::ObjTracking>(mpSystem);
    mpSystem->registerChannel(ID_CH_FE, pFeObjTracking);
    mpSystem->registerSubscriber(ID_TP_SDATA, pFeObjTracking);
    mpSystem->registerSubscriber(ID_TP_FE, pFeObjTracking);
    mpSystem->registerPublisher(ID_TP_OUTPUT, pFeObjTracking);
    // Change dataset sequence to obj_tr_cap
    MsgPtr msgChSeq = make_shared<Message>(ID_CH_DS, DataStore::TOPIC, FCN_DS_REQ_CH_NS, "obj_tr_cap");
    mpSystem->send(msgChSeq);
    // Initialize Frontend
    //ParamPtr pYoloPy = make_shared<ParamType<string>>(KEY_FE_TYPE, nullptr, FE_TR_TYPE_YOLO_PY);
    MsgPtr pMsgConfigFeOT = make_shared<MsgConfig>(ID_CH_FE, nullptr, FE::ObjTracking::TOPIC);
    pFeObjTracking->receive(pMsgConfigFeOT);
    // Set online camera
    auto msgConfOnline = make_shared<Message>(ID_CH_SENSORS, Sensor::TOPIC,
                                              FCN_SEN_CONFIG, TAG_SEN_MX_STREAM);
    mpSystem->send(msgConfOnline);
    // Load default video
    if (!defVideo.empty()) {
        msgConfOnline->setTargetId(FCN_CAM_LOAD_VIDEO);
        msgConfOnline->setMessage(defVideo);
        mpSystem->send(msgConfOnline);
    }
    // Run online camera
    auto msgStartPlay = make_shared<Message>(ID_CH_SENSORS, Sensor::TOPIC,
                                             FCN_SEN_START_PLAY, "start_play");
    mpSystem->send(msgStartPlay);
}

int main([[maybe_unused]] int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    Py_SetProgramName(Py_DecodeLocale(argv[0], nullptr)); // optional but strongly recommended
    Py_Initialize();

    if (!Py_IsInitialized()) {
        LOG(ERROR) << "Python API is not initialized, abort\n";
        return 2;
    }

    string confFile = "../../config/AUN_ARM1.yaml";
    string defVideo = "robo-arm-cap.avi";
    shared_ptr<ParamReceiver> pParamRec = make_shared<ParamReceiver>();

    // Create the system
    shared_ptr<System> mpSystem = make_shared<System>();
    mpSystem->registerChannel(ID_CH_SYS, mpSystem);

    // Load settings
    MsgPtr msgLoadSettings = make_shared<Message>(ID_CH_SYS, System::TOPIC, FCN_LD_PARAMS, confFile);
    mpSystem->receive(msgLoadSettings);

    // Check camera calibration
    bool isCamCalibrated = false;
    // todo: avoid hard-wired strings
    MsgReqPtr msgGetCamParams = make_shared<MsgRequest>(ID_CH_PARAMS, pParamRec, ParameterServer::TOPIC,
                                                        FCN_PS_REQ, string(PARAM_CAM) + "/0/calib");
    mpSystem->send(msgGetCamParams);
    if (pParamRec->mpParam && pParamRec->mpParam->getAllChildren().count("intrinsics") > 0) {
        isCamCalibrated = true;
    }

    if (isCamCalibrated) {
        exec_tracking(mpSystem, defVideo);
    }
    else {
        LOG(ERROR) << argv[0] << ", camera is not calibrated\n";
        return 1;
    }

    Py_Finalize();
    return 0;
}
