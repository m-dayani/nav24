//
// Created by masoud on 4/26/24.
//

#include <iostream>

#include <glog/logging.h>

#include "ParameterBlueprint.h"
#include "System.hpp"
#include "FE_CalibCamCv.hpp"
#include "FE_ObjTracking.hpp"
#include "Camera.hpp"

using namespace std;
using namespace NAV24;


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
    //ParamPtr pYoloOnnx = make_shared<ParamType<string>>(KEY_FE_TYPE, nullptr, FE_TR_TYPE_YOLO_ONNX);
    //ParamPtr pCvOnly = make_shared<ParamType<string>>(KEY_FE_TYPE, nullptr, FE_TR_TYPE_CV_ONLY);
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
    // Run camera in detached mode (so main thread is controlled by ROS)
    auto msgRunCamera = make_shared<MsgRequest>(ID_CH_SENSORS,
                                                mpSystem, Sensor::TOPIC, FCN_SYS_RUN);
    mpSystem->send(msgStartPlay);
}

int main(int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " config_file.yaml\n";
        return 1;
    }

    string confFile = argv[1];
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

    return 0;
}



