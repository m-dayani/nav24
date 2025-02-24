//
// Created by masoud on 2/19/25.
// Inference-based Nav: Inference & Perception is first and
//                      Localization and Mapping comes next
//



#include <iostream>

#include <glog/logging.h>

#include "System.hpp"
#include "FE_CalibCamCv.hpp"
#include "FE_InferenceNav.hpp"
#include "Camera.hpp"

using namespace std;
using namespace NAV24;


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

    // R the Inference Nav front-end
    auto pFeObjTracking = make_shared<FE::InferenceNav>(mpSystem);
    mpSystem->registerChannel(ID_CH_FE, pFeObjTracking);
    mpSystem->registerSubscriber(ID_TP_SDATA, pFeObjTracking);
    mpSystem->registerSubscriber(ID_TP_FE, pFeObjTracking);
    mpSystem->registerPublisher(ID_TP_OUTPUT, pFeObjTracking);

    // Initialize Frontend
    //ParamPtr pYoloOnnx = make_shared<ParamType<string>>(KEY_FE_TYPE, nullptr, FE_TR_TYPE_YOLO_ONNX);
    //ParamPtr pCvOnly = make_shared<ParamType<string>>(KEY_FE_TYPE, nullptr, FE_TR_TYPE_CV_ONLY);
    MsgPtr pMsgConfigFeOT = make_shared<MsgConfig>(ID_CH_FE, nullptr, FE::InferenceNav::TOPIC);
    pFeObjTracking->receive(pMsgConfigFeOT);

    // Run online camera
    auto msgStartPlay = make_shared<Message>(ID_CH_SENSORS, Sensor::TOPIC,
                                             FCN_SEN_START_PLAY, "start_play");
    // Run camera in detached mode (so main thread is controlled by ROS)
    auto msgRunCamera = make_shared<MsgRequest>(ID_CH_SENSORS,
                                                mpSystem, Sensor::TOPIC, FCN_SYS_RUN);
    mpSystem->send(msgStartPlay);

    return 0;
}

