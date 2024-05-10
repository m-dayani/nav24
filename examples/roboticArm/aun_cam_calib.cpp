//
// Created by masoud on 4/26/24.
//

#include <iostream>

#include <glog/logging.h>
#include <opencv2/core.hpp>

#include "ParameterBlueprint.h"
#include "System.hpp"
#include "FE_CalibCamCv.hpp"
#include "FE_ObjTracking.hpp"

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

void exec_calib(const shared_ptr<System>& mpSystem, const string& saveFile, bool needCurrPattern = true) {

    const string seq_calib = "calib";

    // If camera is not calibrated, run the calib front-end
    shared_ptr<FE::CalibCamCv> pFeCamCalib = make_shared<FE::CalibCamCv>(mpSystem);
    // Register front-end to system
    mpSystem->registerChannel(ID_CH_FE, pFeCamCalib);
    mpSystem->registerSubscriber(ID_TP_SDATA, pFeCamCalib);
    mpSystem->registerPublisher(ID_TP_OUTPUT, pFeCamCalib);
    // Change dataset sequence to calib
    MsgPtr msgChSeq = make_shared<Message>(ID_CH_DS, DataStore::TOPIC, FCN_DS_REQ_CH_NS, seq_calib);
    mpSystem->send(msgChSeq);
    // Configure front-end
    vector<ParamPtr> vpParamContainer{};
    auto pCalibCamCvParams = FE::CalibCamCv::getDefaultParameters(vpParamContainer);
    if (pCalibCamCvParams) {
        shared_ptr<ParamType<double>> pParamScale = find_param<ParamType<double>>("grid_scale", pCalibCamCvParams);
        if (pParamScale) {
            pParamScale->setValue(2.5);
        }
    }
    MsgPtr pMsgConfigFeCalib = make_shared<MsgConfig>(ID_CH_FE, pCalibCamCvParams, FE::CalibCamCv::TOPIC);
    pFeCamCalib->receive(pMsgConfigFeCalib);
    // Set offline camera
    auto msgConfOffline = make_shared<Message>(ID_CH_SENSORS, Sensor::TOPIC,
                                               FCN_SEN_CONFIG, TAG_SEN_MX_OFFLINE);
    mpSystem->send(msgConfOffline);
    // Run offline camera
    auto msgStartPlay = make_shared<Message>(ID_CH_SENSORS, Sensor::TOPIC,
                                             FCN_SEN_START_PLAY, "start_play");
    mpSystem->send(msgStartPlay);
    if (needCurrPattern) {
        // Set online camera
        msgConfOffline->setMessage(TAG_SEN_MX_STREAM);
        mpSystem->send(msgConfOffline);
        // Get the most current pattern
        auto msgGetNext = make_shared<MsgRequest>(ID_CH_SENSORS, pFeCamCalib,
                                                  Sensor::TOPIC,FCN_SEN_GET_NEXT, "get_next");
        mpSystem->send(msgGetNext);
    }
    // Calibrate
    auto msgCalib = make_shared<Message>(ID_CH_FE, FE::CalibCamCv::TOPIC, FCN_FE_CAM_CALIB);
    pFeCamCalib->receive(msgCalib);
    // Save the results
    cout << "Saving parameters to: " << saveFile << endl;
    MsgPtr msgSaveSettings = make_shared<Message>(ID_CH_PARAMS, ParameterServer::TOPIC, FCN_PS_SAVE, saveFile);
    mpSystem->send(msgSaveSettings);
}

void exec_tracking(const shared_ptr<System>& mpSystem) {

    // If camera is calibrated, run the object tracking front-end
    auto pFeObjTracking = make_shared<FE::ObjTracking>(mpSystem);
    mpSystem->registerChannel(ID_CH_FE, pFeObjTracking);
    mpSystem->registerSubscriber(ID_TP_SDATA, pFeObjTracking);
    mpSystem->registerSubscriber(ID_TP_FE, pFeObjTracking);
    mpSystem->registerPublisher(ID_TP_OUTPUT, pFeObjTracking);
    // Change dataset sequence to calib
    MsgPtr msgChSeq = make_shared<Message>(ID_CH_DS, DataStore::TOPIC, FCN_DS_REQ_CH_NS, "obj_tr_cap");
    mpSystem->send(msgChSeq);
    // Initialize Frontend
    MsgPtr pMsgConfigFeOT = make_shared<MsgConfig>(ID_CH_FE, nullptr, FE::ObjTracking::TOPIC);
    pFeObjTracking->receive(pMsgConfigFeOT);
    // Set online camera
    auto msgConfOnline = make_shared<Message>(ID_CH_SENSORS, Sensor::TOPIC,
                                              FCN_SEN_CONFIG, TAG_SEN_MX_STREAM);
    mpSystem->send(msgConfOnline);
    // Run online camera
    auto msgStartPlay = make_shared<Message>(ID_CH_SENSORS, Sensor::TOPIC,
                                             FCN_SEN_START_PLAY, "start_play");
    mpSystem->send(msgStartPlay);
}

int main([[maybe_unused]] int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    string confFile = "../../config/AUN_ARM1.yaml";
    string saveFile = "../../config/AUN_ARM1.yaml";
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

    if (!isCamCalibrated) {
        exec_calib(mpSystem, saveFile);
    }
    else {
        exec_tracking(mpSystem);
    }

    return 0;
}


