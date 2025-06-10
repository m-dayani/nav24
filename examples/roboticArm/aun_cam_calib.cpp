//
// Created by masoud on 4/26/24.
//

#include <iostream>

#include <glog/logging.h>

#include "System.hpp"
#include "FE_CalibCamCv.hpp"
#include "FE_ObjTracking.hpp"
#include "Camera.hpp"

using namespace std;
using namespace NAV24;


class ParamReceiverCalib : public ParamReceiver {
public:
    void receive(const MsgPtr &msg) override {
        ParamReceiver::receive(msg);
        if (msg && dynamic_pointer_cast<MsgType<CalibPtrRO>>(msg)) {
            mpCalib = dynamic_pointer_cast<MsgType<CalibPtrRO>>(msg)->getData();
        }
    }

    CalibPtrRO mpCalib;
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
        auto fp = [pFeCamCalib](auto && PH1) { pFeCamCalib->receive(std::forward<decltype(PH1)>(PH1)); };
        auto msgGetNext = make_shared<MsgRequest>(ID_CH_SENSORS, fp,
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

    // Show the last frame
    auto msgShowLastFrame = make_shared<Message>(ID_CH_FE, FE::FrontEnd::TOPIC, FCN_SHOW_LAST_FRAME);
    pFeCamCalib->receive(msgShowLastFrame);
}

int main([[maybe_unused]] int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    if (argc < 3) {
        cerr << "Usage: " << argv[0] << " config_blueprint.yaml config_result.yaml\n";
        return 1;
    }

    string confFile = argv[1];
    string saveFile = argv[2];
    shared_ptr<ParamReceiverCalib> pParamRec = make_shared<ParamReceiverCalib>();

    // Create the system
    shared_ptr<System> mpSystem = make_shared<System>();
    mpSystem->registerChannel(ID_CH_SYS, mpSystem);

    // Load settings
    MsgPtr msgLoadSettings = make_shared<Message>(ID_CH_SYS, System::TOPIC, FCN_LD_PARAMS, confFile);
    mpSystem->receive(msgLoadSettings);

    // Check camera calibration
    auto fp = [pParamRec](auto && PH1) { pParamRec->receive(std::forward<decltype(PH1)>(PH1)); };
    auto msgReqCalib = make_shared<MsgRequest>(ID_CH_SENSORS, fp,
                                               Sensor::TOPIC, FCN_CAM_GET_CALIB);
    mpSystem->send(msgReqCalib);

    bool isCamCalibrated = false;
    if (pParamRec->mpCalib && pParamRec->mpCalib->isCalibrated()) {
        isCamCalibrated = true;
    }

    if (!isCamCalibrated) {
        exec_calib(mpSystem, saveFile);
    }
    else {
        LOG(INFO) << argv[0] << ", camera is calibrated and ready\n";
    }

    return 0;
}


