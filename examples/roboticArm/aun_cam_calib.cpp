//
// Created by masoud on 4/26/24.
//

#include <iostream>

#include <glog/logging.h>
#include <opencv2/core.hpp>

#include "ParameterBlueprint.h"
#include "System.hpp"
#include "FE_CalibCamCv.hpp"

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

    ParamPtr mpParam;
};


int main([[maybe_unused]] int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    string confFile = "../../config/AUN_ARM.yaml";
    shared_ptr<ParamReceiver> pParamRec = make_shared<ParamReceiver>();

    // Create the system
    shared_ptr<System> mpSystem = make_shared<System>();

    // Load settings
    MsgPtr msgLoadSettings = make_shared<Message>(System::TOPIC, confFile, FCN_LD_PARAMS);
    mpSystem->receive(msgLoadSettings);

    // Check camera calibration
    bool isCamCalibrated = false;
    MsgReqPtr msgGetCamParams = make_shared<MsgRequest>(ParameterServer::TOPIC,
                                                        string(PARAM_CAM) + "/0/calib",
                                                        FCN_PS_REQ, pParamRec);
    mpSystem->publish(msgGetCamParams);
    if (pParamRec->mpParam && pParamRec->mpParam->getAllChildren().count("intrinsics") > 0) {
        isCamCalibrated = true;
    }
    if (!isCamCalibrated) {
        // If camera is not calibrated, run the calib front-end
        bool needCurrPattern = true;
        shared_ptr<FE::CalibCamCv> pFeCamCalib = make_shared<FE::CalibCamCv>(mpSystem);
        // Register front-end to system
        mpSystem->registerChannel(pFeCamCalib, FE::FrontEnd::TOPIC);
        // Configure front-end
        vector<ParamPtr> vpParamContainer{};
        auto pCalibCamCvParams = FE::CalibCamCv::getDefaultParameters(vpParamContainer);
        if (pCalibCamCvParams) {
            shared_ptr<ParamType<double>> pParamScale = find_param<ParamType<double>>("grid_scale", pCalibCamCvParams);
            if (pParamScale) {
                pParamScale->setValue(2.5);
            }
        }
        MsgPtr pMsgConfigFeCalib = make_shared<MsgConfig>(FE::CalibCamCv::TOPIC, pCalibCamCvParams);
        pFeCamCalib->receive(pMsgConfigFeCalib);
        // Set offline camera
        auto msgConfOffline = make_shared<Message>(Sensor::TOPIC, TAG_SEN_MX_OFFLINE,
                                                 FCN_SEN_CONFIG);
        mpSystem->publish(msgConfOffline);
        // Run offline camera
        auto msgStartPlay = make_shared<Message>(Sensor::TOPIC, "start_play", FCN_SEN_START_PLAY);
        mpSystem->publish(msgStartPlay);
        if (needCurrPattern) {
            // Set online camera
            msgConfOffline->setMessage(TAG_SEN_MX_STREAM);
            mpSystem->publish(msgConfOffline);
            // Get the most current pattern
            auto msgGetNext = make_shared<MsgRequest>(Sensor::TOPIC, "get_next",
                                                      FCN_SEN_GET_NEXT, pFeCamCalib);
            mpSystem->publish(msgGetNext);
        }
        // TODO: Save the results
    }
    else {
        // If camera is calibrated, run the object tracking front-end
    }

    return 0;
}


