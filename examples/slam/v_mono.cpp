//
// Created by masoud on 8/29/24.
//

#include <iostream>

#include <glog/logging.h>
#include <opencv2/core.hpp>

#include "System.hpp"
#include "FE_CalibCamCv.hpp"
#include "FE_SlamMonoV.hpp"

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

int main([[maybe_unused]] int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    string confFile = "../../config/EuRoC.yaml";
    shared_ptr<ParamReceiver> pParamRec = make_shared<ParamReceiver>();

    // Create the system
    shared_ptr<System> mpSystem = make_shared<System>();
    mpSystem->registerChannel(ID_CH_SYS, mpSystem);

    // Load settings
    MsgPtr msgLoadSettings = make_shared<Message>(ID_CH_SYS, System::TOPIC, FCN_LD_PARAMS, confFile);
    mpSystem->receive(msgLoadSettings);

    auto pFeSlamMonoV = make_shared<FE::SlamMonoV>(mpSystem);
    mpSystem->registerChannel(ID_CH_FE, pFeSlamMonoV);
    mpSystem->registerSubscriber(ID_TP_SDATA, pFeSlamMonoV);
    mpSystem->registerSubscriber(ID_TP_FE, pFeSlamMonoV);
    mpSystem->registerPublisher(ID_TP_OUTPUT, pFeSlamMonoV);

    // Initialize Frontend
    MsgPtr pMsgConfigFeOT = make_shared<MsgConfig>(ID_CH_FE, nullptr, FE::FrontEnd::TOPIC);
    pFeSlamMonoV->receive(pMsgConfigFeOT);

    auto msgStartPlay = make_shared<Message>(ID_CH_SENSORS, Sensor::TOPIC,
                                             FCN_SEN_START_PLAY, "start_play");
    mpSystem->send(msgStartPlay);

    return 0;
}