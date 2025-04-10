//
// Created by masoud on 4/7/25.
// Working with Atlas and Mapping Utilities


#include <thread>
#include <glog/logging.h>

#include "System.hpp"
#include "FE_MappingMonoV.hpp"
#include "ParameterBlueprint.h"


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

    // Create the system
    shared_ptr<System> mpSystem = make_shared<System>(confFile);

    // Create a Mapping FrontEnd
    auto pMappingFE = make_shared<FE::MappingMonoV>(mpSystem);
    mpSystem->registerSubscriber(ID_TP_SDATA, pMappingFE);
    mpSystem->registerChannel(ID_CH_FE, pMappingFE);

    // when all the components are registered, send a setup command to config every thing
    auto fp = [pMappingFE](auto && PH1) { pMappingFE->receive(std::forward<decltype(PH1)>(PH1)); };
    MsgPtr msgOpParams = make_shared<MsgRequest>(ID_CH_PARAMS, fp, ParameterServer::TOPIC,
                                                 FCN_PS_REQ, string(PARAM_OP));
    mpSystem->receive(msgOpParams);

    // play the pose provider (and all other sensors) in the bg
    auto fp1 = [mpSystem](auto && PH1) { mpSystem->receive(std::forward<decltype(PH1)>(PH1)); };
    auto msgRun = make_shared<MsgRequest>(ID_CH_SENSORS, fp1, Sensor::TOPIC, FCN_SYS_RUN);
    mpSystem->send(msgRun);

    cout << "Press a key to finish execution:\n";
    getchar();

    auto msgStop = make_shared<Message>(ID_CH_SYS, System::TOPIC, FCN_SYS_STOP);
    mpSystem->receive(msgStop);

    return 0;
}

