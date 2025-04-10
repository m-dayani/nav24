//
// Created by masoud on 4/4/25.
//

#include <thread>
#include <glog/logging.h>

#include "MapViewer.hpp"
#include "System.hpp"
#include "PoseProvider.hpp"

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
    //string defVideo = "robo-arm-cap.avi";
    //shared_ptr<ParamReceiver> pParamRec = make_shared<ParamReceiver>();

    // Create the system
    shared_ptr<System> mpSystem = make_shared<System>();
    //mpSystem->registerChannel(ID_CH_SYS, mpSystem);

    // Load settings
    MsgPtr msgLoadSettings = make_shared<Message>(ID_CH_SYS, System::TOPIC, FCN_LD_PARAMS, confFile);
    mpSystem->receive(msgLoadSettings);

    // Create a new trajectory
    MsgPtr msgCreateTraj = make_shared<Message>(ID_CH_TRAJECTORY, TrajManager::TOPIC, FCN_TRJ_CREATE, "world0");
    mpSystem->send(msgCreateTraj);

    // play the pose provider (and all other sensors)
    auto fp = [mpSystem](auto && PH1) { mpSystem->receive(std::forward<decltype(PH1)>(PH1)); };
    auto msgRun = make_shared<MsgRequest>(ID_CH_SENSORS, fp, Sensor::TOPIC, FCN_SYS_RUN);
    mpSystem->send(msgRun);

    cout << "Press a key to finish execution:\n";
    getchar();

    auto msgStop = make_shared<Message>(ID_CH_SYS, System::TOPIC, FCN_SYS_STOP);
    mpSystem->receive(msgStop);

    return 0;
}

