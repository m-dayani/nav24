//
// Created by masoud on 2/19/25.
// Retrieve GT poses and draw them (using Pangolin)
//

#include <thread>
#include <glog/logging.h>

#include "MapViewer.hpp"
#include "System.hpp"
#include "PoseProvider.hpp"

using namespace std;
using namespace NAV24;


class PoseReceiver : public MsgCallback {
public:
    explicit PoseReceiver(ChannelPtr pViewer) : MsgCallback(), mpViewer(std::move(pViewer)) {}

    void receive(const MsgPtr &msg) override {

        if (msg) {
            if (dynamic_pointer_cast<MsgType<PosePtr>>(msg)) {
                if (cnt % 100 == 0) {
                    msg->setChId(ID_TP_OUTPUT);
                    mpViewer->publish(msg);
                }
                cnt++;
            }
        }
    }

protected:
    void setup(const MsgPtr &) override {}
    void handleRequest(const MsgPtr &) override {}
    void run() override {}

    ChannelPtr mpViewer;
    unsigned long cnt = 0;
};


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

    // Instantiate PoseProvider
    auto pPoseProvider = make_shared<PoseProvider>(mpSystem);
    mpSystem->registerPublisher(ID_TP_OUTPUT, pPoseProvider);
    mpSystem->registerChannel(ID_CH_SENSORS, pPoseProvider);
    // setup
    auto msgPoseConfig = make_shared<MsgRequest>(ID_CH_PARAMS, pPoseProvider,
                                                 ParameterServer::TOPIC, FCN_PS_REQ, "Input/Pose/0");
    mpSystem->send(msgPoseConfig);

    MsgPtr msgConfPaths = make_shared<MsgRequest>(ID_CH_DS, pPoseProvider, DataStore::TOPIC,
                                                  FCN_DS_REQ, TAG_DS_GET_PATH_GT);
    mpSystem->send(msgConfPaths);

    // Map Viewer
    //auto pMapViewer = make_shared<MapViewer>(mpSystem);
    auto pPoseRec = make_shared<PoseReceiver>(mpSystem);

    auto msgGetNextPose = make_shared<MsgRequest>(ID_CH_SENSORS, pPoseRec,
                                                  Sensor::TOPIC, FCN_SEN_GET_NEXT, "");

    // Give viewer some time to draw objects
    for (int i = 0; i < 1000; i++) {
        mpSystem->send(msgGetNextPose);
    }
    getchar();

    auto msgStop = make_shared<Message>(ID_CH_SYS, System::TOPIC, FCN_SYS_STOP);
    mpSystem->receive(msgStop);

    return 0;
}


