//
// Created by masoud on 6/17/24.
//

#include <thread>
#include <chrono>
#include <utility>
#include <glog/logging.h>

#include "MapViewer.hpp"
#include "System.hpp"
#include "pose/PoseProvider.hpp"

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
    void setup(const MsgPtr &configMsg) override {

    }

    void handleRequest(const MsgPtr &reqMsg) override {

    }

    void run() override {

    }

    ChannelPtr mpViewer;
    unsigned long cnt = 0;
};

int main([[maybe_unused]] int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    string confFile = "../../config/AUN_ARM1.yaml";
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

    auto pPoseRec = make_shared<PoseReceiver>(mpSystem);

    auto msgGetNextPose = make_shared<MsgRequest>(ID_CH_SENSORS, pPoseRec,
                                                  Sensor::TOPIC, FCN_SEN_GET_NEXT, "");

    // Give viewer some time to draw objects
    for (int i = 0; i < 11000; i++) {
        mpSystem->send(msgGetNextPose);
    }
    int c = getchar();

    auto msgStop = make_shared<Message>(ID_CH_SYS, System::TOPIC, FCN_SYS_STOP);
    mpSystem->receive(msgStop);

    return 0;
}

