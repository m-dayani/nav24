
#include <iostream>
#include <memory>
#include <thread>

#include "ParameterServer.hpp"
#include "EurocLoader.hpp"
#include "Visualization.hpp"
#include "TabularTextWriter.hpp"

using namespace std;
using namespace NAV24;


class DummySystem : public Channel, public MsgCallback {
public:

    DummySystem(const string& settings) {


    }

    void publish(const MsgPtr &message) override {

        for (auto channel : mmChannels[message->getTopic()]) {
            channel->receive(message);
        }
    }

    void registerChannel(const MsgCbPtr& callback, const string &topic) override {

        if (mmChannels.count(topic) <= 0) {
            mmChannels[topic] = vector<MsgCbPtr>();
        }
        mmChannels[topic].push_back(callback);
    }

    void unregisterChannel(const MsgCbPtr& callback, const string &topic) override {

    }

    void receive(const MsgPtr &msg) override {

        cout << msg->getMessage() << endl;
    }

    map<string, vector<MsgCbPtr>> mmChannels;
};


int main(int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    if (argc < 2) {
        cout << "Usage: " << argv[0] << " settings.yaml\n";
        return 1;
    }

    string settingsFile = argv[1];
    cout << "Settings File: " << settingsFile << endl;

    shared_ptr<DummySystem> pSystem = make_shared<DummySystem>(settingsFile);
    ChannelPtr chSystem = dynamic_pointer_cast<Channel>(pSystem);
    MsgCbPtr recvSystem = dynamic_pointer_cast<MsgCallback>(pSystem);

    // Testing parameter server
    // Load Settings
    MsgPtr msgLoadSettings = make_shared<Message>(ParameterServer::TOPIC, settingsFile, FCN_PS_LOAD);
    shared_ptr<ParameterServer> pParamServer = make_shared<ParameterServer>(chSystem, msgLoadSettings);
    chSystem->registerChannel(pParamServer, ParameterServer::TOPIC);

    // Print Full Status:
    MsgPtr msgGetFullStat = make_shared<MsgRequest>(ParameterServer::TOPIC, TAG_PS_GET_STAT, FCN_PS_REQ, recvSystem);
    chSystem->publish(msgGetFullStat);

    // Save Settings
    string paramsFile = "../../config/DUMMY.yaml";
    cout << "Saving parameters to: " << paramsFile << endl;
    MsgPtr msgSaveSettings = make_shared<Message>(ParameterServer::TOPIC, paramsFile, FCN_PS_SAVE);
    chSystem->publish(msgSaveSettings);

    // Get required camera parameters
    MsgPtr msgGetCamParams = make_shared<MsgRequest>(ParameterServer::TOPIC, "Camera", FCN_PS_REQ, recvSystem);
    chSystem->publish(msgGetCamParams);

    return 0;
}
