//
// Created by root on 1/26/24.
//

#include <iostream>

#include <glog/logging.h>

#include "ParameterServer.hpp"
#include "ParameterBlueprint.h"
#include "DataStore.hpp"


using namespace std;
using namespace NAV24;


class DummySystem : public MsgCallback {
public:
    void receive(const MsgPtr &msg) override {

        MsgConfigPtr pParamMsg = dynamic_pointer_cast<MsgConfig>(msg);
        if (pParamMsg) {
            auto pParam = pParamMsg->getConfig();
            if (pParam) {
                cout << pParam->printStr() << endl;
            }
        }
        else {
            cout << msg->getMessage() << endl;
        }
    }
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

    auto pSystem = make_shared<DummySystem>(settingsFile);
    ChannelPtr chSystem = static_pointer_cast<Channel>(pSystem);
    MsgCbPtr sysCallback = static_pointer_cast<MsgCallback>(pSystem);

    // Load Settings
    MsgPtr msgLoadSettings = make_shared<Message>(ParameterServer::TOPIC, settingsFile, FCN_PS_LOAD);
    shared_ptr<ParameterServer> pParamServer = make_shared<ParameterServer>(chSystem, msgLoadSettings);
    chSystem->registerChannel(pParamServer, ParameterServer::TOPIC);

    // Data provider
    shared_ptr<DataStore> pDataProvider = make_shared<DataStore>(chSystem);
    chSystem->registerChannel(pDataProvider, DataStore::TOPIC);
    MsgPtr msgGetDsParams = make_shared<MsgRequest>(ParameterServer::TOPIC, PARAM_DS, FCN_PS_REQ, pDataProvider);
    chSystem->publish(msgGetDsParams);

    //cout << pDataProvider->printLoaderStateStr() << endl;
    // Print loader state
    MsgPtr msgPrintLdState = make_shared<MsgRequest>(DataStore::TOPIC, TAG_DS_GET_STAT, FCN_DS_REQ, sysCallback);
    chSystem->publish(msgPrintLdState);

    // Get images path params
    MsgPtr msgImagePaths = make_shared<MsgRequest>(DataStore::TOPIC, TAG_DS_GET_PATH_IMG, FCN_DS_REQ, sysCallback);
    chSystem->publish(msgImagePaths);

    return 0;
}
