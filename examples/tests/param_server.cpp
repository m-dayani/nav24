
#include <iostream>
#include <memory>
#include <thread>

#include <glog/logging.h>

#include "ParameterServer.hpp"
#include "ParameterBlueprint.h"

using namespace std;
using namespace NAV24;


ParamPtr globalParam = nullptr;


class DummySystem : public MsgCallback {
public:
    void receive(const MsgPtr &msg) override {

        if (!msg) {
            DLOG(WARNING) << "Received null message\n";
            return;
        }
        if (dynamic_pointer_cast<MsgConfig>(msg)) {
            auto pMsgConfig = dynamic_pointer_cast<MsgConfig>(msg);
            globalParam = pMsgConfig->getConfig();
        }
        cout << msg->toString() << endl;
    }
};


int main([[maybe_unused]] int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    string confFile = "../../config/AUN_ARM1.yaml";
    string confFile1 = "../../config/EuRoC.yaml";
    string confNoFile = "wrong/file.yaml";
    string saveFile = "../../config/DUMMY.yaml";

    MsgCbPtr pSystem = make_shared<DummySystem>();

    // Testing parameter server normal operation

    // Load settings
    MsgPtr msgLoadSettings = make_shared<Message>(ParameterServer::TOPIC, confFile1, FCN_PS_LOAD);
    shared_ptr<ParameterServer> pParamServer = make_shared<ParameterServer>(nullptr);
    pParamServer->receive(msgLoadSettings);

    // Load different settings
    MsgPtr msgLoadDiffSettings = make_shared<Message>(ParameterServer::TOPIC, confFile, FCN_PS_LOAD);
    pParamServer->receive(msgLoadDiffSettings);

    // Print Full Status:
    MsgPtr msgGetFullStat = make_shared<MsgRequest>(ParameterServer::TOPIC, TAG_PS_GET_STAT, FCN_PS_REQ, pSystem);
    pParamServer->receive(msgGetFullStat);

    // Get required dataset parameters
    MsgPtr msgGetDsParams = make_shared<MsgRequest>(ParameterServer::TOPIC, PARAM_DS, FCN_PS_REQ, pSystem);
    pParamServer->receive(msgGetDsParams);

    // If you change parameters, ParamServer's parameters will change automatically
    if (globalParam) {
        globalParam->setName(globalParam->getName() + "_ps_test");
    }

    // Save Settings
    cout << "Saving parameters to: " << saveFile << endl;
    MsgPtr msgSaveSettings = make_shared<Message>(ParameterServer::TOPIC, saveFile, FCN_PS_SAVE);
    pParamServer->receive(msgSaveSettings);

    // Save to the same load file
    MsgPtr msgSaveToLoadFile = make_shared<Message>(ParameterServer::TOPIC, TAG_PS_USE_LOAD_PATH, FCN_PS_SAVE);
    pParamServer->receive(msgSaveToLoadFile);

    // Testing incorrect use

    MsgPtr msgNull = nullptr;
    MsgPtr msgEmpty = make_shared<Message>();
    MsgPtr msgWrongTopic = make_shared<Message>("WRONG/TOPIC");
    MsgPtr msgWrongAction = make_shared<Message>(ParameterServer::TOPIC, "", 2938);
    MsgPtr msgWrongPath = make_shared<Message>(ParameterServer::TOPIC, confNoFile, FCN_PS_LOAD);
    MsgPtr msgWrongMsg = make_shared<MsgRequest>(ParameterServer::TOPIC, "WRONG/MESSAGE", FCN_PS_REQ, pSystem);
    MsgPtr msgReqNullSender = make_shared<MsgRequest>(ParameterServer::TOPIC, PARAM_DS, FCN_PS_REQ, nullptr);
    MsgPtr msgSaveToNoFile = make_shared<Message>(ParameterServer::TOPIC, confNoFile, FCN_PS_SAVE);
    MsgPtr msgParams = make_shared<MsgConfig>(ParameterServer::TOPIC, globalParam);

    pParamServer->receive(msgNull);
    pParamServer->receive(msgEmpty);
    pParamServer->receive(msgWrongTopic);
    pParamServer->receive(msgWrongAction);
    pParamServer->receive(msgWrongPath);
    pParamServer->receive(msgWrongMsg);
    pParamServer->receive(msgReqNullSender);
    pParamServer->receive(msgSaveToNoFile);
    pParamServer->receive(msgParams);

    return 0;
}
