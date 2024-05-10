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
                if (pDataStore && msg->getTopic() == ParameterServer::TOPIC) {
                    MsgPtr msgParams = make_shared<MsgConfig>(DataStore::TOPIC, pParam);
                    msgParams->setTargetId(FCN_DS_LOAD);
                    pDataStore->receive(msgParams);
                }
                cout << pParam->printStr("") << endl;
            }
        }
        else if (msg) {
            cout << msg->toString() << endl;
        }
    }

protected:
    void setup(const MsgPtr &configMsg) override {

    }

    void handleRequest(const MsgPtr &reqMsg) override {

    }

    void run() override {

    }

public:

    shared_ptr<DataStore> pDataStore = nullptr;
};

class DummyChannel : public Channel {
public:
    void send(const MsgPtr &message) override {

        if (message) {
            cout << "DummyChannel::send, message to send: " << message->toString() << endl;
        }
    }
    void registerChannel(const MsgCbPtr &callback, const string &topic) override {}
    void unregisterChannel(const MsgCbPtr &callback, const string &topic) override {}
};


int main([[maybe_unused]] int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    string configFile = "../../config/BluePrint.yaml";
    string configFile1 = "../../config/AUN_ARM1.yaml";
    string configFile2 = "../../config/EuRoC.yaml";

    // System & loader
    MsgCbPtr pSystem = make_shared<DummySystem>();
    ChannelPtr pChannel = make_shared<DummyChannel>();
    auto pParamServer = make_shared<ParameterServer>(nullptr);

    // Load params messages
    MsgPtr msgLdConfNormal = make_shared<Message>(ParameterServer::TOPIC, configFile, FCN_PS_LOAD);
    MsgPtr msgLdConfTricky = make_shared<Message>(ParameterServer::TOPIC, configFile1, FCN_PS_LOAD);
    MsgPtr msgLdConfTricky1 = make_shared<Message>(ParameterServer::TOPIC, configFile2, FCN_PS_LOAD);

    // Normal operation

    // Data provider
    shared_ptr<DataStore> pDataProvider = make_shared<DataStore>(pChannel);
    //pChannel->registerChannel(pDataProvider, DataStore::TOPIC);

    // Load tricky params
    pParamServer->receive(msgLdConfTricky1);
    // Setup data store with these params
    MsgPtr msgGetDsParams = make_shared<MsgRequest>(ParameterServer::TOPIC, "DS", FCN_PS_REQ, pDataProvider);
    pParamServer->receive(msgGetDsParams);

    // Change the dataset
    pParamServer->receive(msgLdConfTricky);
    msgGetDsParams = make_shared<MsgRequest>(ParameterServer::TOPIC, PARAM_DS0, FCN_PS_REQ, pDataProvider);
    pParamServer->receive(msgGetDsParams);

    // Change the dataset once again
    pParamServer->receive(msgLdConfNormal);
    msgGetDsParams = make_shared<MsgRequest>(ParameterServer::TOPIC, PARAM_DS0, FCN_PS_REQ, pSystem);
    pParamServer->receive(msgGetDsParams);

    // Print loader state
    MsgPtr msgPrintLdState = make_shared<MsgRequest>(DataStore::TOPIC, TAG_DS_GET_STAT, FCN_DS_REQ, pSystem);
    pDataProvider->receive(msgPrintLdState);

    // Change sequence
    MsgPtr msgChSeq = make_shared<Message>(DataStore::TOPIC, TAG_DS_CH_SEQ_INC, FCN_DS_REQ_CHANGE);
    pDataProvider->receive(msgChSeq);

    // Get images path params
    MsgPtr msgImagePaths = make_shared<MsgRequest>(DataStore::TOPIC, TAG_DS_GET_PATH_IMG, FCN_DS_REQ, pSystem);
    pDataProvider->receive(msgImagePaths);

    // Reset and inquire IMU path
    msgChSeq->setMessage(TAG_DS_CH_SEQ_RST);
    pDataProvider->receive(msgChSeq);
    MsgPtr msgImuPath = make_shared<MsgRequest>(DataStore::TOPIC, TAG_DS_GET_PATH_IMU, FCN_DS_REQ, pSystem);
    pDataProvider->receive(msgImuPath);

    // Bad operation

    vector<MsgPtr> badMessages{};

    badMessages.push_back(nullptr);
    badMessages.push_back(make_shared<Message>());
    badMessages.push_back(make_shared<Message>("WRONG/TOPIC"));
    badMessages.push_back(make_shared<Message>(ParameterServer::TOPIC, "", 2938));
    badMessages.push_back(make_shared<Message>(DataStore::TOPIC));
    badMessages.push_back(make_shared<Message>(DataStore::TOPIC, "", 36527988736));
    badMessages.push_back(make_shared<Message>(DataStore::TOPIC, "", FCN_DS_LOAD));
    MsgConfigPtr paramBadConf = make_shared<MsgConfig>(DataStore::TOPIC, nullptr);
    paramBadConf->setTargetId(FCN_DS_LOAD);
    badMessages.push_back(paramBadConf);
    badMessages.push_back(make_shared<MsgRequest>(DataStore::TOPIC, TAG_DS_GET_STAT, FCN_DS_REQ, nullptr));
    badMessages.push_back(make_shared<MsgRequest>(DataStore::TOPIC, "Wrong/Request/Message", FCN_DS_REQ, nullptr));
    badMessages.push_back(make_shared<MsgRequest>(DataStore::TOPIC, TAG_DS_GET_STAT, 9827, pSystem));
    badMessages.push_back(make_shared<Message>(DataStore::TOPIC, "Wrong/Change/Seq/Message", FCN_DS_REQ_CHANGE));
    badMessages.push_back(make_shared<Message>(DataStore::TOPIC, TAG_DS_CH_SEQ_DEC, 76352));
    badMessages.push_back(make_shared<Message>(DataStore::TOPIC, TAG_DS_CH_SEQ_DEC, FCN_DS_REQ_CHANGE));
    badMessages.push_back(make_shared<Message>(DataStore::TOPIC, TAG_DS_CH_SEQ_DEC, FCN_DS_REQ_CHANGE));

    int i = 0;
    for (const auto& msg : badMessages) {
//        cout << "processing message #" << i << endl;
        pDataProvider->receive(msg);
        i++;
    }

    auto msgWrongParams = make_shared<MsgRequest>(ParameterServer::TOPIC, "Input/Camera", FCN_PS_REQ, pSystem);
    pParamServer->receive(msgWrongParams);

    // Print the latest state after these changes
    cout << endl << endl;
    pDataProvider->receive(msgPrintLdState);

    return 0;
}
