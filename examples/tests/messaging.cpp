//
// Created by root on 1/30/24.
//

#include <iostream>
#include <memory>
#include <thread>

#include <glog/logging.h>

#include "Parameter.hpp"
#include "Message.hpp"
#include "Interface.hpp"

using namespace std;
using namespace NAV24;


class DummySystem : public Channel, public MsgCallback {
public:

    DummySystem() : mmChannels() {}

    void publish(const MsgPtr &message) override {

        if (!message) {
            cerr << "WARNING: DummySystem::publish, detected null message\n";
            return;
        }

        string topic = message->getTopic();

        if (mmChannels.contains(topic)) {
            for (const auto& channel : mmChannels[topic]) {
                channel->receive(message);
            }
        }
        else {
            cout << "WARNING: DummySystem::publish, unknown topic: " << topic << endl;
        }
    }

    void registerChannel(const MsgCbPtr& callback, const string &topic) override {

        if (mmChannels.count(topic) <= 0) {
            mmChannels[topic] = vector<MsgCbPtr>();
        }
        mmChannels[topic].push_back(callback);
    }

    void unregisterChannel(const MsgCbPtr& callback, const string &topic) override {

        if (mmChannels.contains(topic)) {
            vector<MsgCbPtr> oldCallbacks = mmChannels[topic];
            mmChannels[topic] = vector<MsgCbPtr>();
            for (const auto& cb : oldCallbacks) {
                if (callback != cb) {
                    mmChannels[topic].push_back(cb);
                }
            }
            if (mmChannels[topic].empty()) {
                mmChannels.erase(topic);
            }
        }
    }

    void receive(const MsgPtr &msg) override {

        if (!msg) {
            cerr << "WARNING: DummySystem::receive, detected null message\n";
            return;
        }

        cout << msg->getMessage() << endl;
    }

protected:
    void setup(const MsgPtr &configMsg) override {

    }

    void handleRequest(const MsgPtr &reqMsg) override {

    }

    void run() override {

    }

public:

    map<string, vector<MsgCbPtr>> mmChannels;
};

class DummyNode : public MsgCallback {
public:

    void receive(const MsgPtr &msg) override {

        if (!msg) {
            cerr << "WARNING: DummyNode, detected null message\n";
            return;
        }

        MsgReqPtr pMsgReq = dynamic_pointer_cast<MsgRequest>(msg);
        if (pMsgReq) {
            auto sender = pMsgReq->getCallback();
            if (!sender) {
                cout << "DummyNode, received request message with empty sender\n";
                return;
            }
            MsgPtr response = make_shared<Message>(pMsgReq->getTopic(), "Response Message from DummyNode");
            sender->receive(response);
            return;
        }

        MsgConfigPtr pMsgConfig = dynamic_pointer_cast<MsgConfig>(msg);
        if (pMsgConfig) {
            auto param = pMsgConfig->getConfig();
            if (param) {
                cout << "DummyNode, received parameter: " << param->printStr("") << endl;
            }
            else {
                cout << "DummyNode, received null parameter!\n";
            }
            return;
        }

        cout << msg->toString() << endl;
    }

protected:
    void setup(const MsgPtr &configMsg) override {

    }

    void handleRequest(const MsgPtr &reqMsg) override {

    }

    void run() override {

    }
};


int main([[maybe_unused]] int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    // Channel System
    auto pSystem = make_shared<DummySystem>();
    ChannelPtr chSystem = dynamic_pointer_cast<Channel>(pSystem);
    MsgCbPtr recvSystem = dynamic_pointer_cast<MsgCallback>(pSystem);

    // Node
    MsgCbPtr dummyNode = make_shared<DummyNode>();

    // A null message
    MsgPtr msgNull = nullptr;
    // An empty message
    MsgPtr msgEmpty{};
    // Empty topic
    MsgPtr msgEmptyTopic = make_shared<Message>("");
    // Topic message
    MsgPtr msgTopic = make_shared<Message>("SampleTopic", "Topic Message", 10);
    // Wrong Target Type
    MsgPtr msgTargetType = make_shared<Message>("OtherTopic", "Wrong Target Type Message", 2.34);
    // Request message
    MsgPtr msgRequestNull = make_shared<MsgRequest>("NullRequestTopic", nullptr);
    // Correct request message
    MsgReqPtr msgRequest = make_shared<MsgRequest>("RequestTopic", recvSystem);
    // Null config message
    MsgPtr msgConfigNull = make_shared<MsgConfig>("NullConfigTopic", nullptr);
    // Correct config message
    shared_ptr<ParamType<string>> param = make_shared<ParamType<string>>("StringParam", nullptr, "SampleString");
    shared_ptr<ParamType<int>> param1 = make_shared<ParamType<int>>("IntParam", nullptr, 37);
    MsgConfigPtr msgConfig = make_shared<MsgConfig>("ConfigTopic", param);

    // Setters and Getters
    msgEmptyTopic->setTargetId(34);
    cout << "Wrong target type message target ID: " << msgTargetType->getTargetId() << endl;
    cout << "Topic message, message before change: " << msgTopic->getMessage() << endl;
    msgTopic->setMessage("New Topic Message");
    cout << "Topic message, message after change: " << msgTopic->toString() << endl;
    cout << "Other topic message, topic before change: " << msgTargetType->getTopic() << endl;
    msgTargetType->setTopic("NewOtherTopic");
    cout << "Other topic message, topic after change: " << msgTargetType->getTopic() << endl;
    cout << "Request message, sender before change: " << msgRequest->getCallback() << endl;
    msgRequest->setCallback(dummyNode);
    cout << "Request message, sender after change: " << msgRequest->getCallback() << endl;
    cout << "Config message, param before change: " << msgConfig->getConfig()->printStr("") << endl;
    param->setValue("NewString");
    cout << "Config message, param after change: " << msgConfig->getConfig()->printStr("") << endl << endl;
    msgConfig->setConfig(param1);

    // Channels and nodes
    chSystem->registerChannel(dummyNode, "SampleTopic");
    chSystem->registerChannel(dummyNode, "NullRequestTopic");
    chSystem->registerChannel(dummyNode, "RequestTopic");
    chSystem->registerChannel(dummyNode, "NullConfigTopic");
    chSystem->registerChannel(dummyNode, "ConfigTopic");
    chSystem->registerChannel(dummyNode, "NewOtherTopic");

    chSystem->unregisterChannel(dummyNode, "OtherTopic");
    chSystem->unregisterChannel(dummyNode, "NewOtherTopic");

    chSystem->publish(msgNull);
    chSystem->publish(msgEmpty);
    chSystem->publish(msgEmptyTopic);
    chSystem->publish(msgTopic);
    chSystem->publish(msgTargetType);
    chSystem->publish(msgRequestNull);
    chSystem->publish(msgRequest);
    chSystem->publish(msgConfigNull);
    chSystem->publish(msgConfig);

    return 0;
}