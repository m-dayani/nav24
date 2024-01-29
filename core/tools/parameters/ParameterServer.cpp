//
// Created by root on 5/10/21.
//

#include "ParameterServer.hpp"
#include "YamlParserCV.hpp"

using namespace std;


namespace NAV24 {

    ParameterServer::ParameterServer(const ChannelPtr &server) :
        mpChannel(server), mConfigFile(""), mpParamRoot(nullptr), mvpAllParams{} {}

    ParameterServer::ParameterServer(const ChannelPtr& server, const MsgPtr& configMsg) :
        ParameterServer(server) {

        this->receive(configMsg);
    }

    void ParameterServer::load(const string &settingsFile) {
        mpParamRoot = YamlParserCV::loadParams(settingsFile, mvpAllParams);
    }

    void ParameterServer::save(const string &pathParams) {
        YamlParserCV::saveParams(pathParams, mpParamRoot);
    }

    void ParameterServer::receive(const MsgPtr &msg) {

        // check the topic
        string topic = msg->getTopic();
        if (topic != ParameterServer::TOPIC) {
            // todo: log warning
            return;
        }

        // check function
        int action = msg->getTargetId();
        switch (action) {
            case FCN_PS_LOAD:
                this->load(msg->getMessage());
                break;
            case FCN_PS_SAVE:
                this->save(msg->getMessage());
                break;
            case FCN_PS_REQ:
                this->handleRequest(msg);
                break;
            default:
                //todo: log warning
                break;
        }
    }

    void ParameterServer::handleRequest(const MsgPtr &msg) {

        MsgReqPtr request = static_pointer_cast<MsgRequest>(msg);
        if (!request) {
            // todo: log warning
            return;
        }

        MsgCbPtr sender = request->getCallback();

        // check message
        string tag = request->getMessage();

        if (tag == TAG_PS_GET_STAT) {
            sender->receive(make_shared<Message>(msg->getTopic(), mpParamRoot->printStr()));
            return;
        }

        // find which parameter it requests
        auto pParam = mpParamRoot->read(tag);

        // create a response message
        MsgPtr response = make_shared<MsgConfig>(msg->getTopic(), pParam);

        // send back to the caller
        sender->receive(response);
    }

} // NAV24
