//
// Created by root on 5/10/21.
//

#include "ParameterServer.hpp"
#include "YamlParserCV.hpp"

#include <glog/logging.h>


using namespace std;


namespace NAV24 {

    ParameterServer::ParameterServer(const ChannelPtr &server) :
        mpChannel(server), mConfigFile(""), mpParamRoot(nullptr), mvpAllParams{} {}

    ParameterServer::ParameterServer(const ChannelPtr& server, const MsgPtr& configMsg) :
        ParameterServer(server) {

        this->receive(configMsg);
    }

    void ParameterServer::load(const string &settingsFile) {

        if (mpParamRoot || mvpAllParams.size() > 0) {
            mvpAllParams = vector<ParamPtr>();
            mpParamRoot = nullptr;
            DLOG(INFO) << "ParameterServer::load, refreshing parameters\n";
        }

        mpParamRoot = YamlParserCV::loadParams(settingsFile, mvpAllParams);

        if (mvpAllParams.size() > 0) {
            mConfigFile = settingsFile;
        }
    }

    void ParameterServer::save(const string &pathParams) {

        if (pathParams == TAG_PS_USE_LOAD_PATH) {
            DLOG(INFO) << "ParameterServer::save, saving to " << mConfigFile << "\n";
            YamlParserCV::saveParams(mConfigFile, mpParamRoot);
        }
        else {
            DLOG(INFO) << "ParameterServer::save, saving to " << pathParams << "\n";
            YamlParserCV::saveParams(pathParams, mpParamRoot);
        }
    }

    void ParameterServer::receive(const MsgPtr &msg) {

        if (!msg) {
            DLOG(WARNING) << "ParameterServer::receive, null message input\n";
            return;
        }

        // check the topic
        string topic = msg->getTopic();
        if (topic != ParameterServer::TOPIC) {
            DLOG(WARNING) << "ParameterServer::receive, unsupported topic: " << topic << "\n";
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
                DLOG(WARNING) << "ParameterServer::receive, unsupported action: " << action << "\n";
                break;
        }
    }

    void ParameterServer::handleRequest(const MsgPtr &msg) {

        MsgReqPtr request = static_pointer_cast<MsgRequest>(msg);
        if (!request) {
            DLOG(WARNING) << "ParameterServer::handleRequest, Null request detected\n";
            return;
        }

        MsgCbPtr sender = request->getCallback();
        if (!sender) {
            DLOG(WARNING) << "ParameterServer::handleRequest, Null sender detected\n";
            return;
        }

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
