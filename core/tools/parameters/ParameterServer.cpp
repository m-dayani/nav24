//
// Created by root on 5/10/21.
//

#include "ParameterServer.hpp"

using namespace std;


namespace NAV24 {

    ParameterServer::ParameterServer(const ChannelPtr &server) :
        mpChannel(server), mConfigFile(""), mpFileStorage(nullptr), mpParam(nullptr) {}

    ParameterServer::ParameterServer(const ChannelPtr& server, const MsgPtr& configMsg) :
        ParameterServer(server) {

        this->receive(configMsg);
    }

    ParameterServer::~ParameterServer() {

        if (mpFileStorage) {
            mpFileStorage->release();
        }
    }

    void ParameterServer::load(const string &settingsFile) {

        mConfigFile = settingsFile;
        mpFileStorage = make_shared<cv::FileStorage>(mConfigFile, cv::FileStorage::READ);
        if(!mpFileStorage || !mpFileStorage->isOpened()) {
            cerr << "** ERROR: Failed to open settings file: " << mConfigFile << endl;
            return;
        }
    }

    void ParameterServer::save(const string &pathParams) {

        shared_ptr<cv::FileStorage> pFileStorage = make_shared<cv::FileStorage>(mConfigFile, cv::FileStorage::WRITE);
        if (!pathParams.empty()) {
            pFileStorage = make_shared<cv::FileStorage>(pathParams, cv::FileStorage::WRITE);
        }

        if(!pFileStorage || !pFileStorage->isOpened()) {
            cerr << "** ERROR: Failed to open settings file: " << pathParams << endl;
            return;
        }

        // todo: how to save or print fileStorage?

        pFileStorage->release();
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

        // check message to find which parameter type it requests
        string tag = request->getMessage();
        ParamPtr pParam = this->getParameter(tag);

        if (msg->getTargetId() == FCN_PS_PRINT) {
            sender->receive(make_shared<Message>(msg->getTopic(), pParam->printStr()));
            return;
        }

        // create a response message
        MsgPtr response = make_shared<MsgConfig>(msg->getTopic(), pParam);

        // send back to the caller
        sender->receive(response);
    }

    const ParamPtr &ParameterServer::getParameter(const std::string& tag) {

        // todo
        //return <#initializer#>;
        return nullptr;
    }

} // NAV24
