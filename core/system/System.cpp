//
// Created by root on 12/21/23.
//

#include "System.hpp"

#include "ParameterBlueprint.h"
#include "Camera.hpp"

using namespace std;

namespace NAV24 {

    System::System() : mmChannels{}, mpParamServer(), mmpDataStores(),
        mpTempParam(nullptr), mpAtlas(nullptr), mpTrajManager(nullptr) {}

    void System::loadSettings(const std::string &settings) {

        // Load params
        MsgPtr msgLoadSettings = make_shared<Message>(ParameterServer::TOPIC, settings, FCN_PS_LOAD);
        mpParamServer = make_shared<ParameterServer>(shared_from_this());
        mpParamServer->receive(msgLoadSettings);
        this->registerChannel(mpParamServer, ParameterServer::TOPIC);

        // Load datasets
        auto msgGetParams = make_shared<MsgRequest>(ParameterServer::TOPIC,
                                                    PARAM_DS, FCN_PS_REQ, shared_from_this());
        mpParamServer->receive(msgGetParams);
        if (mpTempParam && mpTempParam->getName() == "DS") {

            size_t nDs = mpTempParam->getAllChildKeys().size();
            for (size_t i = 0; i < nDs; i++) {
                shared_ptr<DataStore> pDataProvider = make_shared<DataStore>(shared_from_this());
                string msgTarget = string(PARAM_DS) + "/" + to_string(i);
                msgGetParams = make_shared<MsgRequest>(ParameterServer::TOPIC, msgTarget,
                                                       FCN_PS_REQ, pDataProvider);
                mpParamServer->receive(msgGetParams);
                msgGetParams->setCallback(shared_from_this());
                mpParamServer->receive(msgGetParams);
                if (mpTempParam) {
                    auto dsParam = find_param<ParamType<string>>("name", mpTempParam);
                    if (dsParam) {
                        mmpDataStores.insert(make_pair(dsParam->getValue(), pDataProvider));
                    }
                }
            }
        }

        // Load inputs (sensors)
        msgGetParams = make_shared<MsgRequest>(ParameterServer::TOPIC,
                                               PARAM_CAM, FCN_PS_REQ, shared_from_this());
        mpParamServer->receive(msgGetParams);
        if (mpTempParam && mpTempParam->getName() == "Camera") {
            map<string, ParamPtrW> mParams = mpTempParam->getAllChildren();
            for (const auto& camParam : mParams) {
                auto pCamParam = camParam.second.lock();
                if (pCamParam) {
                    auto camName = find_param<ParamType<string>>("name", pCamParam);
                    string camNameStr = (camName) ? camName->getValue() : "cam0";
                    auto ifType = find_param<ParamType<string>>("interface/type", pCamParam);
                    if (ifType) {
                        string interfaceType = ifType->getValue();

                        MsgReqPtr msgGetCamParams{};
                        shared_ptr<Camera> pCamera{};
                        if (interfaceType == "offline") {
                            pCamera = make_shared<CamOffline>(shared_from_this());
                        }
                        else if (interfaceType == "stream") {
                            pCamera = make_shared<CamStream>(shared_from_this());
                        }
                        else if (interfaceType == "mixed") {
                            pCamera = make_shared<CamMixed>(shared_from_this());
                        }
                        if (pCamera && (interfaceType == "offline" || interfaceType == "mixed")) {
                            auto ifTarget = find_param<ParamType<string>>("interface/target", pCamParam);
                            string ifTargetStr = (ifTarget) ? ifTarget->getValue() : "";
                            MsgPtr msgImagePaths = make_shared<MsgRequest>(DataStore::TOPIC, TAG_DS_GET_PATH_IMG,
                                                                           FCN_DS_REQ, pCamera);
                            if (mmpDataStores.count(ifTargetStr) > 0) {
                                mmpDataStores[ifTargetStr]->receive(msgImagePaths);
                            }
                        }
                        if (pCamera) {
                            msgGetCamParams = make_shared<MsgRequest>(ParameterServer::TOPIC,
                                                                      string(PARAM_CAM) + "/" + camParam.first,
                                                                      FCN_PS_REQ, pCamera);
                            mpParamServer->receive(msgGetCamParams);
                            mmpSensors.insert(make_pair(camNameStr, pCamera));
                        }
                    }
                }
            }
        }

        for (const auto& pSensor : mmpSensors) {
            this->registerChannel(pSensor.second, Sensor::TOPIC);
        }

        // Load Relations

        // Load outputs

        // Initialize Components
        this->initComponents();
    }

    void System::publish(const MsgPtr &message) {

        for (const auto& channel : mmChannels[message->getTopic()]) {
            channel->receive(message);
        }
    }

    void System::registerChannel(const MsgCbPtr &callback, const string &topic) {

        if (mmChannels.count(topic) <= 0) {
            mmChannels[topic] = vector<MsgCbPtr>();
        }
        mmChannels[topic].push_back(callback);
    }

    void System::unregisterChannel(const MsgCbPtr &callback, const string &topic) {

    }

    void System::handleConfigMsg(const MsgPtr &msg) {

        if (dynamic_pointer_cast<MsgConfig>(msg)) {
            auto pMsgConf = dynamic_pointer_cast<MsgConfig>(msg);
            mpTempParam = pMsgConf->getConfig();
        }
    }

    void System::receive(const MsgPtr &msg) {

        if (msg) {
            if (msg->getTopic() == System::TOPIC) {
                switch (msg->getTargetId()) {
                    case FCN_LD_PARAMS:
                        this->loadSettings(msg->getMessage());
                        break;
                    default:
                        DLOG(WARNING) << "System::receive: Action is not supported\n";
                        break;
                }
            }
            this->handleConfigMsg(msg);
            //this->handleImageMsg(msg);
        }
    }

    void System::initComponents() {

        // Initialize Atlas (Map/World Manager)
        if (!mpAtlas) {
            mpAtlas = make_shared<Atlas>(shared_from_this());
        }
        // Initialize Trajectory Manager
        if (!mpTrajManager) {
            mpTrajManager = make_shared<TrajManager>();
        }
    }


}