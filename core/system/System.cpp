//
// Created by root on 12/21/23.
//

#include "System.hpp"
#include "ParameterBlueprint.h"
#include "Camera.hpp"
#include "DataConversion.hpp"
#include "FrontEnd.hpp"
#include "ImageViewer.hpp"

using namespace std;

namespace NAV24 {

    System::System() : mmChannels{}, mpParamServer(), mmpDataStores(), mmpTrans(),
                       mpTempParam(nullptr), mpAtlas(nullptr), mpTrajManager(nullptr), mpThreads() {}

    /* -------------------------------------------------------------------------------------------------------------- */

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

    /* -------------------------------------------------------------------------------------------------------------- */

    void System::loadSettings(const std::string &settings) {

        // Load params
        this->loadParameters(settings);

        // Load datasets
        this->loadDatasets();

        // Load inputs (sensors)
        this->loadSensors();

        // Load Relations
        this->loadRelations();

        // Load outputs
        this->loadOutputs();

        // Initialize Components
        this->initComponents();
    }

    void System::loadParameters(const std::string &settings) {

        MsgPtr msgLoadSettings = make_shared<Message>(ParameterServer::TOPIC, settings, FCN_PS_LOAD);
        mpParamServer = make_shared<ParameterServer>(shared_from_this());
        mpParamServer->receive(msgLoadSettings);
        this->registerChannel(mpParamServer, ParameterServer::TOPIC);
    }

    void System::loadDatasets() {

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
                        this->registerChannel(pDataProvider, DataStore::TOPIC);
                    }
                }
            }
        }
    }

    void System::loadSensors() {

        auto msgGetParams = make_shared<MsgRequest>(ParameterServer::TOPIC,
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
    }

    void System::loadRelations() {

        auto msgGetParams = make_shared<MsgRequest>(ParameterServer::TOPIC,
                                               PARAM_REL, FCN_PS_REQ, shared_from_this());
        mpParamServer->receive(msgGetParams);
        if (mpTempParam && mpTempParam->getName() == "Relations") {
            for (const auto& relParamPair : mpTempParam->getAllChildren()) {
                auto pRelParam = relParamPair.second.lock();
                if (pRelParam) {
                    auto pRef = find_param<ParamType<string>>("ref", pRelParam);
                    string ref = (pRef) ? pRef->getValue() : "";
                    auto pTar = find_param<ParamType<string>>("target", pRelParam);
                    string target = (pTar) ? pTar->getValue() : "";
                    auto p_t_rt = find_param<ParamType<double>>("t_rt", pRelParam);
                    double t_rt = (p_t_rt) ? p_t_rt->getValue() : 0.0;
                    auto p_T_rt = find_param<ParamType<cv::Mat>>("T_rt", pRelParam);
                    cv::Mat T_rt = (p_T_rt) ? p_T_rt->getValue() : cv::Mat();

                    if (!ref.empty()) {
                        // TODO: work more on pose class
                        PosePtr pT_rt = make_shared<Pose>();
                        vector<float> q = Converter::toQuaternion(T_rt.rowRange(0, 3).colRange(0, 3));
                        for (char i = 0; i < 4; i++) pT_rt->q[i] = q[i];
                        pT_rt->p[0] = T_rt.at<float>(0, 3);
                        pT_rt->p[1] = T_rt.at<float>(1, 3);
                        pT_rt->p[2] = T_rt.at<float>(2, 3);

                        TransPtr pTrans = make_shared<Transformation>(ref, target, pT_rt, t_rt);

                        string transKey;
                        transKey.append(ref).append(":").append(target);
                        mmpTrans.insert(make_pair(transKey, pTrans));
                    }
                }
            }
        }
    }

    void System::loadOutputs() {

        auto msgGetParams = make_shared<MsgRequest>(ParameterServer::TOPIC,
                                               PARAM_OUT, FCN_PS_REQ, shared_from_this());
        mpParamServer->receive(msgGetParams);
        if (mpTempParam && mpTempParam->getName() == PARAM_OUT) {
            for (const auto& outParamPair : mpTempParam->getAllChildren()) {
                auto pParam = outParamPair.second.lock();
                if (pParam) {
                    OutputPtr pOutput = Output::getNewInstance(pParam, shared_from_this());
                    if (pOutput) {
                        // load output params
                        msgGetParams = make_shared<MsgRequest>(ParameterServer::TOPIC,
                                                               string(PARAM_OUT) + "/" + outParamPair.first,
                                                               FCN_PS_REQ, pOutput);
                        mpParamServer->receive(msgGetParams);
                        mmpOutputs.insert(make_pair(pOutput->getName(), pOutput));
                        this->registerChannel(pOutput, Output::TOPIC);
                        auto msgRunOutput = make_shared<MsgRequest>(Output::TOPIC, "", FCN_SYS_RUN, shared_from_this());
                        pOutput->receive(msgRunOutput);
                        //mpThreads.push_back(thread(&Output::run, pOutput));
                    }
                }
            }
        }
    }

    void System::initComponents() {

        // Initialize Atlas (Map/World Manager)
        if (!mpAtlas) {
            mpAtlas = make_shared<Atlas>(shared_from_this());
            this->registerChannel(mpAtlas, Atlas::TOPIC);
        }
        // Initialize Trajectory Manager
        if (!mpTrajManager) {
            mpTrajManager = make_shared<TrajManager>();
            this->registerChannel(mpTrajManager, TrajManager::TOPIC);
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */

    void System::receive(const MsgPtr &msg) {

        if (msg) {
            if (msg->getTopic() == System::TOPIC) {
                switch (msg->getTargetId()) {
                    case FCN_LD_PARAMS:
                        this->loadSettings(msg->getMessage());
                        break;
                    case FCN_SYS_UPDATE:
                        break;
                    case FCN_GET_TRANS:
                        this->handleRequest(msg);
                        break;
                    default:
                        DLOG(WARNING) << "System::receive: Action is not supported\n";
                        break;
                }
                if (dynamic_pointer_cast<MsgType<shared_ptr<thread>>>(msg)) {
                    auto msgThread = dynamic_pointer_cast<MsgType<shared_ptr<thread>>>(msg);
                    auto pThread = msgThread->getData();
                    if (pThread) {
                        mpThreads.push_back(pThread);
                    }
                }
                if (msg->getTargetId() == FCN_SYS_STOP) {
                    this->stop();
                }
            }
            this->handleConfigMsg(msg);
        }
    }

    void System::handleConfigMsg(const MsgPtr &msg) {

        if (dynamic_pointer_cast<MsgConfig>(msg)) {
            auto pMsgConf = dynamic_pointer_cast<MsgConfig>(msg);
            mpTempParam = pMsgConf->getConfig();
        }
    }

    void System::handleRequest(const MsgPtr& msg) {

        if (!msg || !dynamic_pointer_cast<MsgRequest>(msg)) {
            DLOG(WARNING) << "System::handleRequests, bad message\n";
            return;
        }

        auto msgReq = dynamic_pointer_cast<MsgRequest>(msg);
        auto sender = msgReq->getCallback();

        if (!sender) {
            DLOG(WARNING) << "System::handleRequests, bad sender\n";
            return;
        }

        int action = msg->getTargetId();
        string msgStr = msg->getMessage();

        if (action == FCN_GET_TRANS) {
            if (mmpTrans.count(msgStr) > 0) {

                auto pTrans = mmpTrans[msgStr];
                auto msgTrans = make_shared<MsgType<TransPtr>>(msg->getTopic(), pTrans);
                sender->receive(msgTrans);
            }
        }
    }

    void System::setup(const MsgPtr &configMsg) {

    }

    void System::run() {

    }

    void System::stop() {
        MsgCallback::stop();

        auto msgStop = make_shared<Message>(TOPIC, "", FCN_SYS_STOP);

        for (const auto& pChPair : mmChannels) {
            auto pChannels = pChPair.second;
            for (const auto& pCh : pChannels) {
                if (pCh) {
                    pCh->receive(msgStop);
                }
            }
        }
    }


}