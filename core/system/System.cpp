//
// Created by root on 12/21/23.
//

#include "System.hpp"
#include "ParameterBlueprint.h"
#include "Camera.hpp"
#include "FrontEnd.hpp"
#include "ImageViewer.hpp"

using namespace std;

namespace NAV24 {

    System::System() : mmChannels(), mmPublishers(), mmSubscribers(), mpParamServer(), mmpDataStores(), mmpSensors(),
                       mmpTrans(), mmpOutputs(), mpTempParam(nullptr), mpAtlas(nullptr), mpTrajManager(nullptr),
                       mpThreads() {
        mName = "System";
    }

    /* -------------------------------------------------------------------------------------------------------------- */

    void System::send(const MsgPtr &message) {

        int catId = message->getChId();
        if (mmChannels.count(catId) > 0) {
            for (const auto &channel: mmChannels[catId]) {
                channel->receive(message);
            }
        }
    }

    void System::publish(const MsgPtr &message) {

        int catId = message->getChId();
        if (mmSubscribers.count(catId) > 0) {
            for (const auto &channel: mmSubscribers[catId]) {
                channel->receive(message);
            }
        }
    }

    void System::registerPublisher(const int chId, const MsgCbPtr &callback) {

        if (mmPublishers.count(chId) <= 0) {
            mmPublishers[chId] = vector<MsgCbPtr>();
        }
        mmPublishers[chId].push_back(callback);
    }

    void System::registerSubscriber(const int chId, const MsgCbPtr &callback) {

        if (mmSubscribers.count(chId) <= 0) {
            mmSubscribers[chId] = vector<MsgCbPtr>();
        }
        mmSubscribers[chId].push_back(callback);
    }

    void System::registerChannel(const int chId, const MsgCbPtr &callback) {

        if (mmChannels.count(chId) <= 0) {
            mmChannels[chId] = vector<MsgCbPtr>();
        }
        mmChannels[chId].push_back(callback);
    }

    void System::unregisterChannel(const int chId, const MsgCbPtr &callback) {

        if (mmChannels.contains(chId)) {
            vector<MsgCbPtr> vNewCbs;
            for (const auto& cbPtr : mmChannels[chId]) {
                if (callback != cbPtr) {
                    vNewCbs.push_back(cbPtr);
                }
            }
            mmChannels[chId] = vNewCbs;
        }
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

        MsgPtr msgLoadSettings = make_shared<Message>(ID_CH_PARAMS, ParameterServer::TOPIC, FCN_PS_LOAD, settings);
        mpParamServer = make_shared<ParameterServer>(shared_from_this());
        mpParamServer->receive(msgLoadSettings);
        this->registerChannel(ID_CH_PARAMS, mpParamServer);
    }

    void System::loadDatasets() {

        auto pCh = shared_from_this();
        auto msgGetParams = make_shared<MsgRequest>(ID_CH_PARAMS, pCh,
                                                    ParameterServer::TOPIC,FCN_PS_REQ, PARAM_DS);
        mpParamServer->receive(msgGetParams);
        if (mpTempParam && mpTempParam->getName() == "DS") {

            size_t nDs = mpTempParam->getAllChildKeys().size();
            for (size_t i = 0; i < nDs; i++) {
                shared_ptr<DataStore> pDataProvider = make_shared<DataStore>(pCh);
                string currIdxStr = to_string(i);
                string msgTarget = string(PARAM_DS) + "/" + currIdxStr;
                msgGetParams = make_shared<MsgRequest>(ID_CH_PARAMS, pDataProvider,
                                                       ParameterServer::TOPIC, FCN_PS_REQ, msgTarget);
                mpParamServer->receive(msgGetParams);

                mmpDataStores.insert(make_pair(pDataProvider->getName(), pDataProvider));
                this->registerChannel(ID_CH_DS, pDataProvider);
            }
        }
    }

    void System::loadSensors() {

        loadCameras();

        for (const auto& pSensor : mmpSensors) {
            this->registerChannel(ID_CH_SENSORS, pSensor.second);
            this->registerPublisher(ID_TP_SDATA, pSensor.second);
        }
    }

    void System::loadCameras() {

        auto msgGetParams = make_shared<MsgRequest>(ID_CH_PARAMS, shared_from_this(),
                                                    ParameterServer::TOPIC, FCN_PS_REQ, PARAM_CAM);
        mpParamServer->receive(msgGetParams);
        if (mpTempParam && mpTempParam->getName() == "Camera") {

            map<string, ParamPtrW> mParams = mpTempParam->getAllChildren();
            for (const auto& camParam : mParams) {
                auto pCamParam = camParam.second.lock();
                if (pCamParam) {
                    auto pCamera = Camera::getCamera(pCamParam, shared_from_this(), camParam.first);
                    if (pCamera) {
                        mmpSensors.insert(make_pair(pCamera->getName(), pCamera));
                    }
                }
            }
        }
    }

    void System::loadRelations() {

        auto msgGetParams = make_shared<MsgRequest>(ID_CH_PARAMS, shared_from_this(),
                                                    ParameterServer::TOPIC, FCN_PS_REQ, PARAM_REL);
        mpParamServer->receive(msgGetParams);
        if (mpTempParam && mpTempParam->getName() == "Relations") {
            for (const auto& relParamPair : mpTempParam->getAllChildren()) {
                auto pRelParam = relParamPair.second.lock();
                if (pRelParam) {
                    auto pTrans = TF::PoseSE3::getTrans(pRelParam);
                    if (pTrans) {
                        mmpTrans.insert(make_pair(pTrans->getKey(), pTrans));
                    }
                }
            }
        }
    }

    void System::loadOutputs() {

        auto pChannel = shared_from_this();
        auto msgGetParams = make_shared<MsgRequest>(ID_CH_PARAMS, pChannel,
                                                    ParameterServer::TOPIC, FCN_PS_REQ, PARAM_OUT);
        mpParamServer->receive(msgGetParams);
        if (mpTempParam && mpTempParam->getName() == PARAM_OUT) {
            for (const auto& outParamPair : mpTempParam->getAllChildren()) {
                auto pParam = outParamPair.second.lock();
                if (pParam) {
                    OutputPtr pOutput = Output::getNewInstance(pParam, pChannel);
                    if (pOutput) {
                        // load output params
                        string paramKey = string(PARAM_OUT) + "/" + outParamPair.first;
                        msgGetParams = make_shared<MsgRequest>(ID_CH_PARAMS, pOutput, ParameterServer::TOPIC,
                                                               FCN_PS_REQ, paramKey);
                        mpParamServer->receive(msgGetParams);

                        mmpOutputs.insert(make_pair(pOutput->getName(), pOutput));
                        this->registerChannel(ID_CH_OUTPUT, pOutput);
                        this->registerSubscriber(ID_TP_OUTPUT, pOutput);

                        auto msgRunOutput = make_shared<MsgRequest>(ID_CH_OUTPUT,
                                                                    pChannel, Output::TOPIC, FCN_SYS_RUN);
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
            this->registerChannel(ID_CH_ATLAS, mpAtlas);
        }
        // Initialize Trajectory Manager
        if (!mpTrajManager) {
            mpTrajManager = make_shared<TrajManager>();
            this->registerChannel(ID_CH_TRAJECTORY, mpTrajManager);
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
                auto msgTrans = make_shared<MsgType<PosePtr>>(DEF_CAT, pTrans, msg->getTopic());
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

        auto msgStop = make_shared<Message>(ID_CH_SYS, TOPIC, FCN_SYS_STOP);

        for (const auto& pChPair : mmChannels) {
            auto pChannels = pChPair.second;
            for (const auto& pCh : pChannels) {
                if (pCh == shared_from_this()) {
                    continue;
                }
                if (pCh) {
                    pCh->receive(msgStop);
                }
            }
        }
    }


}