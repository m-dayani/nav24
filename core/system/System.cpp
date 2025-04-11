//
// Created by root on 12/21/23.
//

#include "System.hpp"
#include "ParameterBlueprint.h"
#include "Camera.hpp"
#include "FrontEnd.hpp"
#include "ImageViewer.hpp"
#include "PoseProvider.hpp"

using namespace std;

namespace NAV24 {

    System::System() : mmChannels(), mmPublishers(), mmSubscribers(),
                       mpParamServer(), mmpDataStores(), mmpSensors(),
                       mmpTrans(), mmpOutputs(), mpThreads(),
                       mpTempParam(nullptr), mpAtlas(nullptr), mpTrajManager(nullptr) {
        mName = "System";
    }

    System::System(const string &settings) : System() {

        this->loadSettings(settings);
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

        mpParamServer = make_shared<ParameterServer>(shared_from_this(), settings);
        this->registerChannel(ID_CH_PARAMS, mpParamServer);
//        MsgPtr msgLoadSettings = make_shared<Message>(ID_CH_PARAMS, ParameterServer::TOPIC, FCN_PS_LOAD, settings);
//        mpParamServer->receive(msgLoadSettings);
    }

    void System::loadDatasets() {

        auto pCh = shared_from_this();
        auto fp = [this](auto && PH1) {
            receive(std::forward<decltype(PH1)>(PH1));
        };

        auto msgGetParams = make_shared<MsgRequest>(ID_CH_PARAMS, fp,
                                                    ParameterServer::TOPIC,FCN_PS_REQ, PARAM_DS);
        mpParamServer->receive(msgGetParams);
        if (mpTempParam) {

//            size_t nDs = mpTempParam->getAllChildKeys().size();
            for (const auto& dsInfo : mpTempParam->getAllChildren()) {

                auto pParamDs = dsInfo.second.lock();
                if (pParamDs) {
                    shared_ptr<DataStore> pDataProvider = make_shared<DataStore>(pCh, pParamDs);
                    mmpDataStores.insert(make_pair(pDataProvider->getName(), pDataProvider));
                    this->registerChannel(ID_CH_DS, pDataProvider);
                }

//                string currIdxStr = to_string(i);
//                string msgTarget = string(PARAM_DS) + "/" + currIdxStr;
//                auto fp1 = [pDataProvider](auto && PH1) { pDataProvider->receive(std::forward<decltype(PH1)>(PH1)); };
//                msgGetParams = make_shared<MsgRequest>(ID_CH_PARAMS, fp1,
//                                                       ParameterServer::TOPIC, FCN_PS_REQ, msgTarget);
//                mpParamServer->receive(msgGetParams);
            }
        }
    }

    void System::loadSensors() {

        loadCameras();
        loadPoseSensors();

        for (const auto& pSensor : mmpSensors) {
            this->registerChannel(ID_CH_SENSORS, pSensor.second);
            this->registerPublisher(ID_TP_SDATA, pSensor.second);
        }
    }

    void System::loadCameras() {

        auto pCh = shared_from_this();
        auto fp = [this](auto && PH1) {
            receive(std::forward<decltype(PH1)>(PH1));
        };
        auto msgGetParams = make_shared<MsgRequest>(ID_CH_PARAMS, fp,
                                                    ParameterServer::TOPIC, FCN_PS_REQ, PARAM_CAM);
        mpParamServer->receive(msgGetParams);
        if (mpTempParam) {

            for (const auto& camParam : mpTempParam->getAllChildren()) {
                auto pCamParam = camParam.second.lock();
                if (pCamParam) {
                    auto pCamera = Camera::getCamera(pCh, pCamParam);
                    if (pCamera) {
                        mmpSensors.insert(make_pair(pCamera->getName(), pCamera));
                    }
                }
            }
        }
    }

    void System::loadPoseSensors() {

        auto pCh = shared_from_this();
        auto fp = [this](auto && PH1) {
            receive(std::forward<decltype(PH1)>(PH1));
        };
        auto msgGetParams = make_shared<MsgRequest>(ID_CH_PARAMS, fp,
                                                    ParameterServer::TOPIC, FCN_PS_REQ, PARAM_POSE_SENSOR);
        mpParamServer->receive(msgGetParams);
        if (mpTempParam) {

            map<string, ParamPtrW> mParams = mpTempParam->getAllChildren();
            for (const auto& poseParam : mParams) {
                auto pPoseParam = poseParam.second.lock();
                if (pPoseParam) {
                    auto pPoseProvider = PoseProvider::getPoseProvider(pCh, pPoseParam);
                    if (pPoseProvider) {
                        mmpSensors.insert(make_pair(pPoseProvider->getName(), pPoseProvider));
                    }
                }
            }
        }
    }

    void System::loadRelations() {

        auto fp = [this](auto && PH1) {
            receive(std::forward<decltype(PH1)>(PH1));
        };
        auto msgGetParams = make_shared<MsgRequest>(ID_CH_PARAMS, fp,
                                                    ParameterServer::TOPIC, FCN_PS_REQ, PARAM_REL);
        mpParamServer->receive(msgGetParams);
        if (mpTempParam && mpTempParam->getName() == "Relations") {
            for (const auto& relParamPair : mpTempParam->getAllChildren()) {
                auto pRelParam = relParamPair.second.lock();
                if (pRelParam) {
                    auto pTrans = TF::PoseSE3::getTrans(pRelParam);
                    if (pTrans) {
                        mmpTrans.insert(make_pair(pTrans->getName(), pTrans));
                    }
                }
            }
        }
    }

    void System::loadOutputs() {

        auto pChannel = shared_from_this();
        auto fp = [this](auto && PH1) {
            receive(std::forward<decltype(PH1)>(PH1));
        };
        auto msgGetParams = make_shared<MsgRequest>(ID_CH_PARAMS, fp,
                                                    ParameterServer::TOPIC, FCN_PS_REQ, PARAM_OUT);
        mpParamServer->receive(msgGetParams);
        if (mpTempParam) {
            for (const auto& outParamPair : mpTempParam->getAllChildren()) {
                auto pParam = outParamPair.second.lock();
                if (pParam) {
                    OutputPtr pOutput = Output::getNewInstance(pChannel, pParam);
                    if (pOutput) {

                        mmpOutputs.insert(make_pair(pOutput->getName(), pOutput));
                        this->registerChannel(ID_CH_OUTPUT, pOutput);
                        this->registerSubscriber(ID_TP_OUTPUT, pOutput);
                        this->registerSubscriber(ID_TP_SDATA, pOutput);

                        auto msgRunOutput = make_shared<MsgRequest>(ID_CH_OUTPUT,
                                                                    fp, Output::TOPIC, FCN_SYS_RUN);
                        pOutput->receive(msgRunOutput);
                        //mpThreads.push_back(thread(&Output::run, pOutput));
                    }
                }
            }
        }
    }

    void System::initComponents() {

        auto pChannel = shared_from_this();
        auto fp = [this](auto && PH1) { receive(std::forward<decltype(PH1)>(PH1)); };

        // Initialize Atlas (Map/World Manager)
        if (!mpAtlas) {
            mpAtlas = make_shared<Atlas>(pChannel);
            this->registerChannel(ID_CH_ATLAS, mpAtlas);
            auto msgRunAtlas = make_shared<MsgRequest>(ID_CH_ATLAS,
                                                      fp, Atlas::TOPIC, FCN_SYS_RUN);
            mpAtlas->receive(msgRunAtlas);
        }

        // Initialize Trajectory Manager
        if (!mpTrajManager) {
            mpTrajManager = make_shared<TrajManager>(pChannel);
            this->registerChannel(ID_CH_TRAJECTORY, mpTrajManager);
            this->registerSubscriber(ID_TP_SDATA, mpTrajManager);
            auto msgRunTraj = make_shared<MsgRequest>(ID_CH_TRAJECTORY,
                                                        fp, TrajManager::TOPIC, FCN_SYS_RUN);
            mpTrajManager->receive(msgRunTraj);
        }
    }

    void System::loadOperators() {

        // Load all registered operators (do it after all essential components are initialized)
        auto fp = [this](auto && PH1) {
            receive(std::forward<decltype(PH1)>(PH1));
        };
        auto msgGetParams = make_shared<MsgRequest>(ID_CH_PARAMS, fp,
                                                    ParameterServer::TOPIC, FCN_PS_REQ, PARAM_OP);
        mpParamServer->receive(msgGetParams);
        if (mpTempParam) {
            auto vpOperators = mmChannels[ID_CH_OP];
            for (const auto& pParamInfo : mpTempParam->getAllChildren()) {

                auto pParam = pParamInfo.second.lock();
                if (pParam) {
                    auto msgConf = make_shared<MsgConfig>(ID_CH_PARAMS, pParam);
                    for (const auto& op : vpOperators) {
                        op->receive(msgConf);
                    }
                }
            }
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
                    case FCN_SYS_INIT_OP:
                        this->loadOperators();
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
        auto senderCb = msgReq->getCallbackFun();

        if (!senderCb) {
            DLOG(WARNING) << "System::handleRequests, bad sender\n";
            return;
        }

        int action = msg->getTargetId();
        string msgStr = msg->getMessage();

        if (action == FCN_GET_TRANS) {
            if (mmpTrans.count(msgStr) > 0) {

                auto pTrans = mmpTrans[msgStr];
                auto msgTrans = make_shared<MsgType<PosePtr>>(DEF_CAT, pTrans, msg->getTopic());
                senderCb(msgTrans);
            }
        }
    }

    void System::setup(const MsgPtr &) {

    }

    void System::run() {

    }

    void System::stop() {
        MsgCallback::stop();

        auto msgStop = make_shared<Message>(ID_CH_SYS, TOPIC, FCN_SYS_STOP);

        for (const auto &pChPair: mmChannels) {
            auto pChannels = pChPair.second;
            for (const auto &pCh: pChannels) {
                if (pCh == shared_from_this()) {
                    continue;
                }
                if (pCh) {
                    pCh->receive(msgStop);
                    // wait for the thread to be stopped
                    this_thread::sleep_for(chrono::microseconds(100));
                }
            }
        }

//        for (const auto& th : mpThreads) {
//            if (th->joinable()) {
//                th->join();
//            }
//        }
    }

}