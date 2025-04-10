//
// Created by masoud on 6/19/24.
//

#include <Eigen/Dense>

#include "PoseProvider.hpp"
#include "DataStore.hpp"
#include "ParameterBlueprint.h"
//#include "DataConversion.hpp"
#include "Pose.hpp"
#include "ParameterServer.hpp"

using namespace std;

namespace NAV24 {

    PoseProvider::PoseProvider(const ChannelPtr &pChannel) : Sensor(pChannel),
        tsFactor(1.0), mbPosFirst(true), mbQwFirst(true), mvpPoseHolder() {}

    void PoseProvider::receive(const MsgPtr &msg) {
        Sensor::receive(msg);
    }

    void PoseProvider::setup(const MsgPtr &msg) {
        Sensor::setup(msg);

        if (!msg || !dynamic_pointer_cast<MsgConfig>(msg)) {
            DLOG(WARNING) << "PoseProvider::setup, bad config message, abort\n";
            return;
        }

        // We only need to load image path params here
        if (msg->getTopic() != DataStore::TOPIC) {
            DLOG(INFO) << "PoseProvider::setup, No image paths message, abort\n";
            return;
        }

        auto pConfig = dynamic_pointer_cast<MsgConfig>(msg);
        auto pParamDS = pConfig->getConfig();
        if (pParamDS) {
            // Sequence base
            auto pSeqBase = find_param<ParamType<string>>(PKEY_SEQ_BASE, pParamDS);
            if (pSeqBase) {
                mSeqBase = pSeqBase->getValue();
            }
            // Pose file
            auto pFileName = find_param<ParamType<string>>(PKEY_POSE_FILE, pParamDS);
            if (pFileName) {
                mPoseFile = pFileName->getValue();
            }
            // Ts Factor
            auto pTsFactor = find_param<ParamType<double>>(PKEY_TS_FACT, pParamDS);
            if (pTsFactor) {
                tsFactor = pTsFactor->getValue();
            }

            auto pPosFirst = find_param<ParamType<int>>(PKEY_POS_FIRST, pParamDS);
            mbPosFirst = (pPosFirst) ? pPosFirst->getValue() : mbPosFirst;

            auto pQwFirst = find_param<ParamType<int>>(PKEY_QW_FIRST, pParamDS);
            mbQwFirst = (pQwFirst) ? pQwFirst->getValue() : mbQwFirst;

            // Create the image data store
//            string poseExt = boost::filesystem::extension();
            auto pose_path = boost::filesystem::path(mPoseFile);
            string poseExt = pose_path.extension().string();
            if (poseExt.empty()) {
                poseExt = ".txt";
            }

            mpPoseDS = make_shared<TabularTextDS>(mSeqBase, mPoseFile, poseExt);
            if (mpPoseDS) {
                mpPoseDS->open();
            }
        }
    }

    void PoseProvider::getNext(MsgPtr pReq) {

        if (pReq && mpPoseDS) {
            auto pReq1 = dynamic_pointer_cast<MsgRequest>(pReq);
            if (pReq1) {
                auto senderCb = pReq1->getCallbackFun();
                if (senderCb) {

                    MsgPtr msgPose;
                    this->createPoseMsg(msgPose);
                    if (msgPose) {
                        senderCb(msgPose);
                    }
                }
            }
        }
    }

    void PoseProvider::getNextBr(MsgPtr) {

    }

    void PoseProvider::reset() {

    }

    std::string PoseProvider::printStr(const std::string &prefix) const {
        return Sensor::printStr(prefix);
    }

    void PoseProvider::handleRequest(const MsgPtr &) {

    }

    void PoseProvider::run() {

        DLOG(INFO) << "PoseProvider::run, started\n";

        while (!this->isStopped()) {

            MsgPtr pPoseMsg;
            this->createPoseMsg(pPoseMsg);

            if (pPoseMsg) {
                mpChannel->publish(pPoseMsg);

                auto pPose = dynamic_pointer_cast<MsgType<PosePtr>>(pPoseMsg)->getData();
                if (pPose) {
//                    mvpPoseHolder.push_back(pPose);
                    this->runDelay(static_cast<long>(pPose->getTimestamp()));
                }
            }
        }

        DLOG(INFO) << "PoseProvider::run, stopped\n";
    }

    ParamPtr PoseProvider::getPoseParams(const std::string& seqBase, const std::string& gtFile, double tsFact,
                                         bool posFirst, bool qwFirst, std::vector<ParamPtr>& vpParams) {
        ParamPtr pParam = make_shared<Parameter>(PKEY_IMG_PATHS, nullptr, Parameter::NodeType::MAP_NODE);

        auto pSeqBase = make_shared<ParamType<string>>(PKEY_SEQ_BASE, pParam, seqBase);
        auto pGtFile = make_shared<ParamType<string>>(PKEY_POSE_FILE, pParam, gtFile);
        auto pTsFactor = make_shared<ParamType<double>>(PKEY_TS_FACT, pParam, tsFact);

        auto pPosFirst = make_shared<ParamType<int>>(PKEY_POS_FIRST, pParam, posFirst);
        auto pQwFirst = make_shared<ParamType<int>>(PKEY_QW_FIRST, pParam, qwFirst);

        pParam->insertChild(PKEY_SEQ_BASE, pSeqBase);
        pParam->insertChild(PKEY_POSE_FILE, pGtFile);
        pParam->insertChild(PKEY_TS_FACT, pTsFactor);
        pParam->insertChild(PKEY_POS_FIRST, pPosFirst);
        pParam->insertChild(PKEY_QW_FIRST, pQwFirst);

        vpParams.push_back(pQwFirst);
        vpParams.push_back(pPosFirst);
        vpParams.push_back(pGtFile);
        vpParams.push_back(pSeqBase);
        vpParams.push_back(pTsFactor);
        vpParams.push_back(pParam);

        return pParam;
    }

    PoseProvider::~PoseProvider() {
        if (mpPoseDS) {
            mpPoseDS->close();
            mpPoseDS = nullptr;
        }
    }

    void PoseProvider::createPoseMsg(MsgPtr& pPoseMsg) {

        string line = mpPoseDS->getNextLine();
        if (line.empty() || line[0] == '#') {
//            DLOG(INFO) << "PoseProvider::getNext, empty line or comment: " << line << "\n";
            return;
        }

        // replace all ',' in case of csv files
        std::replace(line.begin(), line.end(), ',', ' ');
        istringstream iss{line};
        double ts = -1;
//                    char c = ',';
        double px = 0, py = 0, pz = 0;
        double qw = 0, qx = 0, qy = 0, qz = 0;

        iss >> ts >> px >> py >> pz >> qw >> qx >> qy >> qz;

        if (!mbQwFirst) {
            // swap qz and qw
            double qq = qz;
            qz = qw;
            qw = qq;
        }

        if (ts >= 0) {
            Eigen::Vector3d t_wc;
            t_wc << px, py, pz;

            Eigen::Quaterniond quat(qw, qx, qy, qz);
            Eigen::Matrix3d R_wc = quat.toRotationMatrix();

            auto pPose = make_shared<TF::PoseSE3>(ts, R_wc, t_wc);

            pPoseMsg = make_shared<MsgType<PosePtr>>(ID_TP_SDATA, pPose);
        }
    }

    std::shared_ptr<Sensor>
    PoseProvider::getPoseProvider(const ParamPtr &pParams, const ChannelPtr &pChannel) {

        shared_ptr<PoseProvider> pPoseProvider = make_shared<PoseProvider>(pChannel);
        pChannel->registerPublisher(ID_TP_OUTPUT, pPoseProvider);
        pChannel->registerChannel(ID_CH_SENSORS, pPoseProvider);

        // setup
        auto pMsgConfig = make_shared<MsgConfig>(ID_CH_SENSORS, pParams, Sensor::TOPIC);
        pPoseProvider->receive(pMsgConfig);

        auto fp = [pPoseProvider](auto && PH1) { pPoseProvider->receive(std::forward<decltype(PH1)>(PH1)); };
        MsgPtr msgConfPaths = make_shared<MsgRequest>(ID_CH_DS, fp, DataStore::TOPIC,
                                                      FCN_DS_REQ, TAG_DS_GET_PATH_GT);
        pChannel->send(msgConfPaths);

        return pPoseProvider;
    }
} // NAV24

