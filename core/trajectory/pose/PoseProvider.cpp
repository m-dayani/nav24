//
// Created by masoud on 6/19/24.
//

#include <Eigen/Dense>

#include "PoseProvider.hpp"
#include "DataStore.hpp"
#include "ParameterBlueprint.h"
#include "DataConversion.hpp"
#include "Pose.hpp"

using namespace std;

namespace NAV24 {

    PoseProvider::PoseProvider(const ChannelPtr &pChannel) : Sensor(pChannel),
        tsFactor(1.0), mbPosFirst(true), mbQwFirst(true) {}

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
            string poseExt = boost::filesystem::extension(boost::filesystem::path(mPoseFile));
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
                auto sender = pReq1->getCallback();
                if (sender) {

                    string line = mpPoseDS->getNextLine();
                    istringstream iss{line};
                    double ts = -1;
                    double px = 0, py = 0, pz = 0;
                    double qw = 0, qx = 0, qy = 0, qz = 0;

                    iss >> ts >> px >> py >> pz >> qw >> qx >> qy >> qz;

                    if (ts >= 0) {
                        Eigen::Vector3d t_wc;
                        t_wc << px, py, pz;

                        Eigen::Quaterniond quat(qw, qx, qy, qz);
                        Eigen::Matrix3d R_wc = quat.toRotationMatrix();

                        auto pPose = make_shared<TF::PoseSE3>("w", "c", ts, R_wc, t_wc);

                        auto msgPose = make_shared<MsgType<PosePtr>>(ID_CH_SENSORS, pPose);
                        sender->receive(msgPose);
                    }
                }
            }
        }
    }

    void PoseProvider::getNextBr(MsgPtr msg) {

    }

    void PoseProvider::reset() {

    }

    std::string PoseProvider::printStr(const std::string &prefix) const {
        return Sensor::printStr(prefix);
    }

    void PoseProvider::handleRequest(const MsgPtr &reqMsg) {

    }

    void PoseProvider::run() {

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
} // NAV24

