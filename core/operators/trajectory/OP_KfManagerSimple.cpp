//
// Created by masoud on 4/6/25.
//

#include "OP_KfManagerSimple.hpp"


using namespace std;

namespace NAV24::OP {

    KfManagerSimple::KfManagerSimple() : Operator(),
            mThDistPose(DEF_KFMS_TH_DIST_POSE), mThTime(DEF_KFMS_TH_TIME),
            mThNumTrackedMps(DEF_KFMS_TH_N_TMPS), mMedPxd(DEF_KFMS_TH_MED_PXD),
            mLastPoseTable() {}

    KfManagerSimple::KfManagerSimple(const ChannelPtr &pChannel) : Operator(pChannel),
            mThDistPose(DEF_KFMS_TH_DIST_POSE), mThTime(DEF_KFMS_TH_TIME),
            mThNumTrackedMps(DEF_KFMS_TH_N_TMPS), mMedPxd(DEF_KFMS_TH_MED_PXD),
            mLastPoseTable() {}

    void KfManagerSimple::checkPose(PosePtr &pPose) {

        if (pPose) {

            string trajName = pPose->getName();
            if (mLastPoseTable.contains(trajName)) {

                auto poseLevel = mLastPoseTable[trajName];
                if (poseLevel.contains(1) && pPose->getLevel() == 0) {

                    // only consider the first level (keyframes)
                    auto pLastPose = poseLevel[1];

                    bool c1 = abs(pPose->getTimestamp() - pLastPose->getTimestamp()) * 1e-9 > mThTime;

                    Eigen::Vector3d last_t_rs, t_rs;
                    last_t_rs = pLastPose->getPose().block<3, 1>(0, 3);
                    t_rs = pPose->getPose().block<3, 1>(0, 3);
                    double dist_pose = (last_t_rs - t_rs).norm();

                    bool c2 = dist_pose > mThDistPose;

                    if (c1 || c2) {
                        pPose->incLevel();
                        mLastPoseTable[trajName][1] = pPose;
                    }
                }
            }
            else {
                mLastPoseTable[trajName] = map<int, PosePtr>();
                mLastPoseTable[trajName].insert(make_pair(1, pPose));
                if (pPose->getLevel() <= 0) {
                    pPose->incLevel();
                }
            }
        }
    }

    std::shared_ptr<KfManagerSimple> KfManagerSimple::getInstance(const ChannelPtr& pChannel,
                                                                  const ParamPtr &pParams) {

        shared_ptr<KfManagerSimple> pKfManagerSimple = make_shared<KfManagerSimple>(pChannel);
        auto msgConfig = make_shared<MsgConfig>(ID_CH_PARAMS, pParams);
        pKfManagerSimple->receive(msgConfig);

        return pKfManagerSimple;
    }

    void KfManagerSimple::receive(const MsgPtr &msg) {

        if (msg) {
            if (dynamic_pointer_cast<MsgConfig>(msg)) {
                this->setup(msg);
            }
        }
    }

    void KfManagerSimple::setup(const MsgPtr &configMsg) {

        if (configMsg && dynamic_pointer_cast<MsgConfig>(configMsg)) {
            auto pParam = dynamic_pointer_cast<MsgConfig>(configMsg)->getConfig();

            auto ppName = find_param<ParamType<string>>("name", pParam);
            mName = (ppName) ? ppName->getValue() : DEF_KFMS_NAME;

            // checking the operator's name
            if (mName != DEF_KFMS_NAME) {
                return;
            }

            auto ppThDistPose = find_param<ParamType<double>>("th_dist_pose", pParam);
            mThDistPose = (ppThDistPose) ? (float) ppThDistPose->getValue() : DEF_KFMS_TH_DIST_POSE;
            auto ppThTime = find_param<ParamType<double>>("th_time", pParam);
            mThTime = (ppThTime) ? (float) ppThTime->getValue() : DEF_KFMS_TH_TIME;
            auto ppNumTrackedMps = find_param<ParamType<int>>("th_n_tracked_mps", pParam);
            mThNumTrackedMps = (ppNumTrackedMps) ? ppNumTrackedMps->getValue() : DEF_KFMS_TH_N_TMPS;
            auto ppMedPxd = find_param<ParamType<int>>("th_med_pxd", pParam);
            mMedPxd = (ppMedPxd) ? ppMedPxd->getValue() : DEF_KFMS_TH_MED_PXD;
        }
    }

    void KfManagerSimple::handleRequest(const MsgPtr &) {

    }

    void KfManagerSimple::run() {

    }

} // NAV24::OP

