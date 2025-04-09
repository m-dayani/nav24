//
// Created by masoud on 4/6/25.
// Simple Keyframe Manager:
//      - Insert new keyframes (ascend pose to higher levels)
//      - Keyframe culling

#ifndef NAV24_OP_KFMANAGERSIMPLE_HPP
#define NAV24_OP_KFMANAGERSIMPLE_HPP


#include "Operator.hpp"
#include "Pose.hpp"
#include "Message.hpp"


namespace NAV24::OP {
//namespace OP {

#define DEF_KFMS_NAME "traj_kfms"
#define DEF_KFMS_TH_DIST_POSE 0.3f
#define DEF_KFMS_TH_TIME 1.0f
#define DEF_KFMS_TH_N_TMPS 50
#define DEF_KFMS_TH_MED_PXD 20

    class KfManagerSimple : public Operator, public MsgCallback {
    public:
        KfManagerSimple();

        void checkPose(PosePtr& pPose);

        static std::shared_ptr<KfManagerSimple> getInstance(const ParamPtr& pParams);

        void receive(const MsgPtr &msg) override;

    protected:
        void setup(const MsgPtr &configMsg) override;

        void handleRequest(const MsgPtr &reqMsg) override;

        void run() override;

    private:
        std::string mName;

        // min pose distance
        float mThDistPose;
        // min time between poses
        float mThTime;
        // min number of tracked map points
        int mThNumTrackedMps;
        // median pixel displacement (between tracked features, reference: last key frame)
        int mMedPxd;

        // (trajectory group name, pose level, pose)
        std::map<std::string, std::map<int, PosePtr>> mLastPoseTable;
    };

//} // OP
} // NAV24


#endif //NAV24_OP_KFMANAGERSIMPLE_HPP
