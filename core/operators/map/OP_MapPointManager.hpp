//
// Created by masoud on 2/24/25.
//
// Manage Map Points: Create/Delete/Invalidate/...

#ifndef NAV24_OP_MAPPOINTMANAGER_HPP
#define NAV24_OP_MAPPOINTMANAGER_HPP

#include <Eigen/Eigen>

#include "Operator.hpp"
#include "Frame.hpp"
#include "OP_FtAssocOrbSlam.hpp"


namespace NAV24::OP {

#define OP_MPM_DEF_TH_N_OBS 3
#define OP_MPM_DEF_TH_N_KF 3

    class MapPointManager : public Operator {
    public:
        explicit MapPointManager(const ChannelPtr& pChannel);

        void checkNewKeyFrame(const FramePtr& pKF, std::vector<WO::WoPtr>& vpPoints3d);

        void receive(const MsgPtr &msg) override;

    private:
        static bool triangulate(Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2, Eigen::Matrix<float,3,4> &Tc1w,
                                Eigen::Matrix<float,3,4> &Tc2w, Eigen::Vector3f &x3D);
        static void findBestCovisibility(const FramePtr& pKF, std::vector<FramePtr>& vpCovisKFs);

        void createMapPoints(const FramePtr &pKF, std::vector<WO::WoPtr>& vpPoints3d);
        static void mapPointCulling(const std::vector<WO::WoPtr>& vpWorldObjs, int nTotalKFs);
//        void updateCovisibilityGraph();

        static float computeMedianDepth(const FramePtr& pFrame);

    private:
        std::shared_ptr<OP::FtAssocOrbSlam> mpFtMatcher;
        CalibPtrRO mpCamCalib;
    };

} // NAV24::OP

#endif //NAV24_OP_MAPPOINTMANAGER_HPP
