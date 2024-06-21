//
// Created by masoud on 6/17/24.
//

#ifndef NAV24_MAPVIEWER_HPP
#define NAV24_MAPVIEWER_HPP

#include <vector>
#include <memory>
#include <queue>
#include <set>
#include <pangolin/pangolin.h>
#include <Eigen/Dense>

#include "Output.hpp"
#include "Frame.hpp"


namespace NAV24 {

    class MapViewer : public Output {
    public:
        MapViewer(const ChannelPtr& pChannel);
        void receive(const MsgPtr &msg) override;

    protected:
        void stop() override;

        bool isStopped() override;
        void setup(const MsgPtr &msg) override;

        void handleRequest(const MsgPtr &msg) override;

        void requestStop(const std::string &channel) override;

        void run() override;

        void drawPose(const PosePtr& pPose) const;
        void drawWorldObject(const WO::woPtr &pWo) const;
        void drawTrajectory(const std::vector<FramePtr>& vpFrame);

    private:
        bool mbDisabled;
        float mFrameSize;
        float mFrameLineWidth;
        float mGraphLineWidth;
        float mPointSize;
        float mCameraSize;
        float mCameraLineWidth;
        float mViewpointX;
        float mViewpointY;
        float mViewpointZ;
        float mViewpointF;

        std::mutex mMtxPoseQueue;
        std::set<PosePtr> mspPose;
        std::mutex mMtxWoQueue;
        std::set<WO::woPtr> mspWorldObjects;
    };
} // NAV24

#endif //NAV24_MAPVIEWER_HPP
