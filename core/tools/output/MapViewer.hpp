//
// Created by masoud on 6/17/24.
//

#ifndef NAV24_MAPVIEWER_HPP
#define NAV24_MAPVIEWER_HPP

#include <vector>
#include <memory>
#include <queue>
#include <set>

#include <Eigen/Dense>
#ifdef LIB_PANGOLIN_FOUND
#include <pangolin/pangolin.h>
#endif

#include "Output.hpp"
#include "Frame.hpp"


namespace NAV24 {

    typedef std::map<std::string, std::vector<PosePtr>> MapNamedPose;

    class MapViewer : public Output {
    public:
        explicit MapViewer(const ChannelPtr& pChannel);
        void receive(const MsgPtr &msg) override;

    protected:
        void stop() override;

        bool isStopped() override;
        void setup(const MsgPtr &msg) override;

        void insertPoses(const std::vector<PosePtr>& vpPose);
        void retrievePoses(MapNamedPose& poseTable);

        void handleRequest(const MsgPtr &msg) override;

        void requestStop(const std::string &channel) override;

        void run() override;

        void drawPose(const PosePtr& pPose) const;
        void drawPoseFrame(const PosePtr& pPose) const;
        void drawWorldObject(const WO::WoPtr &pWo) const;
        void drawTrajectory(const std::vector<PosePtr>& spPose, const std::vector<float> &color) const;
        void drawTrajectories(const MapNamedPose& poseTable) const;

#ifdef LIB_PANGOLIN_FOUND
        void getLastOpenGlCamera(pangolin::OpenGlMatrix& Twc);
#endif

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
        MapNamedPose mPoseTable;
        std::mutex mMtxWoQueue;
        std::set<WO::WoPtr> mspWorldObjects;

        //std::string mWinName;

        PosePtr mLastPose;
        int mSetFirstPoseState;

        std::map<std::string, std::vector<float>> mmTrajColors;
    };
} // NAV24

#endif //NAV24_MAPVIEWER_HPP
