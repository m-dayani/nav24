//
// Created by masoud on 4/28/24.
//

#ifndef NAV24_FRAME_HPP
#define NAV24_FRAME_HPP

#include <DBoW2/DBoW2.h>

#include "Image.hpp"
#include "trajectory/pose/Pose.hpp"
#include "Point2D.hpp"
#include "MatchedFeatures.hpp"
#include "FeatureGrid.hpp"

namespace NAV24 {

    class Frame : public SmartObject {
    public:
        Frame() : ts(-1.0), mId(idCounter++), mOptId(0), mvpObservations(), mpPose(nullptr), mLevel(0) {}
        Frame(double _ts, PosePtr  pose, const std::vector<OB::ObsPtr>& vObs);

        [[nodiscard]] const std::vector<OB::ObsPtr> &getObservations() const;
        virtual void setObservations(const std::vector<OB::ObsPtr> &mvpObservations);
//        void addObservation(const OB::ObsPtr& pObs);

        [[nodiscard]] const PosePtr &getPose() const;
        void setPose(const PosePtr &pose);

        [[nodiscard]] double getTs() const { return ts; }

        [[nodiscard]] long getId() const { return mId; }

        [[nodiscard]] unsigned long getOptId() const { return mOptId; }
        void setOptId(const unsigned long &optId) { mOptId = optId; }

        [[nodiscard]] uint getLevel() const { return mLevel; }
        void incLevel() { mLevel++; }

        [[nodiscard]] std::shared_ptr<Frame> getPrevFrame() const;
        void setPrevFrame(const std::shared_ptr<Frame>& pFrame) { mpPrevFrame = pFrame; }
        void setNextFrame(const std::shared_ptr<Frame>& pFrame) { mpNextFrame = pFrame; }

        virtual void simplify();

        virtual void processKeyframe() {}

    protected:
        double ts;
        const long mId;
        unsigned long mOptId;

        std::vector<OB::ObsPtr> mvpObservations;
        PosePtr mpPose;

        uint mLevel;

        std::weak_ptr<Frame> mpPrevFrame, mpNextFrame;

        static long idCounter;
    };
    typedef std::shared_ptr<Frame> FramePtr;

    class FrameImgMono : public Frame {
    public:
        FrameImgMono(double _ts, const PosePtr& pose, const std::vector<OB::ObsPtr>& vObs);
        FrameImgMono(double _ts, const PosePtr& pose, const std::vector<OB::ObsPtr>& vObs, const ImagePtr& pImage);

        ImagePtr getImage() { return mpImage; }
//        void setImage(const ImagePtr& pImg) { mpImage = pImg; }
        void deleteCvImage();

        void setMatches(const OB::MatchedObsPtr& pMatches12) { mpMatches12 = pMatches12; }
        OB::MatchedObsPtr getMatches() { return mpMatches12; }

        void simplify() override;

    protected:
        ImagePtr mpImage;
        OB::MatchedObsPtr mpMatches12;
    };

    // ORB-SLAM Frame (Monocular)
    class FrameMonoOS : public FrameImgMono {
    public:
        FrameMonoOS(double _ts, const PosePtr& pose, const std::vector<OB::ObsPtr>& vObs) :
                FrameImgMono(_ts, pose, vObs), mpGrid() {}
        FrameMonoOS(double _ts, const PosePtr& pose, const std::vector<OB::ObsPtr>& vObs, const ImagePtr& pImage) :
                FrameImgMono(_ts, pose, vObs, pImage), mpGrid() {}

        void setObservations(const std::vector<OB::ObsPtr> &mvpObservations) override;

        //std::shared_ptr<OB::FeatureGrid> getGrid() { return mpGrid; }
        std::vector<std::size_t> getFeaturesInArea(const OB::ObsPtr& pObs, float windowSize, int minLevel, int maxLevel);

        [[nodiscard]] DBoW2::FeatureVector getFtVecDBoW2() const { return mFtVecDBoW2; }
        void setFtVecDBoW2(const DBoW2::FeatureVector& ftVec) { mFtVecDBoW2 = ftVec; }
//        void computeFtVecDBoW2();
        [[nodiscard]] DBoW2::BowVector getBowVecDBoW2() const { return mBowVecDBoW2; }
        void setBowVecDBoW2(const DBoW2::BowVector& bowVec) { mBowVecDBoW2 = bowVec; }

        void processKeyframe() override;

    protected:
        std::shared_ptr<OB::FeatureGrid> mpGrid;
        DBoW2::FeatureVector mFtVecDBoW2;
        DBoW2::BowVector mBowVecDBoW2;
    };

} // NAV24

#endif //NAV24_FRAME_HPP
