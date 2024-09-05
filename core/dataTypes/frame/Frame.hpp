//
// Created by masoud on 4/28/24.
//

#ifndef NAV24_FRAME_HPP
#define NAV24_FRAME_HPP

#include "Image.hpp"
#include "trajectory/pose/Pose.hpp"
#include "Point2D.hpp"
#include "MatchedFeatures.hpp"
#include "FeatureGrid.hpp"

namespace NAV24 {

    class Frame : public SmartObject {
    public:
        Frame() : ts(-1.0), mpPose(nullptr), mvpObservations(), mId(idCounter++), mOptId(0) {}
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

    protected:
        std::vector<OB::ObsPtr> mvpObservations;
        PosePtr mpPose;
        double ts;
        const long mId;
        static long idCounter;
        unsigned long mOptId;
    };
    typedef std::shared_ptr<Frame> FramePtr;

    class FrameImgMono : public Frame {
    public:
        FrameImgMono(double _ts, const PosePtr& pose, const std::vector<OB::ObsPtr>& vObs);
        FrameImgMono(double _ts, const PosePtr& pose, const std::vector<OB::ObsPtr>& vObs, const ImagePtr& pImage);

        ImagePtr getImage() { return mpImage; }
        void setImage(const ImagePtr& pImg) { mpImage = pImg; }
        void deleteCvImage();
    protected:
        ImagePtr mpImage;
    };

    class FrameMonoGrid : public FrameImgMono {
    public:
        FrameMonoGrid(double _ts, const PosePtr& pose, const std::vector<OB::ObsPtr>& vObs) :
                FrameImgMono(_ts, pose, vObs), mpGrid() {}
        FrameMonoGrid(double _ts, const PosePtr& pose, const std::vector<OB::ObsPtr>& vObs, const ImagePtr& pImage) :
                FrameImgMono(_ts, pose, vObs, pImage), mpGrid() {}

        void setObservations(const std::vector<OB::ObsPtr> &mvpObservations) override;

        //std::shared_ptr<OB::FeatureGrid> getGrid() { return mpGrid; }
        std::vector<std::size_t> getFeaturesInArea(const OB::ObsPtr& pObs, float windowSize, int minLevel, int maxLevel);
    protected:
        std::shared_ptr<OB::FeatureGrid> mpGrid;
    };

} // NAV24

#endif //NAV24_FRAME_HPP
