//
// Created by masoud on 4/28/24.
//

#ifndef NAV24_FRAME_HPP
#define NAV24_FRAME_HPP

#include "Image.hpp"
#include "trajectory/pose/Pose.hpp"
#include "Point2D.hpp"

namespace NAV24 {

    class Frame {
    public:
        Frame() : ts(-1.0), mpPose(nullptr), mvpObservations() {}
        Frame(double _ts, PosePtr  pose, const std::vector<OB::ObsPtr>& vObs);

        [[nodiscard]] const std::vector<OB::ObsPtr> &getObservations() const;
        void setObservations(const std::vector<OB::ObsPtr> &mvpObservations);
        void addObservation(const OB::ObsPtr& pObs);

        [[nodiscard]] const PosePtr &getPose() const;
        void setPose(const PosePtr &pose);

        double getTs() const { return ts; }

    protected:
        std::vector<OB::ObsPtr> mvpObservations;
        PosePtr mpPose;
        double ts;
    };
    typedef std::shared_ptr<Frame> FramePtr;

    class FrameImgMono : public Frame {
    public:
        FrameImgMono(double _ts, const PosePtr& pose, const std::vector<OB::ObsPtr>& vObs);
        FrameImgMono(double _ts, const PosePtr& pose, const std::vector<OB::ObsPtr>& vObs, const ImagePtr& pImage);

        ImagePtr getImage() { return mpImage; }
        void setImage(const ImagePtr& pImg) { mpImage = pImg; }
    protected:
        ImagePtr mpImage;
    };

} // NAV24

#endif //NAV24_FRAME_HPP
