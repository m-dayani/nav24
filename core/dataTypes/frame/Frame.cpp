//
// Created by masoud on 4/28/24.
//

#include "Frame.hpp"

#include <utility>

namespace NAV24 {

    Frame::Frame(double _ts, PosePtr pose, const std::vector<OB::ObsPtr> &vObs) : ts(_ts), mpPose(std::move(pose)),
        mvpObservations(vObs) {}

    const std::vector<OB::ObsPtr> &Frame::getObservations() const {
        return mvpObservations;
    }

    void Frame::setObservations(const std::vector<OB::ObsPtr> &vpObs) {
        Frame::mvpObservations = vpObs;
    }

    const PosePtr &Frame::getPose() const {
        return mpPose;
    }

    void Frame::setPose(const PosePtr &pose) {
        Frame::mpPose = pose;
    }

    FrameImgMono::FrameImgMono(double _ts, const PosePtr &pose, const std::vector<OB::ObsPtr> &vObs) :
        Frame(_ts, pose, vObs) {}

    FrameImgMono::FrameImgMono(double _ts, const PosePtr &pose, const std::vector<OB::ObsPtr> &vObs,
                               const ImagePtr &pImage) : FrameImgMono(_ts, pose, vObs) {
        mpImage = pImage;
    }
} // NAV24
