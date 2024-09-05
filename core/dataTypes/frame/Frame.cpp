//
// Created by masoud on 4/28/24.
//

#include "Frame.hpp"

#include <utility>

using namespace std;

namespace NAV24 {

    long Frame::idCounter;

    Frame::Frame(double _ts, PosePtr pose, const std::vector<OB::ObsPtr> &vObs) :
        ts(_ts), mpPose(std::move(pose)), mvpObservations(vObs), mId(idCounter++), mOptId(0) {}

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

//    void Frame::addObservation(const OB::ObsPtr &pObs) {
//        mvpObservations.push_back(pObs);
//    }

    FrameImgMono::FrameImgMono(double _ts, const PosePtr &pose, const std::vector<OB::ObsPtr> &vObs) :
        Frame(_ts, pose, vObs) {}

    FrameImgMono::FrameImgMono(double _ts, const PosePtr &pose, const std::vector<OB::ObsPtr> &vObs,
                               const ImagePtr &pImage) : FrameImgMono(_ts, pose, vObs) {
        mpImage = pImage;
    }

    void FrameImgMono::deleteCvImage() {
        if (mpImage) {
            mpImage->mImage = cv::Mat();
        }
    }

    void FrameMonoGrid::setObservations(const std::vector<OB::ObsPtr> &vpObservations) {
        Frame::setObservations(vpObservations);
        mpGrid = std::make_shared<OB::FeatureGrid>(mvpObservations);
    }

    std::vector<std::size_t>
    FrameMonoGrid::getFeaturesInArea(const OB::ObsPtr &pObs, float windowSize, int minLevel, int maxLevel) {

        if (pObs && dynamic_pointer_cast<OB::Point2D>(pObs)) {
            auto pt2d = dynamic_pointer_cast<OB::Point2D>(pObs);
            cv::Point2f kpt = pt2d->getPointUd();
            if (mpGrid) {
                return mpGrid->getFeaturesInArea(kpt.x, kpt.y, windowSize, minLevel, maxLevel);
            }
        }
        return {};
    }
} // NAV24
