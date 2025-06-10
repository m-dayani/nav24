//
// Created by masoud on 4/28/24.
//

#include <utility>

#include "Frame.hpp"
#include "OP_VPR_DBoW2.hpp"


using namespace std;

namespace NAV24 {

    long Frame::idCounter;

    Frame::Frame(double _ts, PosePtr pose, const std::vector<OB::ObsPtr> &vObs) :
            ts(_ts), mId(idCounter++), mOptId(0), mvpObservations(vObs), mpPose(std::move(pose)), mLevel(0) {}

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

    std::shared_ptr<Frame> Frame::getPrevFrame() const { return mpPrevFrame.lock(); }

    void Frame::simplify() {

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

    void FrameImgMono::simplify() {
        Frame::simplify();
        mpImage->mImage = cv::Mat();
    }

    void FrameMonoOS::setObservations(const std::vector<OB::ObsPtr> &vpObservations) {
        Frame::setObservations(vpObservations);
        mpGrid = std::make_shared<OB::FeatureGrid>(mvpObservations);
    }

    std::vector<std::size_t>
    FrameMonoOS::getFeaturesInArea(const OB::ObsPtr &pObs, float windowSize, int minLevel, int maxLevel) {

        if (pObs && dynamic_pointer_cast<OB::Point2D>(pObs)) {
            auto pt2d = dynamic_pointer_cast<OB::Point2D>(pObs);
            cv::Point2f kpt = pt2d->getPointUd();
            if (mpGrid) {
                return mpGrid->getFeaturesInArea(kpt.x, kpt.y, windowSize, minLevel, maxLevel);
            }
        }
        return {};
    }

    // can't do this here, requires an OrbVocab instance
//    void FrameMonoOS::computeFtVecDBoW2() {
//        // convert observations to descriptors
//        vector<cv::Mat> vDesc;
//        OP::VPR_DBoW2::getDescriptors(mvpObservations, vDesc);
//        DBoW2::BowVector v1;
//        DBoW2::FeatureVector fv1;
//        int levelup = 0;
//        OrbVocabulary::transform(vDesc, v1, fv1, levelup);
//    }

    void FrameMonoOS::processKeyframe() {
//        this->computeFtVecDBoW2();
    }
} // NAV24
