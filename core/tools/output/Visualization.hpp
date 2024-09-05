//
// Created by masoud on 6/12/24.
//

#ifndef NAV24_VISUALIZATION_HPP
#define NAV24_VISUALIZATION_HPP

#include <vector>
#include <opencv2/core.hpp>

#include "trajectory/pose/Pose.hpp"
#include "WorldObject.hpp"
#include "Calibration.hpp"
#include "Frame.hpp"
#include "MatchedFeatures.hpp"

namespace NAV24 {
    class Visualization {
    public:
        static void projectMap(cv::Mat& img, const PosePtr& pPose_cw, const CalibPtr& pCalib,
                               const std::vector<WO::WoPtr>& vpMapPts);

//        static void drawGrid(cv::Mat& img, const cv::Mat& K, const cv::Mat& D,
//                             const PosePtr& pPose_cw, int nx, int ny, float scale);

        static void drawKeyPoints(cv::Mat& img, const FramePtr& pFrame, bool drawDistorted = false);

        static void drawMatchedTracks(cv::Mat& img, const OB::FtTracksPtr& pTracks);

    protected:
        static std::vector<cv::Scalar> mColorPallet;

    };
} // NAV24

#endif //NAV24_VISUALIZATION_HPP
