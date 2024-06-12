//
// Created by masoud on 6/12/24.
//

#ifndef NAV24_VISUALIZATION_HPP
#define NAV24_VISUALIZATION_HPP

#include <opencv2/core.hpp>
#include "Pose.hpp"

namespace NAV24 {
    class Visualization {
    public:
        static void drawGrid(cv::Mat& img, const cv::Mat& K, const cv::Mat& D,
                             const PosePtr& pPose_cw, int nx, int ny, float scale);

    };
} // NAV24

#endif //NAV24_VISUALIZATION_HPP
