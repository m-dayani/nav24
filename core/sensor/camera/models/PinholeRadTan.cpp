//
// Created by masoud on 8/30/24.
//

#include <iostream>
#include <opencv2/calib3d.hpp>
#include "PinholeRadTan.hpp"

namespace NAV24 {

    std::vector<cv::KeyPoint> PinholeRadTan::UndistortKeyPoints(const std::vector<cv::KeyPoint> &vKPts) {

        std::vector<cv::KeyPoint> vKeysUn = vKPts;
        std::vector<cv::Point2f> vPts(vKPts.size());

        for(size_t i = 0; i < vKPts.size(); i++) vPts[i] = vKPts[i].pt;

//        std::cout << mK_cv << std::endl;
//        std::cout << mD_cv << std::endl;
        cv::undistortPoints(vPts, vPts, mK_cv, mD_cv, mR, mP);

        for(size_t i = 0; i < vKPts.size(); i++) vKeysUn[i].pt = vPts[i];

        return vKeysUn;
    }
} // NAV24
