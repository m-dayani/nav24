//
// Created by masoud on 8/30/24.
//

#ifndef NAV24_PINHOLERADTAN_HPP
#define NAV24_PINHOLERADTAN_HPP

#include "Pinhole.hpp"

namespace NAV24 {
    class PinholeRadTan : public Pinhole {
    public:
        explicit PinholeRadTan(const std::vector<float>& vParams) : Pinhole(vParams) {

            for (int i = 4; i < vParams.size(); i++) {
                mD_cv.at<float>(i-4) = vParams[i];
            }
        }

        std::vector<cv::KeyPoint> UndistortKeyPoints(const std::vector<cv::KeyPoint> &vKPts) override;
    };
} // NAV24

#endif //NAV24_PINHOLERADTAN_HPP
