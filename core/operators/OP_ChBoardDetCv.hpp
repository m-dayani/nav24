//
// Created by masoud on 4/28/24.
//

#ifndef NAV24_OP_CHBOARDDETCV_HPP
#define NAV24_OP_CHBOARDDETCV_HPP

#include <vector>

#include "Observation.hpp"
#include "Point3D.hpp"


namespace NAV24::OP {

    class OP_ChBoardDetCv {
    public:
        OP_ChBoardDetCv(cv::Size  gridSz, const cv::TermCriteria& criteria);

        bool process(const cv::Mat& image, std::vector<OB::ObsPtr>& vpCorners);

    protected:
        cv::TermCriteria mCriteria;
        cv::Size mGridSize;
        cv::Size mWinSize;
        cv::Size mZeroZone;
    };

} // NAV24::OP

#endif //NAV24_OP_CHBOARDDETCV_HPP
