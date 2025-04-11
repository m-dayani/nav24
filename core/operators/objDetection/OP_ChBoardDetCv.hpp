//
// Created by masoud on 4/28/24.
//

#ifndef NAV24_OP_CHBOARDDETCV_HPP
#define NAV24_OP_CHBOARDDETCV_HPP

#include <vector>

#include "Observation.hpp"
#include "Point3D.hpp"
#include "Operator.hpp"


namespace NAV24::OP {

    class ChBoardDetCv : public Operator {
    public:
        ChBoardDetCv(cv::Size gridSz, const cv::TermCriteria& criteria);
        explicit ChBoardDetCv(const ChannelPtr& pChannel);

        bool process(const cv::Mat& image, std::vector<OB::ObsPtr>& vpCorners);

    protected:
        cv::TermCriteria mCriteria;
        cv::Size mGridSize;
        cv::Size mWinSize;
        cv::Size mZeroZone;
    };

} // NAV24::OP

#endif //NAV24_OP_CHBOARDDETCV_HPP
