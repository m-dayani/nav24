//
// Created by masoud on 4/28/24.
//

#include "OP_ChBoardDetCv.hpp"
#include "Point2D.hpp"

#include <utility>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <glog/logging.h>

using namespace std;

namespace NAV24::OP {

    OP_ChBoardDetCv::OP_ChBoardDetCv(cv::Size gridSz, const cv::TermCriteria &criteria) :
        mGridSize(std::move(gridSz)), mCriteria(criteria),
        mWinSize(11, 11), mZeroZone(-1, -1) {


    }

    bool OP_ChBoardDetCv::process(const cv::Mat &gray, std::vector<OB::ObsPtr> &vpCorners) {

        vector<cv::Point2f> vCorners;
        bool res = cv::findChessboardCorners(gray, mGridSize, vCorners);

        if (res) {
            cv::cornerSubPix(gray, vCorners, mWinSize,mZeroZone, mCriteria);
            //cv::drawChessboardCorners(img, mGridSize, vCorners, res);
        }

        vpCorners.reserve(vCorners.size());
        for (const auto& imgPt : vCorners) {
            auto pObs = make_shared<OB::Point2D>(imgPt.x, imgPt.y);
            vpCorners.push_back(pObs);
        }

        return res;
    }
} // NAV24::OP
