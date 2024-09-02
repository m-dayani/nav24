//
// Created by masoud on 4/29/24.
//

#ifndef NAV24_PROBLEM_HPP
#define NAV24_PROBLEM_HPP

#include <memory>
#include <opencv2/core.hpp>

#include "Frame.hpp"
#include "WorldObject.hpp"

namespace NAV24 {

    class Problem {
    public:

        bool solved = false;
    };
    typedef std::shared_ptr<Problem> ProblemPtr;

    class PR_CamCalibCV : public Problem {
    public:

        std::vector<FramePtr> mvpFrames;
        std::vector<WO::WoPtr> mvpWorldObjs;
        cv::Size mImgSize;

        cv::Mat mK;
        cv::Mat mDistCoeffs;
        std::vector<cv::Point3d> rvecs, tvecs;
    };

} // NAV24

#endif //NAV24_PROBLEM_HPP
