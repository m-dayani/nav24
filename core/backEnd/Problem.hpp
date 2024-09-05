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
        Problem() : solved(false), mMtxSolved(), nIterations(5), thConverge(0.1f) {}

        [[nodiscard]] virtual bool isSolved() const { return solved; }
        virtual void setSolved(const bool state) {
            mMtxSolved.lock();
            solved = state;
            mMtxSolved.unlock();
        }

        [[nodiscard]] int getNumIter() const { return nIterations; }
        void setNumIter(const int numIter) { nIterations = numIter; }

        [[nodiscard]] float getThConverge() const { return thConverge; }
        void setThConverge(const float thConv) { thConverge = thConv; }

    protected:
        bool solved;
        std::mutex mMtxSolved;
        int nIterations;
        float thConverge;
    };
    typedef std::shared_ptr<Problem> ProblemPtr;

    class PR_VBA : public Problem {
    public:
        PR_VBA(const std::vector<FramePtr>& vpFrames, const CalibPtr& pCalib) : mvpFrames(vpFrames), mpCalib(pCalib) {}
        std::vector<FramePtr> mvpFrames;
        CalibPtr mpCalib;
        bool mbRobust = false;
    };

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
