//
// Created by masoud on 9/3/24.
//

#ifndef NAV24_OP_MAPINITIALIZATION_HPP
#define NAV24_OP_MAPINITIALIZATION_HPP

#include "Operator.hpp"
#include "OP_2ViewReconstruction.hpp"
#include "Observation.hpp"
#include "Frame.hpp"

namespace NAV24::OP {


    class MapInitializer : public Operator {
    public:
        explicit MapInitializer(const CalibPtr& pCalib, float sigma = DEF_2VR_SIGMA, int iterations = DEF_MAX_RANSAC_ITER);
        explicit MapInitializer(const CalibPtr& pCalib, const Params2VR& params2VR);

        bool reconstruct(const OB::VecObsPair& vpObs, FramePtr& pFrame1, FramePtr& pFrame2, std::vector<WO::WoPtr>& vpPt3D);
        bool reconstruct(FramePtr& pFrame1, FramePtr& pFrame2, std::vector<WO::WoPtr>& vpPt3D);

    protected:
        static void preprocObsPairs(const OB::VecObsPair& vpObs,
                             std::vector<cv::KeyPoint>& vkpt1, std::vector<cv::KeyPoint>& vkpt2,
                             std::vector<int>& vMatches12);
        static int preprocFrames(const FramePtr& pFrame1, const FramePtr& pFrame2,
                           std::vector<cv::KeyPoint>& vkpt1, std::vector<cv::KeyPoint>& vkpt2,
                           std::vector<int>& vMatches12);
        static void postProcMap(FramePtr& pFrame1, FramePtr& pFrame2, std::vector<WO::WoPtr>& vpPt3D, const OB::VecObsPair& vpObs,
                         const cv::Mat& R21, const cv::Mat& t21, const std::vector<cv::Point3f>& vP3D,
                         const std::vector<bool>& vbTriangulated);
        void postProcMap(FramePtr& pFrame1, FramePtr& pFrame2, std::vector<WO::WoPtr>& vpPt3D,
                         const cv::Mat& R21, const cv::Mat& t21, const std::vector<cv::Point3f>& vP3D,
                         const std::vector<bool>& vbTriangulated);
    protected:
        CalibPtr mpCalib;
        std::shared_ptr<TwoViewReconstruction> mpTVR;
    };
} // NAV24::OP

#endif //NAV24_OP_MAPINITIALIZATION_HPP
