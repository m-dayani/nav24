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
        explicit MapInitializer(const cv::Mat& k, float sigma = DEF_2VR_SIGMA, int iterations = DEF_MAX_RANSAC_ITER);
        explicit MapInitializer(const cv::Mat& k, const Params2VR& params2VR);

        bool reconstruct(const std::vector<std::pair<OB::ObsPtr,
                         OB::ObsPtr>>& vpObs, FramePtr& pFrame1, FramePtr& pFrame2,
                         std::vector<WO::WoPtr>& vpPt3D);
    protected:
        std::shared_ptr<TwoViewReconstruction> mpTVR;
    };
} // NAV24::OP

#endif //NAV24_OP_MAPINITIALIZATION_HPP
