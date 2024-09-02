//
// Created by masoud on 9/1/24.
//

#ifndef NAV24_OP_FTASSOC_HPP
#define NAV24_OP_FTASSOC_HPP

#include <opencv2/features2d.hpp>

#include "Operator.hpp"
#include "Frame.hpp"
#include "MatchedFeatures.hpp"

namespace NAV24::OP {
    class FtAssoc : public Operator {
    public:
        FtAssoc() = default;

        virtual std::vector<int> match(const FramePtr& pFrame1, const FramePtr& pFrame2) = 0;
        virtual int match(const FramePtr& pFrame1, const FramePtr& pFrame2, OB::FtTracksPtr& pTracks) = 0;
    };

    class FtAssocOCV : public FtAssoc {
    public:
        FtAssocOCV();

        std::vector<int> match(const FramePtr &pFrame1, const FramePtr &pFrame2) override;

        int match(const FramePtr &pFrame1, const FramePtr &pFrame2, OB::FtTracksPtr &pTracks) override;

    protected:
        static cv::Mat getAllDescriptors(const FramePtr& pFrame);
        static std::vector<cv::KeyPoint> getAllKeyPoints(const FramePtr& pFrame);
    protected:
        cv::Ptr<cv::DescriptorMatcher> mpMatcher;
    };

} // NAV24::OP

#endif //NAV24_OP_FTASSOC_HPP
