//
// Created by masoud on 8/30/24.
//

#ifndef NAV24_OP_FTDT_HPP
#define NAV24_OP_FTDT_HPP

#include "Operator.hpp"
#include "Parameter.hpp"
#include "Frame.hpp"
#include "Message.hpp"

namespace NAV24::OP {
    class FtDt : public Operator {
    public:
        explicit FtDt(int nFt = 0) : mnFeatures(nFt) {}
        static std::shared_ptr<FtDt> create(const ParamPtr& pParam, ChannelPtr& pChannel);
        virtual int detect(FramePtr& pFrame) = 0;

    protected:
        int mnFeatures;
    };
    typedef std::shared_ptr<FtDt> FtDtPtr;

    class FtDtOCV : public FtDt {
    public:
        explicit FtDtOCV(int nFt = 0);

        int detect(FramePtr &pFrame) override;
    protected:
        cv::Ptr<cv::FeatureDetector> mpDetector;
    };
} // NAV24::OP

#endif //NAV24_OP_FTDT_HPP