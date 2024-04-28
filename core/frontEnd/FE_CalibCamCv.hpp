//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_FE_CALIBCAMCV_HPP
#define NAV24_FE_CALIBCAMCV_HPP

#include "FrontEnd.hpp"

namespace NAV24::FE {

    class CalibCamCv : public FrontEnd {
    public:
        inline static const std::string TOPIC = "FE::CalibCamCv";

        explicit CalibCamCv(const ChannelPtr& pChannel);
        void receive(const MsgPtr &msg) override;

        static ParamPtr getDefaultParameters(std::vector<ParamPtr>& vpParamContainer);

    protected:
        void initialize() override;
        void handleImageMsg(const MsgPtr &msg);

    protected:
        bool mbInitialized;
        cv::Size mGridSize;
        float mGridScale;
    };

} // NAV24::FE

#endif //NAV24_FE_CALIBCAMCV_HPP
