//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_FE_CALIBCAMCV_HPP
#define NAV24_FE_CALIBCAMCV_HPP

#include <memory>

#include "FrontEnd.hpp"
#include "Point3D.hpp"
#include "Frame.hpp"
#include "OP_ChBoardDetCv.hpp"


namespace NAV24::FE {

#define FCN_FE_CAM_CALIB 23

    class CalibCamCv : public FrontEnd {
    public:
        inline static const std::string TOPIC = "FE::CalibCamCv";

        explicit CalibCamCv(const ChannelPtr& pChannel);
        void receive(const MsgPtr &msg) override;

        static ParamPtr getDefaultParameters(std::vector<ParamPtr>& vpParamContainer);

    protected:
        void initialize() override;
        void handleImageMsg(const MsgPtr &msg);
        void calibrate();

    protected:
        bool mbInitialized;
        cv::Size mGridSize;
        float mGridScale;
        cv::Size mImageSize;

        std::string mCalibMap;
        std::vector<WO::woPtr> mvpPts3D;

        std::string mTrajectory;
        std::vector<FramePtr> mvpFrames;

        std::shared_ptr<OP::OP_ChBoardDetCv> mpOpChBoardDetCv;

        std::vector<ParamPtr> mvpParamHolder;
    };

} // NAV24::FE

#endif //NAV24_FE_CALIBCAMCV_HPP
