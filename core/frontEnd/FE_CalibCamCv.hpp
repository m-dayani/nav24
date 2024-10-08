//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_FE_CALIBCAMCV_HPP
#define NAV24_FE_CALIBCAMCV_HPP

#include <memory>

#include "FrontEnd.hpp"
#include "Point3D.hpp"
#include "Frame.hpp"
#include "operators/objDetection/OP_ChBoardDetCv.hpp"
#include "Calibration.hpp"


namespace NAV24::FE {

#define FCN_FE_CAM_CALIB 23
#define FCN_SHOW_LAST_FRAME 201

    class CalibCamCv : public FrontEnd {
    public:
        inline static const std::string TOPIC = "FE::CalibCamCv";

        explicit CalibCamCv(const ChannelPtr& pChannel);
        void receive(const MsgPtr &msg) override;

        static ParamPtr getDefaultParameters(std::vector<ParamPtr>& vpParamContainer);

    protected:
        void setup(const MsgPtr &msg) override;
        void handleImageMsg(const MsgPtr &msg);
        void calibrate();

        void drawFrame(const FramePtr& pFrame);
        void drawChessBoard(const ImagePtr& pImage, const std::vector<OB::ObsPtr>& vpCorners, bool res);
        void drawPoseMap();

    protected:
        bool mbInitialized;
        cv::Size mGridSize;
        float mGridScale;
        cv::Size mImageSize;

        std::string mCalibMap;
        std::vector<WO::WoPtr> mvpPts3D;

        std::string mTrajectory;
        std::vector<FramePtr> mvpFrames;

        std::shared_ptr<OP::OP_ChBoardDetCv> mpOpChBoardDetCv;

        std::vector<ParamPtr> mvpParamHolder;

        CalibPtr mpCalib;
    };

} // NAV24::FE

#endif //NAV24_FE_CALIBCAMCV_HPP
