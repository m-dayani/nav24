//
// Created by masoud on 2/22/25.
//

#ifndef NAV24_FE_INFERENCENAV_HPP
#define NAV24_FE_INFERENCENAV_HPP


#include <memory>
#include <thread>

#include "WorldObject.hpp"
#include "FrontEnd.hpp"
#include "Frame.hpp"
#include "OP_ObjDet.hpp"
#include "Calibration.hpp"


namespace NAV24::FE {

    class InferenceNav : public FrontEnd, public std::enable_shared_from_this<InferenceNav> {
    public:
        inline static const std::string TOPIC = "FE::InferenceNav";

        explicit InferenceNav(const ChannelPtr& pChannel);
        void receive(const MsgPtr &msg) override;

    protected:
        void initOperators();
        void setup(const MsgPtr &msg) override;
        virtual void handleImageMsg(const MsgPtr &msg);

        void stop() override;

        void showResults(const ImagePtr& pImg, const cv::Point2f& lastPoint, const WO::WoPtr &Pw);

        void createAndInsertFrame(const ImagePtr& pImg);
//        std::shared_ptr<FrameImgMono> creatNewFrame(const ImagePtr& pImg, const OB::ObsPtr& pObs);
        FramePtr getLastFrame();
        std::shared_ptr<OB::BBox> getLastObservation();

        OB::ObsPtr updateObservation();
        void correctObservation(const OB::ObsTimed& pObs);

    protected:
        bool mbInitialized;

        //std::string mTrType;

        std::string mMapName;
        std::vector<WO::WoPtr> mvpPts3D{};

        std::string mTrajectory;
        PosePtr mpTwc;
        Tf2dPtr mHwc;
        //std::vector<FramePtr> mvpFrames{};
        cv::Size mImgSize;

        ParamPtr mpTempParam;
        std::vector<ParamPtr> mvpParamHolder;

        std::vector<std::shared_ptr<OP::ObjDet>> mvpObjDetectors;
        std::vector<std::shared_ptr<std::thread>> mvpThTrackers;

        std::shared_ptr<FrameImgMono> mpLastFrame;
        std::map<long, FramePtr> mmpFrameBuffer;

        CalibPtr mpCalib;
    };
} // NAV24::FE


#endif //NAV24_FE_INFERENCENAV_HPP
