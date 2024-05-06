//
// Created by masoud on 4/29/24.
//

#ifndef NAV24_FE_OBJTRACKING_HPP
#define NAV24_FE_OBJTRACKING_HPP

#include <memory>

#include "WorldObject.hpp"
#include "FrontEnd.hpp"
#include "Frame.hpp"
#include "OP_ObjTrackingYolo.hpp"
#include "Calibration.hpp"


namespace NAV24::FE {

class ObjTracking : public FrontEnd, public std::enable_shared_from_this<ObjTracking> {
    public:
        inline static const std::string TOPIC = "FE::ObjTracking";

        explicit ObjTracking(const ChannelPtr& pChannel);
        void receive(const MsgPtr &msg) override;

        //static ParamPtr getDefaultParameters(std::vector<ParamPtr>& vpParamContainer);

    protected:
        void setup(const MsgPtr &msg) override;
        void handleImageMsg(const MsgPtr &msg);

    void stop() override;

protected:
        bool mbInitialized;

        std::string mMapName;
        std::vector<WO::woPtr> mvpPts3D{};

        std::string mTrajectory;
        PosePtr mpTwc;
        std::vector<FramePtr> mvpFrames{};

        std::vector<ParamPtr> mvpParamHolder;

        std::shared_ptr<OP::ObjTrYoloOnnx> mpObjTracker;
        std::shared_ptr<std::thread> mpThTracker;

        cv::Point2f mLastImPoint;
        std::mutex mMtxLastPt;

        CalibPtr mpCalib;
    };
} // NAV24::FE

#endif //NAV24_FE_OBJTRACKING_HPP
