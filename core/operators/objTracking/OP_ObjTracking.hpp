//
// Created by masoud on 5/1/24.
//

#ifndef NAV24_OP_OBJTRACKING_HPP
#define NAV24_OP_OBJTRACKING_HPP

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <queue>

#include "Message.hpp"
#include "Interface.hpp"
#include "Operator.hpp"
#include "Sensor.hpp"
#include "Image.hpp"
#include "Frame.hpp"


namespace NAV24::OP {

#define OP_OTR_NAME_YOLO_PY "obj_tr_yolo_py"
#define OP_OTR_NAME_YOLO_ONNX "obj_tr_yolo_onnx"
#define OP_OTR_NAME_CV "obj_tr_cv"

#define FCN_OBJ_TR_RUN 4
#define FCN_OBJ_TR_STOP 6

    class ObjTracking : public MsgCallback, public Operator {
    public:
        inline static const std::string TOPIC = "ObjTracking";

        ObjTracking();
        explicit ObjTracking(const ChannelPtr& pChannel);

        static std::shared_ptr<ObjTracking> createTracker(const ParamPtr& pParam, const ChannelPtr& pChannel);

    protected:
        void setup(const MsgPtr &configMsg) override;
        void handleRequest(const MsgPtr &reqMsg) override;
        void run() override;

        static void fetchFrameInfo(const FramePtr& pFrame, double& ts, cv::Mat& image, OB::ObsPtr& pObs);
        cv::Rect2f fetchBbox(const OB::ObsPtr& pObs) const;

        // Initialize internal tracker objects
        virtual void init(const MsgPtr& msg);
        virtual void update(const FramePtr& pImage);

        void updateLastObs(const double& ts, const cv::Rect2f& bbox);

    protected:
        std::queue<FramePtr> mqpImages;
        std::mutex mMtxImgQ;

        OB::ObsTimed mLastObs;

        bool mbInitialized;
    };
} // NAV24::OP

#endif //NAV24_OP_OBJTRACKING_HPP
