//
// Created by masoud on 5/2/24.
//

#ifndef NAV24_OP_OBJTRACKINGYOLO_HPP
#define NAV24_OP_OBJTRACKINGYOLO_HPP

#include <queue>
//#undef LIB_ONNX_RUNTIME_FOUND
#ifdef LIB_ONNX_RUNTIME_FOUND
#include <onnxruntime/onnxruntime_cxx_api.h>
#endif

#include "Image.hpp"
#include "OP_ObjTracking.hpp"
#include "OP_ObjDet.hpp"


namespace NAV24::OP {

    /// Based on ML object detection models (OpenCV and ONNX_RT)
    class ObjTrackingMl : public ObjTracking {
    public:
        inline static const std::string TOPIC = "OP::ObjTrYoloOnnx";

        explicit ObjTrackingMl(const ChannelPtr&  pChannel);
        void receive(const MsgPtr &msg) override;

        //static ParamPtr getDefParams(const std::string& model_base, const std::string& model_file);

    protected:
        void setup(const MsgPtr& msg) override;
        void handleRequest(const MsgPtr &reqMsg) override;
        //void run() override;
        //void stop() override;

        void update(const FramePtr& pImage) override;

    protected:
        std::shared_ptr<ObjDet> mpObjDetector;
        std::shared_ptr<SensorInterface> mpInterface;
    };
} // NAV24::OP

#endif //NAV24_OP_OBJTRACKINGYOLO_HPP
