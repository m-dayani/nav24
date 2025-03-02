//
// Created by masoud on 2/22/25.
//

#ifndef NAV24_OP_OBJDET_HPP
#define NAV24_OP_OBJDET_HPP

#include <vector>

#include "Operator.hpp"
#include "Observation.hpp"
#include "Parameter.hpp"
#include "Message.hpp"
#include "Image.hpp"


namespace NAV24::OP {

#define OP_ODT_NAME_ML_ONNX "obj_det_ml_onnx"
#define OP_ODT_NAME_ML_ONNX_RT "obj_det_ml_onnx_rt"
#define OP_ODT_NAME_ML_CV "obj_det_ml_tf"
#define OP_ODT_NAME_APRILTAG "obj_det_apriltag"

    class ObjDet : public Operator {
    public:
        virtual void detect(const ImagePtr& pImage, std::vector<OB::ObsPtr>& vpObs) = 0;

        static std::shared_ptr<ObjDet> createDetector(const ParamPtr& pParam, const ChannelPtr& pChannel);

    };
}   // NAV24::OP


#endif //NAV24_OP_OBJDET_HPP
