//
// Created by masoud on 2/22/25.
//

#include "OP_ObjDet.hpp"
#include "OP_ObjDetApriltag.hpp"
#include "OP_ObjDetMl.hpp"
#include "OP_ObjDetOnnxRT.hpp"
#include "ParameterBlueprint.h"

using namespace std;

namespace NAV24::OP {

    std::shared_ptr<ObjDet>
    ObjDet::createDetector(const ChannelPtr &pChannel, const ParamPtr &pParam) {

        shared_ptr<ObjDet> pObjDet = nullptr;

        if (pParam && pChannel) {
            auto pParamName = find_param<ParamType<string>>(PKEY_NAME, pParam);
            string opName = (pParamName) ? pParamName->getValue() : "unknown";

            if (opName == OP_ODT_NAME_APRILTAG) {
                pObjDet = make_shared<ObjDetApriltag>(pChannel);
            }
            else if (opName == OP_ODT_NAME_ML_CV || opName == OP_ODT_NAME_ML_ONNX || opName == OP_ODT_NAME_ML_ONNX_RT) {

                if (opName == OP_ODT_NAME_ML_ONNX_RT) {
                    pObjDet = make_shared<ObjDetOnnxRT>(pChannel);
                }
                else {
                    pObjDet = make_shared<ObjDetMlCv>(pChannel);
                }
            }

            if (pObjDet) {
                pChannel->registerChannel(ID_CH_OP, pObjDet);
            }
        }
        return pObjDet;
    }
}