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
    ObjDet::createDetector(const ParamPtr &pParam, const ChannelPtr &pChannel) {

        shared_ptr<ObjDet> pObjDet = nullptr;

        if (pParam && pChannel) {
            auto pParamName = find_param<ParamType<string>>(PKEY_NAME, pParam);
            string opName = (pParamName) ? pParamName->getValue() : "unknown";
            if (opName == OP_ODT_NAME_APRILTAG) {
                auto pTagFamily = find_param<ParamType<string>>("family", pParam);
                string tagFamily = (pTagFamily) ? pTagFamily->getValue() : "unknown";
                pObjDet = make_shared<ObjDetApriltag>(tagFamily);
            }
            else if (opName == OP_ODT_NAME_ML_CV || opName == OP_ODT_NAME_ML_ONNX || opName == OP_ODT_NAME_ML_ONNX_RT) {

                auto pPathModel = find_param<ParamType<string>>("model", pParam);
                string pathModel = (pPathModel) ? pPathModel->getValue() : "unknown";

                auto pPathDesc = find_param<ParamType<string>>("config", pParam);
                string pathDesc = (pPathDesc) ? pPathDesc->getValue() : "unknown";

                auto pPathLabels = find_param<ParamType<string>>("labels", pParam);
                string pathLabels = (pPathLabels) ? pPathLabels->getValue() : "unknown";

                // Model info:
                ModelInfo modelInfo;

                auto pInputShape = find_param<ParamSeq<int>>("input_shape", pParam);
                vector<int> vInputShape = (pInputShape) ? pInputShape->getValue() : vector<int>();
                if (vInputShape.size() == 2) {
                    modelInfo.mInputShape = cv::Size(vInputShape[0], vInputShape[1]);
                }

                auto pInputScale = find_param<ParamType<double>>("input_scale", pParam);
                modelInfo.mInputScale = (pInputScale) ? pInputScale->getValue() : modelInfo.mInputScale;

                auto pInputMean = find_param<ParamType<double>>("input_mean", pParam);
                modelInfo.mInputMean = (pInputMean) ? pInputMean->getValue() : modelInfo.mInputMean;

                auto pThConf = find_param<ParamType<double>>("th_conf", pParam);
                modelInfo.mThConf = (pThConf) ? pThConf->getValue() : modelInfo.mThConf;

                auto pThScore = find_param<ParamType<double>>("th_score", pParam);
                modelInfo.mThScore = (pThScore) ? pThScore->getValue() : modelInfo.mThScore;

                auto pThNms = find_param<ParamType<double>>("th_nms", pParam);
                modelInfo.mThNms = (pThNms) ? pThNms->getValue() : modelInfo.mThNms;

                if (opName == OP_ODT_NAME_ML_ONNX_RT) {
                    pObjDet = make_shared<ObjDetOnnxRT>(pathModel, pathLabels, modelInfo);
                }
                else {
                    pObjDet = make_shared<ObjDetMlCv>(pathModel, pathDesc, pathLabels, modelInfo);
                }
            }
        }
        return pObjDet;
    }
}