//
// Created by masoud on 5/1/24.
//

//using namespace std;

#include "OP_ObjTracking.hpp"
#include "Message.hpp"
#include "ParameterBlueprint.h"
#include "ParameterServer.hpp"
#include "DataStore.hpp"
#include "OP_ObjTrackingYolo.hpp"
#include "OP_ObjTrackingYoloPy.hpp"
#include "OP_ObjTrackingCv.hpp"

using namespace std;

namespace NAV24::OP {

    ObjTracking::ObjTracking() : MsgCallback(), Operator(),
        mqpImages(), mMtxImgQ(), mMtxImg(), bbox(), mLastTs(-1.0), mbInitialized(false) {}

    ObjTracking::ObjTracking(const ChannelPtr& pChannel) : MsgCallback(pChannel), Operator(),
        mqpImages(), mMtxImgQ(), mMtxImg(), bbox(), mLastTs(-1.0), mbInitialized(false) {}

    void ObjTracking::setup(const MsgPtr &configMsg) {

    }

    void ObjTracking::handleRequest(const MsgPtr &reqMsg) {

    }

    void ObjTracking::run() {

        while(true) {
            mMtxImgQ.lock();
            ImagePtr pImage;
            if (!mqpImages.empty()) {
                pImage = mqpImages.front();
                mqpImages.pop();
            }
            mMtxImgQ.unlock();

            this->update(pImage);

            if (this->isStopped()) {
                break;
            }
        }
    }

    cv::Point2f ObjTracking::find_center(const cv::Rect2f &rect) {

        return {rect.x + rect.width * 0.5f, rect.y + rect.height * 0.5f};
    }

    void ObjTracking::init(const MsgPtr &msg) {

    }

    void ObjTracking::update(const ImagePtr &pImage) {

    }

    shared_ptr<ObjTracking> ObjTracking::createTracker(const ParamPtr& pParam, const ChannelPtr &pChannel) {

        shared_ptr<ObjTracking> pTracker = nullptr;

        if (pParam && pChannel) {

            auto pParamName = find_param<ParamType<string>>(PKEY_NAME, pParam);
            if (pParamName) {
                string opName = pParamName->getValue();

                if (opName == OP_OTR_NAME_YOLO_ONNX) {
                    pTracker = make_shared<ObjTrYoloOnnx>(pChannel);
                    // Retrieve required parameters and configure YOLO detector
                    MsgPtr msgYoloOpParams = make_shared<MsgRequest>(ID_CH_PARAMS, pTracker, ParameterServer::TOPIC,
                                                                     FCN_PS_REQ, string(PARAM_OP) + '/' +pParam->getName());
                    pChannel->send(msgYoloOpParams);
                    // todo: you normally want to address a dataset by the name in a component's interface
                    MsgPtr msgYoloModelPath = make_shared<MsgRequest>(ID_CH_DS, pTracker, DataStore::TOPIC,
                                                                      FCN_DS_REQ, TAG_DS_GET_PATH_MODEL);
                    pChannel->send(msgYoloModelPath);
                }
                else if (opName == OP_OTR_NAME_YOLO_PY) {
                    pTracker = make_shared<OP::ObjTrYoloPy>(pChannel);
                    MsgPtr msgYoloModelPath = make_shared<MsgRequest>(ID_CH_DS, pTracker, DataStore::TOPIC,
                                                                      FCN_DS_REQ, TAG_DS_GET_PATH_MODEL);
                    pChannel->send(msgYoloModelPath);
                    // Retrieve required parameters and configure YOLO detector
                    MsgPtr msgYoloOpParams = make_shared<MsgRequest>(ID_CH_PARAMS, pTracker, ParameterServer::TOPIC,
                                                                     FCN_PS_REQ, string(PARAM_OP) + '/' +pParam->getName());
                    pChannel->send(msgYoloOpParams);
                }
                else if (opName == OP_OTR_NAME_CV) {
                    pTracker = make_shared<ObjTrackingCv>(pChannel);
                    MsgPtr msgTrCvParams = make_shared<MsgRequest>(ID_CH_PARAMS, pTracker, ParameterServer::TOPIC,
                                                                   FCN_PS_REQ, string(PARAM_OP) + '/' + pParam->getName());
                    pChannel->send(msgTrCvParams);
                }
            }
        }

        return pTracker;
    }
} // NAV24::OP
