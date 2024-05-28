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
        mqpImages(), mMtxImgQ(), mbInitialized(false) {}

    ObjTracking::ObjTracking(const ChannelPtr& pChannel) : MsgCallback(pChannel), Operator(),
        mqpImages(), mMtxImgQ(), mbInitialized(false) {}

    void ObjTracking::setup(const MsgPtr &configMsg) {

    }

    void ObjTracking::handleRequest(const MsgPtr &reqMsg) {

    }

    void ObjTracking::run() {

        while(true) {
            mMtxImgQ.lock();
            FramePtr pImage;
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

    void ObjTracking::init(const MsgPtr &msg) {

    }

    void ObjTracking::update(const FramePtr &pImage) {

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

    void ObjTracking::fetchFrameInfo(const FramePtr& pFrame, double &ts, cv::Mat &image, OB::ObsPtr &pObs) {

        if (!pFrame || !static_pointer_cast<FrameImgMono>(pFrame)) {
            return;
        }

        auto pImgFrame = static_pointer_cast<FrameImgMono>(pFrame);
        auto pImg = pImgFrame->getImage();
        image = pImg->mImage;
        pObs = pImgFrame->getObservations().back();

        if (!pImg || !static_pointer_cast<ImageTs>(pImg)) {
            return;
        }

        auto pImgTs = static_pointer_cast<ImageTs>(pImg);
        ts = pImgTs->mTimeStamp;
    }

    cv::Rect2f ObjTracking::fetchBbox(const OB::ObsPtr &pObs) const {

        if (pObs && static_pointer_cast<OB::BBox>(pObs)) {
            return static_pointer_cast<OB::BBox>(pObs)->getBbox();
        }
        auto pLastObs = mLastObs.second;
        if (pLastObs && static_pointer_cast<OB::BBox>(pLastObs)) {
            return static_pointer_cast<OB::BBox>(pLastObs)->getBbox();
        }
        return {};
    }

    void ObjTracking::updateLastObs(const double& ts, const cv::Rect2f &bbox) {
        auto pLastObs = make_shared<OB::BBox>();
        pLastObs->updateBboxAndLastPoint(bbox);
        mLastObs = make_pair(ts, pLastObs);
    }
} // NAV24::OP
