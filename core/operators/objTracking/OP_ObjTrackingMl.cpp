//
// Created by masoud on 5/2/24.
//

#include <memory>
#include <utility>
#include <regex>
#include <iostream>
#include <thread>
#include <boost/filesystem.hpp>
#include <glog/logging.h>
#include <opencv2/imgproc.hpp>

#include "FrontEnd.hpp"
#include "OP_ObjTrackingMl.hpp"
#include "ObsMl.hpp"

using namespace std;


namespace NAV24::OP {

//#define RET_OK nullptr
#define MAX_SIZE_BUFFER 1


    ObjTrackingMl::ObjTrackingMl(const ChannelPtr& pChannel) : ObjTracking(pChannel) {}

    void ObjTrackingMl::receive(const MsgPtr &msg) {

        if (!msg) {
            DLOG(WARNING) << "ObjTrYoloOnnx::receive, null message detected\n";
            return;
        }

        if (dynamic_pointer_cast<MsgType<FramePtr>>(msg)) {
            auto pFrame = dynamic_pointer_cast<MsgType<FramePtr>>(msg)->getData();
            if (pFrame) {
                mMtxImgQ.lock();
                if (mqpImages.size() <= MAX_SIZE_BUFFER) {
                    mqpImages.push(pFrame);
                }
                mMtxImgQ.unlock();
            }
        }

        if (dynamic_pointer_cast<MsgConfig>(msg)) {
            this->setup(msg);
        }

        if (dynamic_pointer_cast<MsgRequest>(msg)) {
            this->handleRequest(msg);
        }

        int action = msg->getTargetId();
        if (action == FCN_OBJ_TR_STOP) {
            this->stop();
        }
    }

    void ObjTrackingMl::handleRequest(const MsgPtr &msg) {
        ObjTracking::handleRequest(msg);

        if (msg && dynamic_pointer_cast<MsgRequest>(msg)) {

            auto pReqMsg = dynamic_pointer_cast<MsgRequest>(msg);
            auto senderCb = pReqMsg->getCallbackFun();
            if (senderCb) {
                int action = msg->getTargetId();
                if (action == FCN_OBJ_TR_RUN) {
                    auto pThRun = make_shared<thread>(&ObjTrackingMl::run, this);
                    auto msgRes = make_shared<MsgType<shared_ptr<thread>>>(ID_CH_SYS, pThRun,
                                                                           msg->getTopic());
                    senderCb(msgRes);
                }
            }
        }
    }
    
    void ObjTrackingMl::setup(const MsgPtr &msg) {

        auto configMsg = dynamic_pointer_cast<MsgConfig>(msg);
        if (configMsg) {
            auto pParam = configMsg->getConfig();
            if (!pParam) {
                DLOG(WARNING) << "ObjTrYoloOnnx::initialize, parameter is null\n";
                return;
            }

            mpObjDetector = ObjDet::createDetector(mpChannel, pParam);
        }
    }

    void ObjTrackingMl::update(const FramePtr &pImage) {

        double ts = -1.0;
        cv::Mat image;
        OB::ObsPtr pObs;
        fetchFrameInfo(pImage, ts, image, pObs);

        if (image.empty()) {
            DVLOG(2) << "ObjTrYoloOnnx::process, empty image detected\n";
            return;
        }

        assert(!image.empty() && image.channels() == 3);

        auto pImg = make_shared<ImageTs>(image, ts, "");
        vector<OB::ObsPtr> vpObs;
        mpObjDetector->detect(pImg, vpObs);

        if (vpObs.empty()) {
            DLOG(WARNING) << "ObjTrYoloOnnx::detect, detections is empty\n";
            return;
        }

//        display_image(image, detections[0]);

//        auto w = image.cols, h = image.rows;
        for (const auto &d : vpObs) {

//            cv::Rect2d detRect(d.x * w, d.y * h, d.w * w, d.h * h);
            if (dynamic_pointer_cast<OB::ObsMl>(d)) {
                auto pBbox = make_shared<OB::BBox>();
                pBbox->updateBboxAndLastPoint(dynamic_pointer_cast<OB::ObsMl>(d)->getBbox());
                auto pObsTimed = make_pair(ts, pBbox);
                auto pMsgPtObs = make_shared<MsgType<OB::ObsTimed>>(ID_TP_FE, pObsTimed,
                                                                    FE::FrontEnd::TOPIC);
                mpChannel->publish(pMsgPtObs);
            }
        }
    }

} // NAV24::OP