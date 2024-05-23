//
// Created by masoud on 5/6/24.
//

#include <thread>
#include <glog/logging.h>

#include "Output.hpp"
#include "Image.hpp"
#include "OP_ObjTrackingCv.hpp"
#include "FrontEnd.hpp"
#include "ParameterBlueprint.h"

using namespace cv;
using namespace std;

namespace NAV24::OP {

// Convert to string
//#define SSTR( x ) static_cast< std::ostringstream & >( \
//( std::ostringstream() << std::dec << x ) ).str()
#define MAX_SIZE_BUFFER 1

    ObjTrackingCv::ObjTrackingCv(const ChannelPtr& pChannel) : ObjTracking(pChannel),
        mTrIdx(DEF_TR_CV_OPT), mbBboxInit(false), mbTrInit(false) {

        mvTrOptions = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW", "GOTURN", "CSRT"};
        this->initTrackerObj();
    }

    void ObjTrackingCv::setup(const MsgPtr &msg) {
        ObjTracking::setup(msg);

        // Initialize tracker object with message
        if (msg && dynamic_pointer_cast<MsgConfig>(msg)) {

            auto pMsgConf = dynamic_pointer_cast<MsgConfig>(msg);
            auto pParams = pMsgConf->getConfig();
            if (pParams) {
                auto pParamName = find_param<ParamType<string>>(PKEY_NAME, pParams);
                mName = (pParamName) ? pParamName->getValue() : "ObjTrackerCV";

                auto pParamOpt = find_param<ParamType<int>>("method", pParams);
                mTrIdx = (pParamOpt) ? pParamOpt->getValue() : DEF_TR_CV_OPT;

                this->initTrackerObj();
            }
        }
    }

    void ObjTrackingCv::handleRequest(const MsgPtr &msg) {
        ObjTracking::handleRequest(msg);

        if (msg && dynamic_pointer_cast<MsgRequest>(msg)) {

            auto pReqMsg = dynamic_pointer_cast<MsgRequest>(msg);
            auto sender = pReqMsg->getCallback();
            if (sender) {
                int action = msg->getTargetId();
                if (action == FCN_OBJ_TR_RUN) {
                    auto pThRun = make_shared<thread>(&ObjTrackingCv::run, this);
                    auto msgRes = make_shared<MsgType<shared_ptr<thread>>>(ID_CH_SYS, pThRun,
                                                                           msg->getTopic());
                    sender->receive(msgRes);
                }
            }
        }
    }

    void ObjTrackingCv::update(const ImagePtr& pImage) {

        if (!pImage || pImage->mImage.empty()) {
            DVLOG(2) << "ObjTrackingCv::update, empty image received\n";
            return;
        }

        cv::Mat frame = pImage->mImage.clone();

        // Start timer
        auto timer = (double)getTickCount();

        // Update the tracking result
        bool ok = mpTracker->update(frame, bbox);

        // Calculate Frames per second (FPS)
        auto fps = static_cast<float>(getTickFrequency() / ((double)getTickCount() - timer));


        if (ok) {
            // Tracking success : Draw the tracked object
            rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 );
        }
        else {
            // Tracking failure detected.
            putText(frame, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
        }

        // Display tracker type on frame
        putText(frame, mTrName + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);

        // Display FPS on frame
        putText(frame, "FPS : " + to_string(fps), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);

        // Update point track
        cv::Point2f trPoint = ObjTracking::find_center(bbox);
        auto msgPt = make_shared<MsgType<cv::Point2f>>(ID_TP_FE, trPoint,
                                                       FE::FrontEnd::TOPIC);
        mpChannel->publish(msgPt);

        // Display frame.
        auto pImage1 = make_shared<ImageTs>(frame.clone(), -1, "");
        auto msgDisp = make_shared<MsgSensorData>(ID_TP_OUTPUT, pImage1,
                                                  Output::TOPIC, DEF_ACTION, "Tracking");
        mpChannel->publish(msgDisp);
    }

    void ObjTrackingCv::receive(const MsgPtr &msg) {

        if (msg) {
            int action = msg->getTargetId();
            if (action == FCN_TR_CV_INIT_OPT && !msg->getMessage().empty()) {
                istringstream iss{msg->getMessage()};
                int opt = 0;
                iss >> opt;
                mTrIdx = opt;
                this->initTrackerObj();
            }
            if (dynamic_pointer_cast<MsgType<cv::Rect2f>>(msg)) {
                auto pBboxMsg = dynamic_pointer_cast<MsgType<cv::Rect2f>>(msg);
                bbox = pBboxMsg->getData();
                mbBboxInit = true;
            }
            if (dynamic_pointer_cast<MsgSensorData>(msg)) {

                auto psData = dynamic_pointer_cast<MsgSensorData>(msg);
                auto pData = psData->getData();
                if (pData && dynamic_pointer_cast<ImageTs>(pData)) {
                    auto pImage = dynamic_pointer_cast<ImageTs>(pData);
                    cv::Mat image = pImage->mImage.clone();

                    if (action == FCN_TR_CV_INIT_BB) {
                        bbox = selectROI(image, false);
                        mbBboxInit = true;
                    }
                    if (action == FCN_TR_CV_INIT_OBJ) {
                        mpTracker->init(image, bbox);
                        mbTrInit = true;
                    }
                    if (action == FCN_TR_CV_TRACK) {
                        //this->process(image);
                        mMtxImgQ.lock();
                        if (mqpImages.size() <= MAX_SIZE_BUFFER) {
                            mqpImages.push(pImage);
                        }
                        mMtxImgQ.unlock();
                    }
                }
            }
            if (dynamic_pointer_cast<MsgRequest>(msg)) {
                this->handleRequest(msg);
            }
            if (dynamic_pointer_cast<MsgConfig>(msg)) {
                this->setup(msg);
            }
        }
    }

    void ObjTrackingCv::initTrackerObj() {

        if (mTrIdx < 0 || mTrIdx >= mvTrOptions.size()) {
            mTrIdx = 2;
        }
        mTrName = mvTrOptions[mTrIdx];

#if (CV_MAJOR_VERSION < 3)
        {
            mpTracker = Tracker::create(mTrName);
        }
#else
        {
            if (mTrName == "BOOSTING")
                mpTracker = TrackerBoosting::create();
            if (mTrName == "MIL")
                mpTracker = TrackerMIL::create();
            if (mTrName == "KCF")
                mpTracker = TrackerKCF::create();
            if (mTrName == "TLD")
                mpTracker = TrackerTLD::create();
            if (mTrName == "MEDIANFLOW")
                mpTracker = TrackerMedianFlow::create();
            if (mTrName == "GOTURN")
                mpTracker = TrackerGOTURN::create();
            if (mTrName == "CSRT")
                mpTracker = TrackerCSRT::create();
        }
#endif

        DLOG(INFO) << "ObjTrackingCv::initTrackerObj, changed tracker to (" << mTrIdx << ", " << mTrName << ")\n";
    }

    /*void ObjTrackingCv::run() {
        ObjTracking::run();

        while(true) {
            cv::Mat img;
            mMtxImgQ.lock();
            if (!mqpImages.empty()) {
                auto pImage = mqpImages.front();
                if (pImage && !pImage->mImage.empty()) {
                    img = pImage->mImage.clone();
                }
                mqpImages.pop();
            }
            mMtxImgQ.unlock();

            if (!img.empty()) {
                this->process(img);
            }

            if (this->isStopped()) {
                break;
            }
        }
    }*/
} // NAV24::OP