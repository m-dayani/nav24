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
//#include "Point2D.hpp"

using namespace cv;
using namespace std;

namespace NAV24::OP {

// Convert to string
//#define SSTR( x ) static_cast< std::ostringstream & >( \
//( std::ostringstream() << std::dec << x ) ).str()
#define MAX_SIZE_BUFFER 1

    ObjTrackingCv::ObjTrackingCv(const ChannelPtr& pChannel) : ObjTracking(pChannel),
        mTrIdx(DEF_TR_CV_OPT), mbTrInit(false), mbManInit(false), mInitTs(-1.0) {

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

                auto pParamManInit = find_param<ParamType<int>>("manInit", pParams);
                mbManInit = (pParamManInit) ? pParamManInit->getValue() != 0 : false;

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

    void ObjTrackingCv::update(const FramePtr& pImage) {

        double ts = -1.0;
        cv::Mat image;
        OB::ObsPtr pObs;
        fetchFrameInfo(pImage, ts, image, pObs);
        cv::Rect2d bbox = fetchBbox(pObs);

        if (image.empty()) {
            DVLOG(2) << "ObjTrackingCv::update, empty image received\n";
            return;
        }

        // ignore frames before the init point
        if (ts < mInitTs) {
            return;
        }

        // Start timer
        auto timer = (double)getTickCount();

        // Update the tracking result
        bool ok = mpTracker->update(image, bbox);

        // Calculate Frames per second (FPS)
        auto fps = static_cast<float>(getTickFrequency() / ((double)getTickCount() - timer));


        if (ok) {
            // Tracking success :
            // Update point track
            this->updateLastObs(ts, bbox);
            auto msgPt = make_shared<MsgType<OB::ObsTimed>>(ID_TP_FE, mLastObs,
                                                            FE::FrontEnd::TOPIC);
            mpChannel->publish(msgPt);

            // Draw the tracked object
            rectangle(image, bbox, Scalar( 255, 0, 0 ), 2, 1 );
        }
        else {
            // Tracking failure detected.
            mbTrInit = false;
            putText(image, "Tracking failure detected", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);
        }

        // Display tracker type on frame
        putText(image, mTrName + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);

        // Display FPS on frame
        putText(image, "FPS : " + to_string(fps), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);

        // Display frame.
        auto pImage1 = make_shared<ImageTs>(image.clone(), ts, "");
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
            if (dynamic_pointer_cast<MsgType<FramePtr>>(msg)) {
                auto pImage = dynamic_pointer_cast<MsgType<FramePtr>>(msg)->getData();
                string msgStr = msg->getMessage();
                bool isInitMsg = !msgStr.empty() && msgStr == "init";
                if (!mbTrInit) {
                    // if tracker is not initialized, initialize it
                    this->init(msg);
                }
                else {
                    // else, add it for tracking
                    if (!isInitMsg) {
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

    void ObjTrackingCv::init(const MsgPtr &msg) {
        ObjTracking::init(msg);

        if (mbTrInit) {
            DLOG(INFO) << "OP::ObjTrackingCv::init, tracker already initialized\n";
            return;
        }
        if (!msg || !dynamic_pointer_cast<MsgType<FramePtr>>(msg)) {
            // wrong message
            DLOG(INFO) << "OP::ObjTrackingCv::init, wrong message type\n";
            return;
        }
        auto pFrame = dynamic_pointer_cast<MsgType<FramePtr>>(msg)->getData();
        if (!pFrame) {
            DLOG(INFO) << "OP::ObjTrackingCv::init, frame is null\n";
            return;
        }

        double ts = -1.0;
        cv::Mat image;
        OB::ObsPtr pObs;
        cv::Rect2f bbox;
        fetchFrameInfo(pFrame, ts, image, pObs);

        if (image.empty()) {
            DLOG(INFO) << "OP::ObjTrackingCv::init, image is empty, abort\n";
            return;
        }

        if (mbManInit) {
            bbox = selectROI(image, false);
            mpTracker->init(image, bbox);
            mbTrInit = true;
        }
        else {
            // wait for the master tracker to select a bbox
            if (pObs) {
                bbox = fetchBbox(pObs);
                DLOG(INFO) << "OP::ObjTrackingCv::init, received bbox: " << bbox << "\n";
                if (!bbox.empty()) {
                    mpTracker->init(image, bbox);
                    mbTrInit = true;
                }
            }
        }

        if (mbTrInit) {
            mInitTs = ts;
            // add bbox
            this->updateLastObs(ts, bbox);
            DLOG(INFO) << "OP::ObjTrackingCv::init, initialized tracker at " << ts << "\n";
        }
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