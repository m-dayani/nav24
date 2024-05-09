//
// Created by masoud on 5/6/24.
//


#include "Output.hpp"
#include "Image.hpp"
#include "OP_ObjTrackingCv.hpp"
#include "FrontEnd.hpp"

using namespace cv;
using namespace std;

namespace NAV24::OP {

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()


    ObjTrackingCv::ObjTrackingCv(const ChannelPtr& pChannel) : ObjTracking(pChannel), mTrIdx(DEF_TR_CV_OPT),
        bbox(287, 23, 86, 320), mbBboxInit(false), mbTrInit(false) {

        mvTrOptions = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "CSRT"};
        this->initTrackerObj();
    }

    void ObjTrackingCv::setup(const MsgPtr &msg) {
        ObjTracking::setup(msg);

        // Initialize tracker object with message
        if (msg && dynamic_pointer_cast<MsgConfig>(msg)) {

        }
    };

    void ObjTrackingCv::handleRequest(const MsgPtr &reqMsg) {
        ObjTracking::handleRequest(reqMsg);
    }

    void ObjTrackingCv::process(const Mat &frame) {

        // Start timer
        auto timer = (double)getTickCount();

        // Update the tracking result
        bool ok = mpTracker->update(frame, bbox);

        // Calculate Frames per second (FPS)
        float fps = getTickFrequency() / ((double)getTickCount() - timer);


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
        putText(frame, "FPS : " + SSTR(int(fps)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);

        // Update point track
        cv::Point2f trPoint = ObjTracking::find_center(bbox);
        auto msgPt = make_shared<MsgType<cv::Point2f>>(FE::FrontEnd::TOPIC, trPoint);
        mpChannel->publish(msgPt);

        // Display frame.
        auto pImage = make_shared<ImageTs>(frame.clone(), -1, "");
        auto msgDisp = make_shared<MsgSensorData>(Output::TOPIC, pImage);
        msgDisp->setMessage("Tracking");
        mpChannel->publish(msgDisp);
//        imshow("Tracking", frame);

        // Exit if ESC pressed.
//        int k = waitKey(1);
//        if(k == 27)
//        {
//            return;
//        }
    }

    void ObjTrackingCv::run() {
        ObjTracking::run();
    }

    void ObjTrackingCv::stop() {
        MsgCallback::stop();
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
                        this->process(image);
                    }
                }
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
    }
} // NAV24::OP