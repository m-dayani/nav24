//
// Created by masoud on 5/6/24.
//

#ifndef NAV24_OP_OBJTRACKINGCV_HPP
#define NAV24_OP_OBJTRACKINGCV_HPP

#include <queue>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#include "OP_ObjTracking.hpp"
#include "Message.hpp"


namespace NAV24::OP {

#define DEF_TR_CV_OPT 2

#define FCN_TR_CV_INIT_OBJ 28
#define FCN_TR_CV_INIT_OPT 29
#define FCN_TR_CV_INIT_BB 31
#define FCN_TR_CV_TRACK 23

    class ObjTrackingCv : public ObjTracking {
    public:
        inline static const std::string TOPIC = "OP::ObjTrackingCv";

        explicit ObjTrackingCv(const ChannelPtr& pChannel);
        //void init(const cv::Mat& image);

        void receive(const MsgPtr &msg) override;

    protected:
        void initTrackerObj();

        void setup(const MsgPtr &configMsg) override;
        void handleRequest(const MsgPtr &reqMsg) override;

        void update(const ImagePtr& pImage) override;
        //void run() override;

    private:
        std::size_t mTrIdx;
        std::string mTrName;
        std::vector<std::string> mvTrOptions;
        cv::Ptr<cv::Tracker> mpTracker;

        bool mbTrInit;
        bool mbBboxInit;
    };
} // NAV24::OP

#endif //NAV24_OP_OBJTRACKINGCV_HPP
