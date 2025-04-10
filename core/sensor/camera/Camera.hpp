//
// Created by masoud on 2/6/24.
//

#ifndef NAV24_CAMERA_HPP
#define NAV24_CAMERA_HPP

#include <opencv2/highgui.hpp>

#include "Sensor.hpp"
#include "Calibration.hpp"
#include "TabularTextDS.hpp"
#include "WorldObject.hpp"
#include "trajectory/pose/Pose.hpp"


namespace NAV24 {

#define DEF_IMG_WIDTH 640
#define DEF_IMG_HEIGHT 480
#define DEF_CAM_FPS 30.f
#define DEF_CAM_TS 0.33f

#define FCN_CAM_GET_CALIB 23
#define FCN_CAM_LOAD_VIDEO 31

    class Camera : public Sensor {
    public:
        inline static const std::string TOPIC = "Camera";

        enum CamOperation {
            NONE,
            OFFLINE,
            STREAM,
            BOTH
        };

        explicit Camera(const ChannelPtr& pChannel);

        void receive(const MsgPtr &msg) override;

        static std::shared_ptr<Sensor> getCamera(const ChannelPtr& pChannel, const ParamPtr& pCamParams);

        static WO::WoPtr unproject(const OB::ObsPtr& pObs, const TransPtr& pPose_wc, const CalibPtr& pCalib, float scale=1.f);
        static OB::ObsPtr project(const WO::WoPtr& pWo, const TransPtr& pPose_cw, const CalibPtr& pCalib, float scale=1.f);

    protected:
        void setup(const MsgPtr &msg) override;

        void handleRequest(const MsgPtr &reqMsg) override;

        [[nodiscard]] std::string printStr(const std::string &prefix) const override;

    protected:
        CamOperation mCamOp;

        cv::Size mImgSz;
        float mFps;
        float mTs;
        double tsFactor;

//        static int camIdx;

        CalibPtr mpCalib;
    };

    /* ============================================================================================================== */

    // All sensors must support storage operation (to record sensors)
    class CameraMono : public Camera {
    public:
        explicit CameraMono(const ChannelPtr& pChannel);
        ~CameraMono();

        void receive(const MsgPtr &msg) override;

        static ParamPtr getFoldersParams(const std::string& seqBase, const std::string& imgBase,
                                         const std::string& imgFile, const double& tsFact,
                                         std::vector<ParamPtr>& vpParam);

    protected:
        void setup(const MsgPtr &msg) override;
        void setupStream(const MsgPtr &msg);
        void setupOffline(const MsgPtr &msg);
        void initVideoCap(int port, const std::string& video = "");


        void getNext(MsgPtr pReq) override;
        void getNextBr(MsgPtr msg) override;
        MsgPtr getNextStream(const MsgPtr& msg);
        MsgPtr getNextOffline(const MsgPtr& msg);
        void getNextImageFile(std::string& path, double& ts);

        void run() override;
        void runStream();
        void runOffline();

        void reset() override;


        [[nodiscard]] std::string printStr(const std::string &prefix) const override;

    protected:
        std::string mVideoBase;
        std::string mVideoFile;
        std::shared_ptr<cv::VideoCapture> mpVideoCap;
        std::mutex mMtxCap;

        std::shared_ptr<TabularTextDS> mpImgDS;
    };

    /* ============================================================================================================== */

    /*class CamStream : public virtual Camera {
    public:
        explicit CamStream(const ChannelPtr& pChannel);
        ~CamStream();

        void receive(const MsgPtr &msg) override;

    protected:
        void setup(const MsgPtr &msg) override;
        void initVideoCap(int port, const std::string& video = "");

        void getNextBr(MsgPtr msg) override;

        void reset() override;

    protected:
        void getNext(MsgPtr pReq) override;
        void run() override;

        std::string mPathVideo;
        std::string mVideoFile;
        std::shared_ptr<cv::VideoCapture> mpVideoCap;
        std::mutex mMtxCap;
    };*/

    /* ============================================================================================================== */

    /*class CamOffline : public virtual Camera {
    public:
        explicit CamOffline(const ChannelPtr& pChannel);
        ~CamOffline();

        //void receive(const MsgPtr &msg) override;

        static ParamPtr getFoldersParams(const std::string& seqBase, const std::string& imgBase,
                                         const std::string& imgFile, const double& tsFact,
                                         std::vector<ParamPtr>& vpParam);

    protected:
        void setup(const MsgPtr &msg) override;

        void getNextBr(MsgPtr msg) override;

        void getNextImageFile(std::string& path, double& ts);
        void getNext(MsgPtr pReq) override;
        void run() override;

        void reset() override;

        [[nodiscard]] std::string printStr(const std::string &prefix) const override;

    protected:
        std::string mSeqBase;
        std::string mImgBase;
        std::string mImgFile;
        double tsFactor;

        std::shared_ptr<TabularTextDS> mpImgDS;
    };*/

    /* ============================================================================================================== */

    /*class CamMixed : public CamOffline, public CamStream {
    public:
        enum CamOperation {
            NONE,
            OFFLINE,
            STREAM,
            BOTH
        };

        explicit CamMixed(const ChannelPtr &pChannel);

        void receive(const MsgPtr &msg) override;

    protected:
        void setup(const MsgPtr &msg) override;

        void getNext(MsgPtr pReq) override;

        void run() override;

        void getNextBr(MsgPtr msg) override;

        void reset() override;

        [[nodiscard]] std::string printStr(const std::string &prefix) const override;

    private:
        CamOperation mCamOp;
    };*/

}   //NAV24

#endif //NAV24_CAMERA_HPP
