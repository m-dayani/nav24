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

        explicit Camera(const ChannelPtr& pChannel);

        void receive(const MsgPtr &msg) override;

        static std::shared_ptr<Sensor> getCamera(const ParamPtr& pCamParams, const ChannelPtr& pChannel, const std::string& stdIdx);

        static WO::woPtr unproject(const OB::obsPtr& pObs, const TransPtr& pPose_wc, const CalibPtr& pCalib, float scale=1.f);
        static OB::obsPtr project(const WO::woPtr& pWo, const TransPtr& pPose_cw, const CalibPtr& pCalib, float scale=1.f);

    protected:
        void setup(const MsgPtr &msg) override;

        void handleRequest(const MsgPtr &reqMsg) override;

        [[nodiscard]] std::string printStr(const std::string &prefix) const override;

    protected:
        cv::Size mImgSz;
        float mFps;
        float mTs;
        CalibPtr mpCalib;

        static int camIdx;
    };

    /* ============================================================================================================== */

    class CamStream : public virtual Camera {
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
    };

    /* ============================================================================================================== */

    class CamOffline : public virtual Camera {
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
    };

    /* ============================================================================================================== */

    class CamMixed : public CamOffline, public CamStream {
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
    };

}   //NAV24

#endif //NAV24_CAMERA_HPP
