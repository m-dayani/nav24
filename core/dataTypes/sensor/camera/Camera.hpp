//
// Created by masoud on 2/6/24.
//

#ifndef NAV24_CAMERA_HPP
#define NAV24_CAMERA_HPP

#include <opencv2/highgui.hpp>

#include "Sensor.hpp"
#include "Calibration.hpp"
#include "TabularTextDS.hpp"


namespace NAV24 {



    class Camera : public Sensor {
    public:
        inline static const std::string TOPIC = "Camera";

        explicit Camera(const ChannelPtr& pChannel) : Sensor(pChannel) {}

        void receive(const MsgPtr &msg) override;

        // remove these test methods
        //void playTest() { this->play(); }
        //void getNextTest(const MsgPtr &msg) { this->getNext(msg); }
        //std::string toString() { return printStr(""); }

    protected:
        void loadParams(const MsgPtr &msg) override;
        [[nodiscard]] std::string printStr(const std::string &prefix) const override;

    protected:
        int imWidth{}, imHeight{};
        float fps{};
        CalibPtr mpCalib;
    };

    class CamStream : public Camera {
    public:
        explicit CamStream(const ChannelPtr& pChannel) : Camera(pChannel) {}
        ~CamStream();

    protected:
        void loadParams(const MsgPtr &msg) override;

        void reset() override;

    protected:
        void getNext(MsgPtr pReq) override;
        void play() override;

        std::shared_ptr<cv::VideoCapture> mpVideoCap;
    };

    class CamOffline : public Camera {
    public:
        explicit CamOffline(const ChannelPtr& pChannel) : Camera(pChannel), tsFactor(1.0) {}
        ~CamOffline();

        void receive(const MsgPtr &msg) override;

        static ParamPtr getFoldersParams(const std::string& seqBase, const std::string& imgBase,
                                         const std::string& imgFile, const double& tsFact,
                                         std::vector<ParamPtr>& vpParam);

    protected:
        void loadParams(const MsgPtr &msg) override;

        void getNextImageFile(std::string& path, double& ts);
        void getNext(MsgPtr pReq) override;
        void play() override;

        void reset() override;

        [[nodiscard]] std::string printStr(const std::string &prefix) const override;

    protected:
        std::string mSeqBase;
        std::string mImgBase;
        std::string mImgFile;
        double tsFactor;

        std::shared_ptr<TabularTextDS> mpImgDS;
    };

    class CamMixed : public CamStream, CamOffline {
    public:
        //CamMixed() = default;

    protected:
        void getNext(MsgPtr pReq) override {

        }

        void play() override {

        }
    };

}   //NAV24

#endif //NAV24_CAMERA_HPP
