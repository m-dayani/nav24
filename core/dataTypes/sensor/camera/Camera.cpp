//
// Created by masoud on 2/6/24.
//

#include "Camera.hpp"

#include <boost/filesystem.hpp>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "DataStore.hpp"
#include "Image.hpp"
#include "FrontEnd.hpp"


using namespace std;

namespace NAV24 {

#define PARAM_KEY_IMG_SIZE "resolution"
#define PARAM_KEY_CAM_FPS "fps"
#define PARAM_KEY_CAM_CALIB "calib"

#define PARAM_KEY_IMG_PATHS "ImagePathParams"
#define PARAM_KEY_SEQ_BASE "seqBase"
#define PARAM_KEY_IMG_BASE "imageBase"
#define PARAM_KEY_IMG_FILE "imageFile"
#define PARAM_KEY_TS_FACT "tsFactor"

    void Camera::receive(const NAV24::MsgPtr &msg) {
        Sensor::receive(msg);

        if (msg) {
            if (msg->getTargetId() == FCN_CAM_GET_CALIB) {
                auto pMsgReq = dynamic_pointer_cast<MsgRequest>(msg);
                if (pMsgReq) {
                    auto sender = pMsgReq->getCallback();
                    if (sender) {
                        CalibPtr pCalib{};
                        if (dynamic_cast<CamOffline*>(this)) {
                            pCalib = CamOffline::Camera::mpCalib;
                        }
                        else if (dynamic_cast<CamStream*>(this)) {
                            pCalib = CamStream::Camera::mpCalib;
                        }
                        auto pMsgCalib = make_shared<MsgType<CalibPtr>>(TOPIC, pCalib);
                        sender->receive(pMsgCalib);
                    }
                }
            }
        }
    }

    void Camera::setup(const MsgPtr &msg) {
        Sensor::setup(msg);

        if (!msg || !dynamic_pointer_cast<MsgConfig>(msg)) {
            DLOG(WARNING) << "Camera::setup, bad config message, abort\n";
            return;
        }

        auto pConfig = dynamic_pointer_cast<MsgConfig>(msg);
        auto pParamCam = pConfig->getConfig();

        if (pParamCam) {
            // Image size
            auto pImgSize = find_param<ParamSeq<int>>(PARAM_KEY_IMG_SIZE, pParamCam);
            if (pImgSize) {
                vector<int> imgSize = pImgSize->getValue();
                if (imgSize.size() >= 2) {
                    imWidth = imgSize[0];
                    imHeight = imgSize[1];
                }
            }

            // Frame per seconds
            auto pFps = find_param<ParamType<double>>(PARAM_KEY_CAM_FPS, pParamCam);
            if (pFps) {
                this->fps = (float) pFps->getValue();
            }

            // Calib
            mpCalib = make_shared<Calibration>(pParamCam->read(PARAM_KEY_CAM_CALIB));
        }
    }

    string Camera::printStr(const string &prefix) const {

        ostringstream oss;

        oss << Sensor::printStr(prefix);
        oss << "Image Size: " << imWidth << " x " << imHeight << "\n";
        oss << "Fps: " << fps << "\n";
        if (mpCalib) {
            oss << mpCalib->printStr();
        }

        return oss.str();
    }

    /* -------------------------------------------------------------------------------------------------------------- */

    CamStream::~CamStream() {

        if (mpVideoCap && mpVideoCap->isOpened()) {
            mpVideoCap->release();
        }
    }

    void CamStream::getNext(NAV24::MsgPtr msg) {

        if (!msg) {
            DLOG(WARNING) << "CamStream::getNext, Null message detected, abort\n";
            return;
        }

        MsgReqPtr request = dynamic_pointer_cast<MsgRequest>(msg);
        if (!request) {
            DLOG(WARNING) << "CamStream::getNext, Null request detected\n";
            return;
        }

        MsgCbPtr sender = request->getCallback();
        if (!sender) {
            DLOG(WARNING) << "CamStream::getNext, Null sender detected\n";
            return;
        }

        if (!mpVideoCap) {
            DLOG(WARNING) << "CamStream::getNext, VideoCapture is not opened, abort\n";
            return;
        }

        cv::Mat image;
        mpVideoCap->read(image);
        auto ts_chrono = chrono::time_point_cast<chrono::nanoseconds>(chrono::system_clock::now());


        auto ts = ts_chrono.time_since_epoch().count();
        ImagePtr imgObj = make_shared<ImageTs>(image.clone(), ts, "");
        auto msgSensor = make_shared<MsgSensorData>(FE::FrontEnd::TOPIC, imgObj);
        msgSensor->setTargetId(FCN_SEN_GET_NEXT);

        sender->receive(msgSensor);
    }

    void CamStream::run() {

        if (mpChannel && mpVideoCap) {
            mbIsStopped = false;
            while (mpVideoCap->isOpened()) {

                cv::Mat image;
                bool res = mpVideoCap->read(image);
                auto ts_chrono = chrono::time_point_cast<chrono::nanoseconds>(chrono::system_clock::now());

                if (!res) {
                    break;
                }

                auto ts = ts_chrono.time_since_epoch().count();
                ImagePtr imgObj = make_shared<ImageTs>(image.clone(), ts, "");
                auto msg = make_shared<MsgSensorData>(FE::FrontEnd::TOPIC, imgObj);

                mpChannel->publish(msg);

                if (mbIsStopped) {
                    break;
                }
            }
        }
    }

    void CamStream::setup(const MsgPtr &msg) {
        Camera::setup(msg);

        if (mpInterface) {
            // Initialize OpenCV VideoCapture
            mpVideoCap = make_shared<cv::VideoCapture>(mpInterface->port);
        }
    }

    void CamStream::reset() {
        mbIsStopped = true;
    }

    /* -------------------------------------------------------------------------------------------------------------- */

    void CamOffline::receive(const MsgPtr &msg) {
        Camera::receive(msg);
    }

    void CamOffline::getNextImageFile(std::string& path, double& ts) {

        if (mpImgDS) {
            if (mImgFile.empty()) {
                path = mpImgDS->getNextFile();
                ts = -1.0;
            }
            else {
                vector<string> vData{};
                vData.reserve(2);
                mpImgDS->getNextData(vData);

                if (vData.size() >= 2 && !TabularTextDS::isComment(vData[0])) {

                    istringstream iss{vData[0]};
                    iss >> ts;
                    if (mImgBase.empty()) {
                        path = mSeqBase + '/' + vData[1];
                    }
                    else {
                        path = mSeqBase + '/' + mImgBase + '/' + vData[1];
                    }
                }
                else if (!vData.empty() && TabularTextDS::isComment(vData[0])) {
                    path = vData[0];
                }
            }
        }
    }

    void CamOffline::getNext(MsgPtr msg) {

        if (!msg) {
            DLOG(WARNING) << "CamOffline::getNext, Null message detected, abort\n";
            return;
        }

        MsgReqPtr request = dynamic_pointer_cast<MsgRequest>(msg);
        if (!request) {
            DLOG(WARNING) << "CamOffline::getNext, Null request detected\n";
            return;
        }

        MsgCbPtr sender = request->getCallback();
        if (!sender) {
            DLOG(WARNING) << "CamOffline::getNext, Null sender detected\n";
            return;
        }

        if (!mpImgDS) {
            DLOG(WARNING) << "CamOffline::getNext, ImageDS is not configured, abort\n";
            return;
        }

        double ts = -1.0;
        string nextFile{};
        this->getNextImageFile(nextFile, ts);
        while(TabularTextDS::isComment(nextFile)) {
            nextFile = string{};
            this->getNextImageFile(nextFile, ts);
        }

        cv::Mat image = cv::imread(nextFile, cv::IMREAD_UNCHANGED);
        ImagePtr imgObj = make_shared<ImageTs>(image.clone(), ts, nextFile);
        auto msgSensor = make_shared<MsgSensorData>(FE::FrontEnd::TOPIC, imgObj);
        msgSensor->setTargetId(FCN_SEN_GET_NEXT);

        sender->receive(msgSensor);
    }

    void CamOffline::run() {

        if (mpChannel && mpImgDS) {

            double ts = -1.0;
            string nextFile{};

            this->getNextImageFile(nextFile, ts);
            mbIsStopped = false;
            while(!nextFile.empty()) {

                if (TabularTextDS::isComment(nextFile)) {
                    nextFile = string{};
                    this->getNextImageFile(nextFile, ts);
                    continue;
                }

                cv::Mat image = cv::imread(nextFile, cv::IMREAD_UNCHANGED);
                ImagePtr imgObj = make_shared<ImageTs>(image.clone(), ts, nextFile);
                auto msgSensor = make_shared<MsgSensorData>(FE::FrontEnd::TOPIC, imgObj);

                mpChannel->publish(msgSensor);

                nextFile = string{};
                this->getNextImageFile(nextFile, ts);

                if (mbIsStopped) {
                    break;
                }
            }
        }
    }

    void CamOffline::setup(const MsgPtr &msg) {
        Camera::setup(msg);

        if (!msg || !dynamic_pointer_cast<MsgConfig>(msg)) {
            DLOG(WARNING) << "CamOffline::setup, bad config message, abort\n";
            return;
        }

        // We only need to load image path params here
        if (msg->getTopic() != DataStore::TOPIC) {
            DLOG(INFO) << "CamOffline::setup, No image paths message, abort\n";
            return;
        }

        auto pConfig = dynamic_pointer_cast<MsgConfig>(msg);
        auto pParamDS = pConfig->getConfig();
        if (pParamDS) {
            // Sequence base
            auto pSeqBase = find_param<ParamType<string>>(PARAM_KEY_SEQ_BASE, pParamDS);
            if (pSeqBase) {
                mSeqBase = pSeqBase->getValue();
            }
            // Image base
            auto pImgBase = find_param<ParamType<string>>(PARAM_KEY_IMG_BASE, pParamDS);
            if (pImgBase) {
                mImgBase = pImgBase->getValue();
            }
            // Image file
            auto pFileName = find_param<ParamType<string>>(PARAM_KEY_IMG_FILE, pParamDS);
            if (pFileName) {
                mImgFile = pFileName->getValue();
            }
            // Ts Factor
            auto pTsFactor = find_param<ParamType<double>>(PARAM_KEY_TS_FACT, pParamDS);
            if (pTsFactor) {
                tsFactor = pTsFactor->getValue();
            }

            // Create the image data store
            string imgExt = boost::filesystem::extension(boost::filesystem::path(mImgFile));
            if (imgExt.empty()) {
                imgExt = ".png";
            }

            if (mImgFile.empty()) {
                mpImgDS = make_shared<TabularTextDS>(mSeqBase + "/" + mImgBase, mImgFile, imgExt);
            }
            else {
                mpImgDS = make_shared<TabularTextDS>(mSeqBase, mImgFile, imgExt);
                mpImgDS->open();
            }
        }
    }

    string CamOffline::printStr(const string &prefix) const {

        ostringstream oss;

        oss << Camera::printStr(prefix);
        oss << prefix << "Sequence Base: " << mSeqBase << "\n";
        oss << prefix << "Image Base: " << mImgBase << "\n";
        oss << prefix << "File Name: " << mImgFile << "\n";
        oss << prefix << "Timestamp Factor: " << tsFactor << "\n";
        if (mpImgDS) {
            oss << mpImgDS->printStr(prefix);
        }

        return oss.str();
    }

    CamOffline::~CamOffline() {
        if (mpImgDS) {
            mpImgDS->close();
        }
    }

    void CamOffline::reset() {
        mbIsStopped = true;
        if (mpImgDS) {
            mpImgDS->reset();
        }
    }

    ParamPtr CamOffline::getFoldersParams(const string &seqBase, const string &imgBase, const string &imgFile,
                                          const double &tsFact, vector <ParamPtr> &vpParams) {

        ParamPtr pParam = make_shared<Parameter>(PARAM_KEY_IMG_PATHS, nullptr, Parameter::NodeType::MAP_NODE);

        auto pSeqBase = make_shared<ParamType<string>>(PARAM_KEY_SEQ_BASE, pParam, seqBase);
        auto pImgBase = make_shared<ParamType<string>>(PARAM_KEY_IMG_BASE, pParam, imgBase);
        auto pImgFile = make_shared<ParamType<string>>(PARAM_KEY_IMG_FILE, pParam, imgFile);
        auto pTsFactor = make_shared<ParamType<double>>(PARAM_KEY_TS_FACT, pParam, tsFact);

        pParam->insertChild(PARAM_KEY_SEQ_BASE, pSeqBase);
        pParam->insertChild(PARAM_KEY_IMG_BASE, pImgBase);
        pParam->insertChild(PARAM_KEY_IMG_FILE, pImgFile);
        pParam->insertChild(PARAM_KEY_TS_FACT, pTsFactor);

        vpParams.push_back(pImgBase);
        vpParams.push_back((pImgFile));
        vpParams.push_back(pSeqBase);
        vpParams.push_back(pTsFactor);
        vpParams.push_back(pParam);

        return pParam;
    }

    /* -------------------------------------------------------------------------------------------------------------- */

    CamMixed::CamMixed(const ChannelPtr &pChannel) : Camera(pChannel), CamOffline(pChannel), CamStream(pChannel),
        mCamOp{CamOperation::OFFLINE} {}

    void CamMixed::receive(const MsgPtr &msg) {
        CamOffline::receive(msg);
        //CamStream::receive(msg);

        if (!msg) {
            DLOG(WARNING) << "CamMixed::receive, Null message detected, abort\n";
            return;
        }

        if (msg->getTargetId() == FCN_SEN_CONFIG) {
            string camOp = msg->getMessage();
            if (camOp == TAG_SEN_MX_OFFLINE) {
                mCamOp = CamOperation::OFFLINE;
            }
            else if (camOp == TAG_SEN_MX_STREAM) {
                mCamOp = CamOperation::STREAM;
            }
            else if (camOp == TAG_SEN_MX_BOTH) {
                mCamOp = CamOperation::BOTH;
            }
            else {
                mCamOp = CamOperation::NONE;
            }
        }
    }

    void CamMixed::getNext(MsgPtr msg) {

        if (!msg) {
            DLOG(WARNING) << "CamMixed::getNext, Null message detected, abort\n";
            return;
        }

        switch (mCamOp) {
            case OFFLINE:
                CamOffline::getNext(msg);
                break;
            case STREAM:
                CamStream::getNext(msg);
                break;
            case BOTH:
                CamOffline::getNext(msg);
                CamStream::getNext(msg);
                break;
            case NONE:
            default:
                DLOG(WARNING) << "CamMixed::getNext, Action not supported.\n";
                break;
        }
    }

    void CamMixed::run() {

        switch (mCamOp) {
            case OFFLINE:
                CamOffline::run();
                break;
            case STREAM:
                CamStream::run();
                break;
            case BOTH:
                CamOffline::run();
                CamStream::run();
                break;
            case NONE:
            default:
                DLOG(WARNING) << "CamMixed::run, Action not supported.\n";
                break;
        }

    }

    void CamMixed::reset() {
        CamOffline::reset();
        CamStream::reset();
    }

    void CamMixed::setup(const MsgPtr &msg) {
        CamStream::setup(msg);
        CamOffline::setup(msg);
    }

    string CamMixed::printStr(const string &prefix) const {
        return CamStream::printStr(prefix) + CamOffline::printStr(prefix);
    }
}   //NAV24