//
// Created by masoud on 2/6/24.
//

#include <thread>
#include <chrono>
#include <boost/filesystem.hpp>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "Camera.hpp"
#include "DataStore.hpp"
#include "Image.hpp"
#include "FrontEnd.hpp"
#include "ParameterBlueprint.h"
#include "Point2D.hpp"
#include "Point3D.hpp"


using namespace std;

namespace NAV24 {

//#define DEF_CAM_NAME "cam"

//    int Camera::camIdx = 0;

    Camera::Camera(const ChannelPtr& pChannel) : Sensor(pChannel), mCamOp(OFFLINE),
                                                 mImgSz(DEF_IMG_WIDTH, DEF_IMG_HEIGHT),
                                                 mFps(DEF_CAM_FPS), mTs(DEF_CAM_TS), tsFactor(1.0),
                                                 mpCalib() {
        DLOG(INFO) << "Camera::Camera, Constructor\n";
    }

    void Camera::receive(const NAV24::MsgPtr &msg) {
        Sensor::receive(msg);

        if (msg) {
            if (dynamic_pointer_cast<MsgRequest>(msg)) {
                this->handleRequest(msg);
            }
        }
    }

    /* -------------------------------------------------------------------------------------------------------------- */

    void Camera::setup(const MsgPtr &msg) {
        Sensor::setup(msg);

        if (!msg || !dynamic_pointer_cast<MsgConfig>(msg)) {
            DLOG(WARNING) << "Camera::setup, bad config message, abort\n";
            return;
        }

        auto pConfig = dynamic_pointer_cast<MsgConfig>(msg);
        if (!pConfig) {
            DLOG(WARNING) << "Camera::setup, bad config message, abort\n";
            return;
        }

        auto pParamCam = pConfig->getConfig();
        if (pParamCam && msg->getTopic() != DataStore::TOPIC) {
            // The parent (Sensor) class loads the interface, so you don't have to do it again

            // Image size
            auto pImgSize = find_param<ParamSeq<int>>(PKEY_IMG_SIZE, pParamCam);
            if (pImgSize) {
                vector<int> imgSize = pImgSize->getValue();
                if (imgSize.size() >= 2) {
                    mImgSz = cv::Size(imgSize[0], imgSize[1]);
                }
            }

            // Frame per seconds
            auto pFps = find_param<ParamType<double>>(PKEY_CAM_FPS, pParamCam);
            if (pFps) {
                mFps = (float) pFps->getValue();
                if (mFps > 0) {
                    mTs = 1.f / mFps;
                }
                else {
                    mFps = DEF_CAM_FPS;
                }
            }

            // Calib
            mpCalib = make_shared<Calibration>(pParamCam->read(PKEY_CAM_CALIB));

            // Load important paths
            auto fp = [this](auto && PH1) {
                receive(std::forward<decltype(PH1)>(PH1));
            };
            MsgPtr msgConfPaths = make_shared<MsgRequest>(ID_CH_DS, fp, DataStore::TOPIC,
                                                          FCN_DS_REQ, TAG_DS_GET_PATH_IMG);

            auto interfaceType = mpInterface->interfaceType;
            if (interfaceType == SensorInterface::STREAM || interfaceType == SensorInterface::MIXED) {
                msgConfPaths->setMessage(TAG_DS_GET_PATH_VIDEO);
            }
            mpChannel->send(msgConfPaths);
        }
    }

    void Camera::handleRequest(const MsgPtr &msg) {

        if (msg) {
            int action = msg->getTargetId();
            if (action == FCN_CAM_GET_CALIB) {
                auto pMsgReq = dynamic_pointer_cast<MsgRequest>(msg);
                if (pMsgReq) {
                    auto senderCb = pMsgReq->getCallbackFun();
                    if (senderCb) {
                        auto pMsgCalib = make_shared<MsgType<CalibPtr>>(DEF_CAT, mpCalib);
                        senderCb(pMsgCalib);
                    }
                }
            }
            if (action == FCN_SEN_GET_NEXT) {
                this->getNext(msg);
            }
        }
    }

    string Camera::printStr(const string &prefix) const {

        ostringstream oss;

        oss << Sensor::printStr(prefix);
        oss << "Image Size: " << mImgSz.width << " x " << mImgSz.height << "\n";
        oss << "Fps: " << mFps << "\n";
        if (mpCalib) {
            oss << mpCalib->printStr("");
        }

        return oss.str();
    }

    std::shared_ptr<Sensor> Camera::getCamera(const ChannelPtr& pChannel, const ParamPtr &pCamParams) {

        auto pCamera = make_shared<CameraMono>(pChannel);
        auto msgConfig = make_shared<MsgConfig>(ID_CH_PARAMS, pCamParams);
        pCamera->receive(msgConfig);

        return pCamera;
    }

    WO::WoPtr Camera::unproject(const OB::ObsPtr &pObs, const TransPtr &pPose_wc, const NAV24::CalibPtr &pCalib, const float) {

        auto pc = pCalib->undistort(pObs);
        auto Pc = dynamic_pointer_cast<OB::Point2D>(pc);
        auto Pc_cv = Pc->getPointUd();
        auto pWo = make_shared<WO::Point3D>(Pc_cv.x, Pc_cv.y, 1.0);
        return pPose_wc->transform(pWo);
        //auto Pw_cv = static_pointer_cast<WO::Point3D>(Pw)->getPoint();
        //return make_shared<WO::Point3D>(Pw_cv.x / Pw_cv.z, Pw_cv.y / Pw_cv.z, 1.0);
    }

    OB::ObsPtr Camera::project(const WO::WoPtr &pWo, const TransPtr &pPose_cw, const NAV24::CalibPtr &pCalib, const float) {

        auto Pc = pPose_cw->transform(pWo);
        return pCalib->project(Pc);
    }

    /* ============================================================================================================== */

    /*CamStream::CamStream(const ChannelPtr& pChannel) : Camera(pChannel), mMtxCap() {
        DLOG(INFO) << "CamStream::CamStream\n";
    }
    CamStream::~CamStream() {

        if (mpVideoCap && mpVideoCap->isOpened()) {
            mpVideoCap->release();
        }
    }

    void CamStream::getNext(NAV24::MsgPtr msg) {

        if (!mpVideoCap) {
            DLOG(WARNING) << "CamStream::getNext, VideoCapture is not opened, abort\n";
            return;
        }

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

        cv::Mat image;
        mpVideoCap->read(image);

        auto ts_chrono = chrono::time_point_cast<chrono::nanoseconds>(chrono::system_clock::now());
        auto ts = ts_chrono.time_since_epoch().count();

        ImagePtr imgObj = make_shared<ImageTs>(image.clone(), ts, "");
        auto msgSensor = make_shared<MsgSensorData>(DEF_CAT, imgObj,
                                                    DEF_TOPIC, FCN_SEN_GET_NEXT);

        sender->receive(msgSensor);
    }

    void CamStream::run() {

        if (mpChannel && mpVideoCap) {

            auto Ts = std::chrono::milliseconds(static_cast<int>(mTs * 1000.f));

            while (mpVideoCap->isOpened()) {

                auto t1 = chrono::high_resolution_clock::now();

                cv::Mat image;
                mMtxCap.lock();
                bool res = mpVideoCap->read(image);
                mMtxCap.unlock();
                auto ts_chrono = chrono::time_point_cast<chrono::nanoseconds>(chrono::system_clock::now());

                if (!res) {
                    break;
                }

                auto ts = ts_chrono.time_since_epoch().count();
                ImagePtr imgObj = make_shared<ImageTs>(image.clone(), ts, "");
                auto msg = make_shared<MsgSensorData>(ID_TP_SDATA, imgObj);

                auto t2 = chrono::high_resolution_clock::now();

                mpChannel->publish(msg);

                auto duration = duration_cast<chrono::milliseconds>(t2 - t1);
                if (Ts > duration) {
                    std::this_thread::sleep_for(Ts - duration);
                }

                if (this->isStopped()) {
                    break;
                }
            }
        }
    }

    void CamStream::setup(const MsgPtr &msg) {
        Camera::setup(msg);

        if (msg && dynamic_pointer_cast<MsgConfig>(msg)) {
            auto pMsgConf = dynamic_pointer_cast<MsgConfig>(msg);
            auto pParamVideoPath = pMsgConf->getConfig();
            if (pParamVideoPath && dynamic_pointer_cast<ParamType<string>>(pParamVideoPath)) {
                mPathVideo = dynamic_pointer_cast<ParamType<string>>(pParamVideoPath)->getValue();
            }
        }

        if (mpInterface) {
            int port = mpInterface->port;
            if (port >= 0) {
                this->initVideoCap(port);
            }
            else if (!mPathVideo.empty() && !mVideoFile.empty()) {
                this->initVideoCap(port, mPathVideo + "/" + mVideoFile);
            }
        }
    }

    void CamStream::initVideoCap(const int port, const std::string& video) {

        // Initialize OpenCV VideoCapture
        if (port >= 0) {
            mpVideoCap = make_shared<cv::VideoCapture>(port);
        }
        else if (!video.empty()) {
            mpVideoCap = make_shared<cv::VideoCapture>(video);
        }
        if (mpVideoCap) {
            // set image size
            mpVideoCap->set(cv::CAP_PROP_FRAME_HEIGHT, mImgSz.height);
            mpVideoCap->set(cv::CAP_PROP_FRAME_WIDTH, mImgSz.width);
            // disable auto-focus
            mpVideoCap->set(cv::CAP_PROP_AUTOFOCUS, 0);
        }
    }

    void CamStream::reset() {
        this->stop();
    }

    void CamStream::receive(const MsgPtr &msg) {
        Camera::receive(msg);

        if (msg) {
            if (msg->getTargetId() == FCN_CAM_LOAD_VIDEO) {
                mVideoFile = msg->getMessage();
                this->setup(msg);
            }
        }
    }

    void CamStream::getNextBr(MsgPtr msg) {
        if (!mpVideoCap || !mpChannel) {
            DLOG(WARNING) << "CamStream::getNextBr, VideoCapture is not opened or no channel, abort\n";
            return;
        }

        if (!msg) {
            DLOG(WARNING) << "CamStream::getNextBr, Null message detected, abort\n";
            return;
        }

        cv::Mat image;
        mpVideoCap->read(image);

        auto ts_chrono = chrono::time_point_cast<chrono::nanoseconds>(chrono::system_clock::now());
        auto ts = ts_chrono.time_since_epoch().count();

        ImagePtr imgObj = make_shared<ImageTs>(image.clone(), ts, "");
        auto msgSensor = make_shared<MsgSensorData>(ID_TP_SDATA, imgObj);
        mpChannel->publish(msgSensor);
    }*/

    /* ============================================================================================================== */

    /*CamOffline::CamOffline(const ChannelPtr& pChannel) : Camera(pChannel), tsFactor(1.0) {
        DLOG(INFO) << "CamOffline::CamOffline\n";
    }

//    void CamOffline::receive(const MsgPtr &msg) {
//        Camera::receive(msg);
//    }

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
        auto msgSensor = make_shared<MsgSensorData>(DEF_CAT, imgObj,
                                                    DEF_TOPIC, FCN_SEN_GET_NEXT);
        sender->receive(msgSensor);
    }

    void CamOffline::run() {

        if (mpChannel && mpImgDS) {

            double ts = -1.0;
            string nextFile{};
            auto Ts = std::chrono::milliseconds(static_cast<int>(mTs * 1000.f));

            this->getNextImageFile(nextFile, ts);
            while(!nextFile.empty()) {

                auto t1 = chrono::high_resolution_clock::now();

                if (TabularTextDS::isComment(nextFile)) {
                    nextFile = string{};
                    this->getNextImageFile(nextFile, ts);
                    continue;
                }

                cv::Mat image = cv::imread(nextFile, cv::IMREAD_UNCHANGED);
                ImagePtr imgObj = make_shared<ImageTs>(image.clone(), ts, nextFile);
                auto msgSensor = make_shared<MsgSensorData>(ID_TP_SDATA, imgObj);

                nextFile = string{};
                this->getNextImageFile(nextFile, ts);

                auto t2 = chrono::high_resolution_clock::now();

                mpChannel->publish(msgSensor);

                auto duration = duration_cast<chrono::milliseconds>(t2 - t1);
//                if (Ts > duration) {
//                    std::this_thread::sleep_for(Ts - duration);
//                }

                if (this->isStopped()) {
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
            auto pSeqBase = find_param<ParamType<string>>(PKEY_SEQ_BASE, pParamDS);
            if (pSeqBase) {
                mSeqBase = pSeqBase->getValue();
            }
            // Image base
            auto pImgBase = find_param<ParamType<string>>(PKEY_IMG_BASE, pParamDS);
            if (pImgBase) {
                mImgBase = pImgBase->getValue();
            }
            // Image file
            auto pFileName = find_param<ParamType<string>>(PKEY_IMG_FILE, pParamDS);
            if (pFileName) {
                mImgFile = pFileName->getValue();
            }
            // Ts Factor
            auto pTsFactor = find_param<ParamType<double>>(PKEY_TS_FACT, pParamDS);
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
        this->stop();
        if (mpImgDS) {
            mpImgDS->reset();
        }
    }

    ParamPtr CamOffline::getFoldersParams(const string &seqBase, const string &imgBase, const string &imgFile,
                                          const double &tsFact, vector <ParamPtr> &vpParams) {

        ParamPtr pParam = make_shared<Parameter>(PKEY_IMG_PATHS, nullptr, Parameter::NodeType::MAP_NODE);

        auto pSeqBase = make_shared<ParamType<string>>(PKEY_SEQ_BASE, pParam, seqBase);
        auto pImgBase = make_shared<ParamType<string>>(PKEY_IMG_BASE, pParam, imgBase);
        auto pImgFile = make_shared<ParamType<string>>(PKEY_IMG_FILE, pParam, imgFile);
        auto pTsFactor = make_shared<ParamType<double>>(PKEY_TS_FACT, pParam, tsFact);

        pParam->insertChild(PKEY_SEQ_BASE, pSeqBase);
        pParam->insertChild(PKEY_IMG_BASE, pImgBase);
        pParam->insertChild(PKEY_IMG_FILE, pImgFile);
        pParam->insertChild(PKEY_TS_FACT, pTsFactor);

        vpParams.push_back(pImgBase);
        vpParams.push_back((pImgFile));
        vpParams.push_back(pSeqBase);
        vpParams.push_back(pTsFactor);
        vpParams.push_back(pParam);

        return pParam;
    }

    void CamOffline::getNextBr(MsgPtr msg) {
        if (!msg || !mpChannel) {
            DLOG(WARNING) << "CamOffline::getNextBr, Null message detected, abort\n";
            return;
        }

        if (!mpImgDS) {
            DLOG(WARNING) << "CamOffline::getNextBr, ImageDS is not configured, abort\n";
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
        auto msgSensor = make_shared<MsgSensorData>(ID_TP_SDATA, imgObj);
        mpChannel->publish(msgSensor);
    }*/

    /* ============================================================================================================== */

    /*CamMixed::CamMixed(const ChannelPtr &pChannel) : Camera(pChannel), CamOffline(pChannel), CamStream(pChannel),
        mCamOp(CamOperation::OFFLINE) {
        DLOG(INFO) << "CamMixed::CamMixed\n";
    }

    void CamMixed::receive(const MsgPtr &msg) {
        CamOffline::receive(msg);
        CamStream::receive(msg);

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

    void CamMixed::getNextBr(MsgPtr msg) {
        if (!msg) {
            DLOG(WARNING) << "CamMixed::getNextBr, Null message detected, abort\n";
            return;
        }

        switch (mCamOp) {
            case OFFLINE:
                CamOffline::getNextBr(msg);
                break;
            case STREAM:
                CamStream::getNextBr(msg);
                break;
            case BOTH:
                CamOffline::getNextBr(msg);
                CamStream::getNextBr(msg);
                break;
            case NONE:
            default:
                DLOG(WARNING) << "CamMixed::getNextBr, Action not supported.\n";
                break;
        }
    }*/

    /* ============================================================================================================== */

    CameraMono::CameraMono(const ChannelPtr &pChannel) : Camera(pChannel), mMtxCap() {
//        DLOG(INFO) << "CamStream::CamStream\n";
    }

    CameraMono::~CameraMono() {
        if (mpVideoCap && mpVideoCap->isOpened()) {
            mpVideoCap->release();
            mpVideoCap = nullptr;
        }
    }

    void CameraMono::receive(const MsgPtr &msg) {
        Camera::receive(msg);

        if (!msg) {
            DLOG(WARNING) << "CameraMono::receive, Null message detected, abort\n";
            return;
        }

        if (msg->getTargetId() == FCN_CAM_LOAD_VIDEO) {
            // sensor file here is the video file and sensor type is stream
//            mSensorFile = msg->getMessage();
//            mCamOp = CamOperation::STREAM;
            mVideoFile = msg->getMessage();
            this->setup(msg);
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

    /* -------------------------------------------------------------------------------------------------------------- */

    ParamPtr CameraMono::getFoldersParams(const string &seqBase, const string &imgBase, const string &imgFile,
                                          const double &tsFact, vector <ParamPtr> &vpParams) {

        ParamPtr pParam = make_shared<Parameter>(PKEY_IMG_PATHS, nullptr, Parameter::NodeType::MAP_NODE);

        auto pSeqBase = make_shared<ParamType<string>>(PKEY_SEQ_BASE, pParam, seqBase);
        auto pImgBase = make_shared<ParamType<string>>(PKEY_IMG_BASE, pParam, imgBase);
        auto pImgFile = make_shared<ParamType<string>>(PKEY_IMG_FILE, pParam, imgFile);
        auto pTsFactor = make_shared<ParamType<double>>(PKEY_TS_FACT, pParam, tsFact);

        pParam->insertChild(PKEY_SEQ_BASE, pSeqBase);
        pParam->insertChild(PKEY_IMG_BASE, pImgBase);
        pParam->insertChild(PKEY_IMG_FILE, pImgFile);
        pParam->insertChild(PKEY_TS_FACT, pTsFactor);

        vpParams.push_back(pImgBase);
        vpParams.push_back((pImgFile));
        vpParams.push_back(pSeqBase);
        vpParams.push_back(pTsFactor);
        vpParams.push_back(pParam);

        return pParam;
    }

    void CameraMono::setup(const MsgPtr &msg) {
        Camera::setup(msg);
        this->setupStream(msg);
        this->setupOffline(msg);
    }

    void CameraMono::initVideoCap(int port, const string &video) {

        // Initialize OpenCV VideoCapture
        if (port >= 0) {
            mpVideoCap = make_shared<cv::VideoCapture>(port);
        }
        else if (!video.empty()) {
            mpVideoCap = make_shared<cv::VideoCapture>(video);
        }
        if (mpVideoCap) {
            // set image size
            mpVideoCap->set(cv::CAP_PROP_FRAME_HEIGHT, mImgSz.height);
            mpVideoCap->set(cv::CAP_PROP_FRAME_WIDTH, mImgSz.width);
            // disable auto-focus
            mpVideoCap->set(cv::CAP_PROP_AUTOFOCUS, 0);
        }
    }

    void CameraMono::getNext(MsgPtr msg) {

        if (!msg) {
            DLOG(WARNING) << "CameraMono::getNext, Null message detected, abort\n";
            return;
        }

        MsgReqPtr request = dynamic_pointer_cast<MsgRequest>(msg);
        if (!request) {
            DLOG(WARNING) << "CameraMono::getNext, Null request detected\n";
            return;
        }

        auto senderCb = request->getCallbackFun();
        if (!senderCb) {
            DLOG(WARNING) << "CameraMono::getNext, Null sender detected\n";
            return;
        }

        MsgPtr pMsg = nullptr, pMsg2 = nullptr;
        switch (mCamOp) {
            case OFFLINE:
                pMsg = this->getNextOffline(msg);
                break;
            case STREAM:
                pMsg = this->getNextStream(msg);
                break;
            case BOTH:
                pMsg = this->getNextOffline(msg);
                pMsg2 = this->getNextStream(msg);
                break;
            case NONE:
            default:
                DLOG(WARNING) << "CamMixed::getNext, Action not supported.\n";
                break;
        }

        senderCb(pMsg);
        if (pMsg2) {
            senderCb(pMsg2);
        }
    }

    void CameraMono::getNextBr(MsgPtr msg) {

        if (!msg || !mpChannel) {
            DLOG(WARNING) << "CameraMono::getNextBr, Null channel or message detected, abort\n";
            return;
        }

        MsgPtr pMsg = nullptr, pMsg2 = nullptr;
        switch (mCamOp) {
            case OFFLINE: {
                pMsg = this->getNextOffline(msg);
                break;
            }
            case STREAM: {
                pMsg = this->getNextStream(msg);
                break;
            }
            case BOTH: {
                pMsg = this->getNextOffline(msg);
                pMsg2 = this->getNextStream(msg);
                break;
            }
            case NONE:
            default:
                DLOG(WARNING) << "CamMixed::getNextBr, Action not supported.\n";
                break;
        }

        mpChannel->publish(pMsg);
        if (pMsg2) {
            mpChannel->publish(pMsg);
        }
    }

    MsgPtr CameraMono::getNextStream(const MsgPtr& msg) {

        if (!mpVideoCap) {
            DLOG(WARNING) << "CamStream::getNextStream, VideoCapture is not opened, abort\n";
            return nullptr;
        }

        if (!msg) {
            DLOG(WARNING) << "CamStream::getNextStream, Null message detected, abort\n";
            return nullptr;
        }

        cv::Mat image;
        mpVideoCap->read(image);

        auto ts_chrono = chrono::time_point_cast<chrono::nanoseconds>(chrono::system_clock::now());
        auto ts = ts_chrono.time_since_epoch().count();

        ImagePtr imgObj = make_shared<ImageTs>(image.clone(), ts, "");
        auto msgSensor = make_shared<MsgSensorData>(ID_TP_SDATA, imgObj);
        return msgSensor;
    }

    MsgPtr CameraMono::getNextOffline(const MsgPtr& msg) {

        if (!msg) {
            DLOG(WARNING) << "CamOffline::getNext, Null message detected, abort\n";
            return nullptr;
        }

        if (!mpImgDS) {
            DLOG(WARNING) << "CamOffline::getNext, ImageDS is not configured, abort\n";
            return nullptr;
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
        auto msgSensor = make_shared<MsgSensorData>(DEF_CAT, imgObj,
                                                    DEF_TOPIC, FCN_SEN_GET_NEXT);

        return msgSensor;
    }

    void CameraMono::getNextImageFile(string &path, double &ts) {

        if (mpImgDS) {
            if (mSensorFile.empty()) {
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
                    if (mSensorBase.empty()) {
                        path = mSeqPath + '/' + vData[1];
                    }
                    else {
                        path = mSeqPath + '/' + mSensorBase + '/' + vData[1];
                    }
                }
                else if (!vData.empty() && TabularTextDS::isComment(vData[0])) {
                    path = vData[0];
                }
            }
        }
    }

    void CameraMono::run() {

        switch (mCamOp) {
            case OFFLINE:
                this->runOffline();
                break;
            case STREAM:
                this->runStream();
                break;
            case BOTH:
                this->runOffline();
                this->runStream();
                break;
            case NONE:
            default:
                DLOG(WARNING) << "CamMixed::run, Action not supported.\n";
                break;
        }
    }

    void CameraMono::runStream() {

        if (mpChannel && mpVideoCap) {

            auto Ts = std::chrono::milliseconds(static_cast<int>(mTs * 1000.f));

            while (mpVideoCap->isOpened()) {

                auto t1 = chrono::high_resolution_clock::now();

                cv::Mat image;
                mMtxCap.lock();
                bool res = mpVideoCap->read(image);
                mMtxCap.unlock();
                auto ts_chrono = chrono::time_point_cast<chrono::nanoseconds>(chrono::system_clock::now());

                if (!res) {
                    break;
                }

                auto ts = ts_chrono.time_since_epoch().count();
                ImagePtr imgObj = make_shared<ImageTs>(image.clone(), ts, "");
                auto msg = make_shared<MsgSensorData>(ID_TP_SDATA, imgObj);

                auto t2 = chrono::high_resolution_clock::now();

                mpChannel->publish(msg);

                auto duration = duration_cast<chrono::milliseconds>(t2 - t1);
                if (Ts > duration) {
                    std::this_thread::sleep_for(Ts - duration);
                }

                if (this->isStopped()) {
                    break;
                }
            }
        }
    }

    void CameraMono::runOffline() {

        if (mpChannel && mpImgDS) {

            double ts = -1.0;
            string nextFile{};
//            auto Ts = std::chrono::milliseconds(static_cast<int>(mTs * 1000.f));

            this->getNextImageFile(nextFile, ts);
            while(!nextFile.empty()) {

//                auto t1 = chrono::high_resolution_clock::now();

                if (TabularTextDS::isComment(nextFile)) {
                    nextFile = string{};
                    this->getNextImageFile(nextFile, ts);
                    continue;
                }

                cv::Mat image = cv::imread(nextFile, cv::IMREAD_UNCHANGED);
                ImagePtr imgObj = make_shared<ImageTs>(image.clone(), ts, nextFile);
                auto msgSensor = make_shared<MsgSensorData>(ID_TP_SDATA, imgObj);

                nextFile = string{};
                this->getNextImageFile(nextFile, ts);

//                auto t2 = chrono::high_resolution_clock::now();

                mpChannel->publish(msgSensor);

//                auto duration = duration_cast<chrono::milliseconds>(t2 - t1);
//                if (Ts > duration) {
//                    std::this_thread::sleep_for(Ts - duration);
//                }

                this->runDelay(static_cast<long>(ts));

                if (this->isStopped()) {
                    break;
                }
            }
        }
    }

    void CameraMono::reset() {
        this->stop();
        if (mpImgDS) {
            mpImgDS->reset();
        }
    }

    std::string CameraMono::printStr(const string &prefix) const {
        ostringstream oss;

        oss << Camera::printStr(prefix);
        oss << prefix << "Sequence Base: " << mSeqPath << "\n";
        oss << prefix << "Sensor Base: " << mSensorBase << "\n";
        oss << prefix << "Sensor File Name: " << mSensorFile << "\n";
        oss << prefix << "Timestamp Factor: " << tsFactor << "\n";
        if (mpImgDS) {
            oss << mpImgDS->printStr(prefix);
        }

        return oss.str();
    }

    void CameraMono::setupStream(const MsgPtr &msg) {

        if (msg && dynamic_pointer_cast<MsgConfig>(msg)) {
            auto pMsgConf = dynamic_pointer_cast<MsgConfig>(msg);
            auto pParamVideoPath = pMsgConf->getConfig();
            if (pParamVideoPath && dynamic_pointer_cast<ParamType<string>>(pParamVideoPath)) {
                mVideoBase = dynamic_pointer_cast<ParamType<string>>(pParamVideoPath)->getValue();
            }
        }

        if (mpInterface) {
            int port = mpInterface->port;
            if (port >= 0) {
                this->initVideoCap(port);
            }
            else if (!mVideoBase.empty() && !mVideoFile.empty()) {
                this->initVideoCap(port, mVideoBase + "/" + mVideoFile);
            }
        }
    }

    void CameraMono::setupOffline(const MsgPtr &msg) {

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
            auto pSeqBase = find_param<ParamType<string>>(PKEY_SEQ_BASE, pParamDS);
            if (pSeqBase) {
                mSeqPath = pSeqBase->getValue();
            }
            // Image base
            auto pImgBase = find_param<ParamType<string>>(PKEY_IMG_BASE, pParamDS);
            if (pImgBase) {
                mSensorBase = pImgBase->getValue();
            }
            // Image file
            auto pFileName = find_param<ParamType<string>>(PKEY_IMG_FILE, pParamDS);
            if (pFileName) {
                mSensorFile = pFileName->getValue();
            }
            // Ts Factor
            auto pTsFactor = find_param<ParamType<double>>(PKEY_TS_FACT, pParamDS);
            if (pTsFactor) {
                tsFactor = pTsFactor->getValue();
            }

            // Create the image data store
//            string imgExt = boost::filesystem::extension(boost::filesystem::path(mSensorFile));
            auto path_sensor = boost::filesystem::path(mSensorFile);
            string imgExt = path_sensor.extension().string();
            if (imgExt.empty()) {
                imgExt = ".jpg";
            }

            if (mSensorFile.empty()) {
                mpImgDS = make_shared<TabularTextDS>(mSeqPath + "/" + mSensorBase, mSensorFile, imgExt);
            }
            else {
                mpImgDS = make_shared<TabularTextDS>(mSeqPath, mSensorFile, imgExt);
                mpImgDS->open();
            }
        }
    }


}   //NAV24