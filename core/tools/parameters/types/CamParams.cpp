//
// Created by root on 5/10/21.
//

#include "CamParams.hpp"
#include "DataConversion.hpp"

using namespace std;

namespace NAV24 {

    CameraType::CameraType(const cv::FileNode &cameraNode) : mCamType(NOT_SUPPORTED) {

        this->read(cameraNode);
    }

    CameraType::CameraType(CameraType *pCamType) : mCamType(pCamType->mCamType) {}

    std::string CameraType::mapCameraType(CameraTypeEnum camType) {

        switch (camType) {
            case PINHOLE:
                return "PinHole";
            case PINHOLE_DISTORTED:
                return "PinHoleDistorted";
            case FISHEYE:
                return "FishEye";
            case KANNALA_BRANDT8:
                return "KannalaBrandt8";
            case NOT_SUPPORTED:
            default:
                return "NotSupported";
        }
    }

    CameraType::CameraTypeEnum CameraType::mapCameraType(const std::string &camType) {

        if (camType == "PinHole") {
            return PINHOLE;
        }
        else if (camType == "PinHoleDistorted") {
            return PINHOLE_DISTORTED;
        }
        else if (camType == "FishEye") {
            return FISHEYE;
        }
        else if (camType == "KannalaBrandt8") {
            return KANNALA_BRANDT8;
        }
        else {
            return NOT_SUPPORTED;
        }
    }

    bool CameraType::isPinHole() const {
        return mCamType == PINHOLE || mCamType == PINHOLE_DISTORTED;
    }

    bool CameraType::isFishEye() const {
        return mCamType == FISHEYE || mCamType == KANNALA_BRANDT8;
    }

    void CameraType::write(cv::FileStorage &fs) const {

        fs << "type" << mapCameraType(mCamType);
    }

    void CameraType::read(const cv::FileNode &camNode) {

        string camTypeStr = YamlParserCV::readString(camNode, "type", mapCameraType(NOT_SUPPORTED));
        mCamType = mapCameraType(camTypeStr);
    }

    string CameraType::printStr(const string &prefix) const {

        ostringstream oss;
        oss << prefix << "Camera Type: " << mapCameraType(mCamType) << endl;
        return oss.str();
    }

    /* -------------------------------------------------------------------------------------------------------------- */

    vector<string> CalibParams::mvIntrinsicsKeys;
    vector<string> CalibParams::mvRTDistKeys;
    vector<string> CalibParams::mvKBDistKeys;

    CalibParams::CalibParams() :
            missParams(false), mDistType(NONE)
    {
        if (mvIntrinsicsKeys.empty()) {

            // Intrinsics Key Map
            mvIntrinsicsKeys = {"fx", "fy", "cx", "cy"};
            // Radial-Tangential Distortion Key Map
            mvRTDistKeys = {"k1", "k2", "p1", "p2", "k3"};
            // Equidistant Distortion Key Map
            mvKBDistKeys = {"k1", "k2", "k3", "k4", "k5"};
        }
    }

    CalibParams::CalibParams(const cv::FileNode &calibNode) : CalibParams() {

        this->read(calibNode);

        if (mDistType == RADIAL_TANGENTIAL) {
            mvDistCoefKeys = mvRTDistKeys;
        }
        else if (mDistType == EQUIDISTANT) {
            mvDistCoefKeys = mvKBDistKeys;
        }
        else {
            missParams = true;
        }

        if(this->checkDistCoefs()) {
            this->initD();
        }
    }

    CalibParams::CalibParams(CalibParams* pCalib) :
            missParams(pCalib->missParams), mDistType(pCalib->mDistType), mvIntrinsics(pCalib->mvIntrinsics),
            mvDistCoefs(pCalib->mvDistCoefs), mK(pCalib->mK.clone()), mD(pCalib->mD.clone()),
            mR(pCalib->mR.clone()), mP(pCalib->mP.clone()), mvDistCoefKeys(pCalib->mvDistCoefKeys)
    {}

    bool CalibParams::checkIntrinsics() {

        if (mvIntrinsics.size() < 4) {
            missParams = true;
            return !missParams;
        }
        return true;
    }

    bool CalibParams::checkDistCoefs() {

        if (mvDistCoefs.size() < 4) {
            missParams = true;
            return !missParams;
        }
        return true;
    }

    void CalibParams::initK() {

        if (!checkIntrinsics()) {
            return;
        }

        mK = cv::Mat::eye(3,3, CV_32FC1);

        mK.at<float>(0,0) = mvIntrinsics[0];
        mK.at<float>(1,1) = mvIntrinsics[1];
        mK.at<float>(0,2) = mvIntrinsics[2];
        mK.at<float>(1,2) = mvIntrinsics[3];
    }

    void CalibParams::initD() {

        if (!checkDistCoefs()) {
            return;
        }

        mD = cv::Mat::zeros(mvDistCoefs.size(), 1, CV_32FC1);

        for (size_t i = 0; i < mvDistCoefs.size() && i < mvDistCoefKeys.size(); i++) {
            mD.at<float>(i) = mvDistCoefs[i];
        }
    }

    CalibParams::DistortionType CalibParams::mapDistType(const std::string &distType) {

        if (distType == "equidistant") {
            return EQUIDISTANT;
        }
        else if (distType == "radial-tangential") {
            return RADIAL_TANGENTIAL;
        }
        else {
            return NONE;
        }
    }

    std::string CalibParams::mapDistType(const DistortionType distType) {

        switch (distType) {
            case EQUIDISTANT:
                return "equidistant";
            case RADIAL_TANGENTIAL:
                return "radial-tangential";
            default:
                return "none";
        }
    }

    void CalibParams::write(cv::FileStorage &fs) const {

        YamlParserCV::writeSequence<float>(fs, "intrinsics", mvIntrinsics);

        YamlParserCV::writeString(fs, "distType", mapDistType(mDistType));

        YamlParserCV::writeSequence<float>(fs, "distCoefs", mvDistCoefs);

        fs << "R" << mR;

        fs << "P" << mP;
    }

    void CalibParams::read(const cv::FileNode &calibNode) {

        // Camera Intrinsics
        YamlParserCV::readSequence<float>(calibNode, "intrinsics", mvIntrinsics);
        if (!this->checkIntrinsics()) {
            return;
        }
        this->initK();

        string distTypeStr = YamlParserCV::readString(calibNode, "distType", mapDistType(RADIAL_TANGENTIAL));
        mDistType = mapDistType(distTypeStr);

        // Distortion Coefficients
        YamlParserCV::readSequence<float>(calibNode, "distCoefs", mvDistCoefs);

        // Rectification Matrix
        calibNode["R"] >> mR;
        if (!mR.empty() && !(mR.rows == 3 && mR.cols == 3)) {
            mR = cv::Mat();
        }

        // Projection Matrix
        calibNode["P"] >> mP;
        if (mP.empty() || !(mP.rows == 3 && (mP.cols == 3 || mP.cols == 4))) {
            mK.copyTo(mP);
        }
    }

    string CalibParams::printStr(const std::string &prefix) const {

        ostringstream oss;

        if (missParams) {
            oss << prefix << "Missing some Calibration Parameters\n";
        }

        oss << prefix << "Intrinsics:\n" << prefix << "\t";
        for (size_t i = 0; i < mvIntrinsicsKeys.size() && i < mvIntrinsics.size(); i++) {
            oss << mvIntrinsicsKeys[i] << ": " << mvIntrinsics[i] << ", ";
        }
        oss << endl;

        oss << prefix << "Distortion Type: " << mapDistType(mDistType) << endl;

        oss << prefix << "Distortion Coefficients:\n" << prefix << "\t";
        for (size_t i = 0; i < mvDistCoefKeys.size() && i < mvDistCoefKeys.size(); i++) {
            oss << mvDistCoefKeys[i] << ": " << mvDistCoefs[i] << ", ";
        }
        oss << endl;

        oss << prefix << "Rectification Matrix:\n";
        oss << Converter::toString(mR, prefix+"\t");

        oss << prefix << "Projection Matrix:\n";
        oss << Converter::toString(mP, prefix+"\t");

        return oss.str();
    }

    /* -------------------------------------------------------------------------------------------------------------- */

    CamParams::CamParams() :
            missParams(false), mImageSize(752, 480), leftCam(true), mLappingBegin(-1),
            mLappingEnd(-1), mbf(0.f), fps(30.0f), mMinFrames(0.f), mMaxFrames(0.f), mbRGB(true),
            mThDepth(0.f), mDepthMapFactor(0.f), mThFarPoints(0.f), mpLinkedCam(nullptr)
    {}

    CamParams::CamParams(const cv::FileStorage &fSettings) : CamParams() {

        cv::FileNode camNode = fSettings["Camera"];
        this->read(camNode);

        if (!camNode["right"].empty()) {

            // Copy this object to share params
            mpLinkedCam = new CamParams(this);
            // Read right camera (different) parameters
            this->read(camNode["right"], mpLinkedCam);
        }
    }

    CamParams::CamParams(CamParams *pCP) :
            missParams(pCP->missParams), mImageSize(pCP->mImageSize), leftCam(pCP->leftCam),
            mLappingBegin(pCP->mLappingBegin), mLappingEnd(pCP->mLappingEnd), mbf(pCP->mbf), mTbc(pCP->mTbc.clone()),
            fps(pCP->fps), mMinFrames(pCP->mMinFrames), mMaxFrames(pCP->mMaxFrames), mbRGB(pCP->mbRGB),
            mThDepth(pCP->mThDepth), mDepthMapFactor(pCP->mDepthMapFactor), mThFarPoints(pCP->mThFarPoints),
            mpCameraType(pCP->mpCameraType), mpCalibParams(pCP->mpCalibParams), mpLinkedCam(pCP->mpLinkedCam)
    {}

    CamParams::~CamParams() {

        delete mpLinkedCam;
    }

    void CamParams::write(cv::FileStorage &fs, CamParams* pCamParams) {

        fs << "Camera" << "{";
        bool rightCam = false;
        if (pCamParams) {
            fs << "right" << "{";
            rightCam = true;
        }
        else {
            pCamParams = this;
        }

        pCamParams->mpCameraType->write(fs);

        pCamParams->mpCalibParams->write(fs);

        vector<int> vImSize{pCamParams->mImageSize.width, pCamParams->mImageSize.height};
        YamlParserCV::writeSequence<int>(fs, "resolution", vImSize);

        YamlParserCV::writeReal(fs, "fps", pCamParams->fps);

        fs << "Tbc" << pCamParams->mTbc;

        YamlParserCV::writeReal(fs, "bf", pCamParams->mbf);

        YamlParserCV::writeBool(fs, "RGB", pCamParams->mbRGB);

        YamlParserCV::writeReal(fs, "thDepth", pCamParams->mThDepth);

        if (rightCam) {
            fs << "}";
        }
        fs << "}";
    }

    void CamParams::read(const cv::FileNode &camNode, CamParams* pCamParams) {

        if (!pCamParams) {
            // Left Camera (default)
            pCamParams = this;
            pCamParams->leftCam = true;
        }
        else {
            pCamParams->leftCam = false;
        }

        // Camera Type
        CameraTypePtr pCamType = make_shared<CameraType>(camNode);
        if (pCamType->mCamType != CameraType::NOT_SUPPORTED) {
            pCamParams->mpCameraType = make_shared<CameraType>(pCamType.get());
        }

        // Calibration Parameters
        CalibParamsPtr pCalibParams = make_shared<CalibParams>(camNode["calib"]);
        if (!pCalibParams->missParams) {
            pCamParams->mpCalibParams = make_shared<CalibParams>(pCalibParams.get());
        }

        // Resolution
        vector<int> vImSize;
        YamlParserCV::readSequence<int>(camNode, "resolution", vImSize);
        if (vImSize.size() == 2) {
            pCamParams->mImageSize = cv::Size(vImSize[0], vImSize[1]);
        }

        // Other
        pCamParams->fps = YamlParserCV::readReal<float>(camNode, "fps", pCamParams->fps);

        cv::Mat Tbc;
        camNode["Tbc"] >> Tbc;
        if (!Tbc.empty()) {
            pCamParams->mTbc = Tbc.clone();
        }

        pCamParams->mbf = YamlParserCV::readReal<float>(camNode, "bf", pCamParams->mbf);

        pCamParams->mbRGB = YamlParserCV::readBool(camNode, "RGB", pCamParams->mbRGB);

        pCamParams->mThDepth = YamlParserCV::readReal<float>(camNode, "thDepth", pCamParams->mThDepth);
    }

    std::string CamParams::printStr(CamParams* pCamParams, const string &prefix) {

        ostringstream oss;

        oss << pCamParams->mpCameraType->printStr(prefix);

        oss << prefix << "Calibration Parameters:\n";
        oss << pCamParams->mpCalibParams->printStr(prefix + "\t");

        oss << prefix << "Image Resolution: " << pCamParams->mImageSize.width
            << "x" << pCamParams->mImageSize.height << endl;

        oss << prefix << "Frame Rate: " << pCamParams->fps << " fps\n";

        string colorOrder = "RGB";
        if (!pCamParams->mbRGB) {
            colorOrder = "BGR";
        }
        oss << prefix << "Color order: " << colorOrder << endl;

        oss << prefix << "Stereo Baseline: " << pCamParams->mbf << endl;

        return oss.str();
    }

    void CamParams::write(cv::FileStorage &fs) const {
        //this->write(fs, this);
    }

    void CamParams::read(const cv::FileNode &node) {
        this->read(node, this);
    }

    string CamParams::printStr(const string &prefix) {
        return this->printStr(this, prefix);
    }

} // NAV24

