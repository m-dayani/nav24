//
// Created by masoud on 2/6/24.
//

#include "Calibration.hpp"

#include <glog/logging.h>
#include <Eigen/Eigen>
#include <opencv2/calib3d.hpp>

#include "Point2D.hpp"
#include "Point3D.hpp"
#include "Pinhole.hpp"
#include "PinholeRadTan.hpp"
#include "KannalaBrandt8.hpp"


using namespace std;

namespace NAV24 {

#define PARAM_KEY_INTRINSICS "intrinsics"
#define PARAM_KEY_DIST_TYPE "distType"
#define PARAM_KEY_DIST_COEFS "distCoefs"
#define PARAM_KEY_R "R"
#define PARAM_KEY_P "P"

    Calibration::Calibration(const ParamPtr& pParams) :
            mCamType(CameraType::PINHOLE), mImWidth(0), mImHeight(0) {
        this->loadParams(pParams);
    }

    void Calibration::loadParams(const ParamPtr& pParams) {

        if (!pParams) {
            DLOG(WARNING) << "Calibration::setup, Null param detected, abort\n";
            return;
        }

        vector<float> vParams;
        vParams.reserve(10);

        // Load intrinsics
        auto pParamIntrinsics = find_param<ParamSeq<double>>(PARAM_KEY_INTRINSICS, pParams);
        if (pParamIntrinsics) {
            vector<double> vIntrinsics = pParamIntrinsics->getValue();
            for (const auto& v : vIntrinsics) {
                vParams.push_back((float)v);
            }
        }

        // Load distortion
        auto pParamDisttype = find_param<ParamType<string>>(PARAM_KEY_DIST_TYPE, pParams);
        if (pParamDisttype) {
            distType = pParamDisttype->getValue();
        }

        auto pParamDistCoefs = find_param<ParamSeq<double>>(PARAM_KEY_DIST_COEFS, pParams);
        if (pParamDistCoefs) {
            vector<double> vDistCoefs = pParamDistCoefs->getValue();
            for (const auto& v : vDistCoefs) {
                vParams.push_back((float)v);
            }
        }

        // Rectification Matrix
        auto pParamR = find_param<ParamType<cv::Mat>>(PARAM_KEY_R, pParams);
        cv::Mat R;
        if (pParamR) {
            R = pParamR->getValue();
        }

        // Projection Matrix
        auto pParamP = find_param<ParamType<cv::Mat>>(PARAM_KEY_P, pParams);
        cv::Mat P;
        if (pParamP) {
            P = pParamP->getValue();
        }

        if (vParams.empty()) {
            vParams.resize(4);
        }
        if (distType == "radial-tangential") {
            mpCamModel = make_shared<PinholeRadTan>(vParams);
        }
        else if (distType == "kannala-brandt8") {
            mpCamModel = make_shared<KannalaBrandt8>(vParams);
        }
        else {
            // no distortion
            mpCamModel = make_shared<Pinhole>(vParams);
        }
        if (!R.empty()) {
            mpCamModel->setRectificationMat(R);
        }
        if (!P.empty()) {
            mpCamModel->setProjectionMat(P);
        }
    }

    std::string Calibration::printStr(const std::string& prefix) {

        ostringstream oss;

        oss << prefix << "K: " << getK_cv() << "\n";
        oss << prefix << "Distortion Type: " << distType << "\n";
        oss << prefix << "D: " << getD_cv() << "\n";

        return oss.str();
    }

    ParamPtr Calibration::getCalibParams(const cv::Mat &K, const cv::Mat &D, const string &distType,
                                         vector <ParamPtr> &vpParamHolder) {
        ParamPtr pRoot = make_shared<Parameter>("calib", nullptr, Parameter::NodeType::MAP_NODE);

        ParamPtr pDistType = make_shared<ParamType<string>>("distType", pRoot, distType);
        pDistType->setType(Parameter::NodeType::STRING);
        vector<double> intrinsics = {K.at<double>(0, 0), K.at<double>(1, 1),
                K.at<double>(0, 2), K.at<double>(1, 2)};
        ParamPtr pIntrinsics = make_shared<ParamSeq<double>>("intrinsics", pRoot, intrinsics);
        pIntrinsics->setType(Parameter::NodeType::SEQ_REAL);
        vector<double> dist(D.rows);
        for (size_t i = 0; i < dist.size(); i++) dist[i] = D.at<double>((int) i, 0);
        ParamPtr pDist = make_shared<ParamSeq<double>>("distCoefs", pRoot, dist);
        pDist->setType(Parameter::NodeType::SEQ_REAL);

        pRoot->insertChild("distType", pDistType);
        pRoot->insertChild("intrinsics", pIntrinsics);
        pRoot->insertChild("distCoefs", pDist);

        vpParamHolder.push_back(pRoot);
        vpParamHolder.push_back(pDistType);
        vpParamHolder.push_back(pIntrinsics);
        vpParamHolder.push_back(pDist);

        return pRoot;
    }


    OB::ObsPtr Calibration::undistort(const OB::ObsPtr &pObs) const {

        OB::ObsPtr pObsOut = pObs;

        if (dynamic_pointer_cast<OB::Point2D>(pObs)) {
            auto pObsIn = dynamic_pointer_cast<OB::Point2D>(pObs);
            cv::KeyPoint kpt;
            kpt.pt = pObsIn->getPoint();
            auto vpUndist = mpCamModel->UndistortKeyPoints({kpt});
            pObsIn->setPointUd(vpUndist[0].pt);
            pObsIn->updateDistorted(true);
        }

        return pObsOut;
    }

    OB::ObsPtr Calibration::distort(const OB::ObsPtr &pObs) {

        // todo: implement distort
        return pObs;
    }

    WO::WoPtr Calibration::unproject(const OB::ObsPtr& pt2d) {

        WO::WoPtr pWobj;
        cv::Point2f kpt;

        if (dynamic_pointer_cast<OB::Point2D>(pt2d)) {
            auto pObsIn = dynamic_pointer_cast<OB::Point2D>(pt2d);
            kpt = pObsIn->getPoint();
        }

        auto Pt3d = mpCamModel->unproject(kpt);

        pWobj = make_shared<WO::Point3D>(Pt3d.x, Pt3d.y, Pt3d.z);
        return pWobj;
    }

    OB::ObsPtr Calibration::project(const WO::WoPtr& pt3d) {

        OB::ObsPtr pObs = nullptr;

        if (dynamic_pointer_cast<WO::Point3D>(pt3d)) {
            auto pWo = dynamic_pointer_cast<WO::Point3D>(pt3d);

            auto pt2d = mpCamModel->project(pWo->getPoint());
            pObs = make_shared<OB::Point2D>(pt2d.x, pt2d.y);
        }

        return pObs;
    }

    std::vector<OB::ObsPtr> Calibration::undistort(const vector <OB::ObsPtr> &vpObs) {

        vector<OB::ObsPtr> vpObsOut(vpObs.size());
        for (size_t i = 0; i < vpObs.size(); i++) {
            vpObsOut[i] = undistort(vpObs[i]);
        }
        return vpObsOut;
    }

    std::vector<float> Calibration::computeImageBounds(const cv::Mat &image) {

        vector<float> res(4);

        if(!this->isCalibrated()) {

            vector<OB::ObsPtr> vpImageCorners;
            vpImageCorners.push_back(make_shared<OB::Point2D>(0.f, 0.f));
            vpImageCorners.push_back(make_shared<OB::Point2D>(image.cols, 0.f));
            vpImageCorners.push_back(make_shared<OB::Point2D>(0.f, image.rows));
            vpImageCorners.push_back(make_shared<OB::Point2D>(image.cols, image.rows));

            vpImageCorners = this->undistort(vpImageCorners);

            vector<float> vCoords;
            vCoords.reserve(8);
            for (const auto& pImageCorner : vpImageCorners) {
                auto point = dynamic_pointer_cast<OB::Point2D>(pImageCorner)->getPointUd();
                vCoords.push_back(point.x);
                vCoords.push_back(point.y);
            }

            res[0] = min(vCoords[0], vCoords[4]);
            res[1] = max(vCoords[2], vCoords[6]);
            res[2] = min(vCoords[1], vCoords[3]);
            res[3] = max(vCoords[5], vCoords[7]);
        }
        else {
            res = {0.f, (float)image.cols, 0.f, (float)image.rows};
        }

        return res;
    }

    bool Calibration::isCalibrated() const {

        return distType != "radial-tangential" && distType != "kannala-brandt8";
    }

    Eigen::Vector2d Calibration::project(const Eigen::Vector3d &pt3d) {
        if (mpCamModel) {
            return mpCamModel->project(pt3d);
        }
        return {};
    }

    Eigen::Matrix<double, 2, 3> Calibration::projectJac(const Eigen::Vector3d &pt3d) {
        if (mpCamModel) {
            return mpCamModel->projectJac(pt3d);
        }
        return {};
    }


    bool Calibration::isInImage(const float x, const float y) const {

        return (x >= 0 && x < float(mImWidth)) && (y >= 0 && y < float(mImHeight));
    }

    bool Calibration::isInImage(const float x, const float y, const int imWidth, const int imHeight) {

        return (x >= 0 && x < float(imWidth)) && (y >= 0 && y < float(imHeight));
    }

    bool Calibration::isInImage(const float x, const float y, const cv::Scalar &imageSize) {

        return (x >= 0 && x < imageSize[0]) && (y >= 0 && y < imageSize[1]);
    }

    bool Calibration::isDistorted(const cv::Mat &distCoefs) {

        return !distCoefs.empty() && distCoefs.rows * distCoefs.cols >= 4 &&
               fabs(distCoefs.at<float>(0)) > 1e-9;
    }

    void Calibration::generateUndistMaps() {

        if (this->isPinhole()) {
            DLOG(INFO) << "Generating Pinhole distortion maps...\n";
            this->generateUndistMapsPinhole();
        }
        else if (this->isFishEye()) {
            DLOG(INFO) << "Generating FishEye distortion maps...\n";
            this->generateUndistMapsFishEye();
        }
    }

    void Calibration::generateUndistMapsPinhole() {

        //cv::initUndistortRectifyMap(mK, mDistCoefs, mR, mP,
        //        mImSize, CV_32FC1, mUndistMapX, mUndistMapY);

        mUndistMapX = cv::Mat(mImHeight, mImWidth, CV_32FC1);
        mUndistMapY = cv::Mat(mImHeight, mImWidth, CV_32FC1);

        for (int x = 0; x < mImWidth; x++) {
            for (int y = 0; y < mImHeight; y++) {

                cv::Point2f srcPt((float) x, (float) y);
                this->undistPointPinhole(srcPt, srcPt);

                mUndistMapX.at<float>(y, x) = srcPt.x;
                mUndistMapY.at<float>(y, x) = srcPt.y;
            }
        }
    }

    void Calibration::generateUndistMapsFishEye() {

        //cv::fisheye::initUndistortRectifyMap(mK, mDistCoefs, mR, mP,
        //        mImSize, CV_32FC1, mUndistMapX, mUndistMapY);

        mUndistMapX = cv::Mat(mImHeight, mImWidth, CV_32FC1);
        mUndistMapY = cv::Mat(mImHeight, mImWidth, CV_32FC1);

        for (int x = 0; x < mImWidth; x++) {
            for (int y = 0; y < mImHeight; y++) {

                cv::Point2f srcPt((float) x, (float) y);
                this->undistPointFishEye(srcPt, srcPt);

                mUndistMapX.at<float>(y, x) = srcPt.x;
                mUndistMapY.at<float>(y, x) = srcPt.y;
            }
        }
    }

    void Calibration::undistPoint(const cv::Point2f &srcPt, cv::Point2f &dstPt) {

        if (this->isPinhole()) {
            this->undistPointPinhole(srcPt, dstPt);
        }
        else if (this->isFishEye()) {
            this->undistPointFishEye(srcPt, dstPt);
        }
    }

    void Calibration::undistPointPinhole(const cv::Point2f &srcPt, cv::Point2f &dstPt) {

        undistPointPinhole(srcPt, dstPt, mK, mDistCoefs, mR, mP);
    }

    void Calibration::undistPointPinhole(const cv::Point2f &srcPt, cv::Point2f &dstPt, const cv::Mat &K,
                                          const cv::Mat &distCoefs, const cv::Mat& R, const cv::Mat& P) {

        if(!isDistorted(distCoefs)) {
            DLOG_EVERY_N(WARNING, 1000) << "MyCalibrator::undistPointPinhole: Point is not distorted -> nothing to do!\n";
            dstPt = srcPt;
            return;
        }

        cv::Mat srcMat(1, 1, CV_32FC2, cv::Scalar(srcPt.x, srcPt.y));
        cv::undistortPoints(srcMat, srcMat, K, distCoefs, R, P);
        srcMat.reshape(1);

        dstPt.x = srcMat.at<float>(0,0);
        dstPt.y = srcMat.at<float>(0,1);
    }

    void Calibration::undistPointFishEye(const cv::Point2f &srcPt, cv::Point2f &dstPt) {

        undistPointFishEye(srcPt, dstPt, mK, mDistCoefs, mR, mP);
    }

    void Calibration::undistPointFishEye(const cv::Point2f &srcPt, cv::Point2f &dstPt, const cv::Mat &K,
                                          const cv::Mat &distCoefs, const cv::Mat& R, const cv::Mat& P) {

        if(!isDistorted(distCoefs)) {
            DLOG_EVERY_N(WARNING, 1000) << "MyCalibrator::undistPointFishEye: Point is not distorted -> nothing to do!\n";
            dstPt = srcPt;
            return;
        }

        cv::Mat srcMat(1, 1, CV_32FC2, cv::Scalar(srcPt.x, srcPt.y));
        cv::fisheye::undistortPoints(srcMat, srcMat, K, distCoefs, R, P);
        srcMat.reshape(1);

        dstPt.x = srcMat.at<float>(0,0);
        dstPt.y = srcMat.at<float>(0,1);
    }

    void Calibration::undistPointMaps(const cv::Point2f &srcPt, cv::Point2f &dstPt) {

        undistPointMaps(srcPt, dstPt, mUndistMapX, mUndistMapY);
    }

    // Attention!! cv undistMaps are like image: size = (height, width)!
    void Calibration::undistPointMaps(const cv::Point2f &srcPt, cv::Point2f &dstPt,
                                       const cv::Mat &mapX, const cv::Mat &mapY) {

        int rowsX = mapX.rows;
        int colsX = mapX.cols;
        int rowsY = mapY.rows;
        int colsY = mapY.cols;

        int x = static_cast<int>(srcPt.x);
        int y = static_cast<int>(srcPt.y);

        if (!(rowsX == rowsY && colsX == colsY && x >= 0 && x < colsX && y >= 0 && y < rowsX)) {
            dstPt.x = -1;
            dstPt.y = -1;
            return;
        }

        dstPt.x = mapX.at<float>(y, x);
        dstPt.y = mapY.at<float>(y, x);
    }

    void Calibration::undistKeyPoints(const std::vector<cv::KeyPoint> &vDistKPts, std::vector<cv::KeyPoint> &vUndistKPts) {

        if (this->isPinhole()) {
            this->undistKeyPointsPinhole(vDistKPts, vUndistKPts);
        }
        else if (this->isFishEye()) {
            this->undistKeyPointsFishEye(vDistKPts, vUndistKPts);
            //this->undistKeyPointsPinhole(vDistKPts, vUndistKPts);
            //vUndistKPts = vDistKPts;
        }
    }

    void Calibration::undistKeyPointsPinhole(const std::vector<cv::KeyPoint> &vDistKPts, std::vector<cv::KeyPoint> &vUndistKPts) {

        undistKeyPointsPinhole(vDistKPts, vUndistKPts, mK, mDistCoefs, mR, mP);
    }

    void Calibration::undistKeyPointsPinhole(const std::vector<cv::KeyPoint> &vDistKPts,
                                              std::vector<cv::KeyPoint> &vUndistKPts, const cv::Mat &K, const cv::Mat &distCoefs,
                                              const cv::Mat& R, const cv::Mat& P) {

        if (vDistKPts.empty()) {
            LOG(WARNING) << "MyCalibrator::undistKeyPointsPinhole: Empty key point vector -> nothing to do!\n";
            return;
        }
        if(!isDistorted(distCoefs)) {
            DLOG(WARNING) << "MyCalibrator::undistKeyPointsPinhole: Key points are not distorted -> nothing to do!\n";
            vUndistKPts = vDistKPts;
            return;
        }

        int nPts = (int) vDistKPts.size();
        // Fill matrix with points
        cv::Mat mat(nPts,2, CV_32F);

        for(int i=0; i<nPts; i++)
        {
            mat.at<float>(i,0)=vDistKPts[i].pt.x;
            mat.at<float>(i,1)=vDistKPts[i].pt.y;
        }

        // Undistort points
        mat=mat.reshape(2);
        cv::undistortPoints(mat, mat, K, distCoefs, R, P);
        mat=mat.reshape(1);


        // Fill undistorted keypoint vector
        vUndistKPts.resize(nPts);
        for(int i=0; i<nPts; i++)
        {
            cv::KeyPoint kp = vDistKPts[i];
            kp.pt.x=mat.at<float>(i,0);
            kp.pt.y=mat.at<float>(i,1);
            vUndistKPts[i]=kp;
        }
    }

    void Calibration::undistKeyPointsFishEye(const std::vector<cv::KeyPoint> &vDistKPts,
                                              std::vector<cv::KeyPoint> &vUndistKPts) {

        undistKeyPointsFishEye(vDistKPts, vUndistKPts, mK, mDistCoefs, mR, mP);
    }

    void Calibration::undistKeyPointsFishEye(const std::vector<cv::KeyPoint> &vDistKPts,
                                              std::vector<cv::KeyPoint> &vUndistKPts, const cv::Mat &K, const cv::Mat &distCoefs,
                                              const cv::Mat& R, const cv::Mat& P) {

        if (vDistKPts.empty()) {
            LOG(WARNING) << "MyCalibrator::undistKeyPointsFishEye: Empty key point vector -> nothing to do!\n";
            return;
        }
        if(!isDistorted(distCoefs)) {
            DLOG(WARNING) << "MyCalibrator::undistKeyPointsFishEye: Key points are not distorted -> nothing to do!\n";
            vUndistKPts = vDistKPts;
            return;
        }

        int nPts = (int) vDistKPts.size();
        // Fill matrix with points
        cv::Mat mat(nPts,2, CV_32F);

        for(int i=0; i<nPts; i++)
        {
            mat.at<float>(i,0)=vDistKPts[i].pt.x;
            mat.at<float>(i,1)=vDistKPts[i].pt.y;
        }

        // Undistort points
        mat=mat.reshape(2);
        cv::fisheye::undistortPoints(mat, mat, K, distCoefs, R, P);
        mat=mat.reshape(1);

        // Fill undistorted keypoint vector
        vUndistKPts.resize(nPts);
        for(int i=0; i<nPts; i++)
        {
            cv::KeyPoint kp = vDistKPts[i];
            kp.pt.x=mat.at<float>(i,0);
            kp.pt.y=mat.at<float>(i,1);
            vUndistKPts[i]=kp;
        }
    }

    void Calibration::undistImageMaps(const cv::Mat &srcImage, cv::Mat &dstImage) {

        undistImageMaps(srcImage, mUndistMapX, mUndistMapY, dstImage);
    }

    void Calibration::undistImageMaps(const cv::Mat &srcImage, const cv::Mat &mapX, const cv::Mat &mapY, cv::Mat &dstImage) {

        cv::remap(srcImage,dstImage,mapX,mapY,cv::INTER_LINEAR);
    }

}   //NAV24
