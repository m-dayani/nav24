//
// Created by masoud on 2/6/24.
//

#include <glog/logging.h>
#include <opencv2/calib3d.hpp>
#include <boost/serialization/export.hpp>

#include "Calibration.hpp"
#include "Point2D.hpp"
#include "Point3D.hpp"
#include "ParameterBlueprint.h"
#include "DataConversion.hpp"


using namespace std;

namespace NAV24 {

#define PARAM_KEY_INTRINSICS "intrinsics"
#define PARAM_KEY_DIST_TYPE "distType"
#define PARAM_KEY_DIST_COEFS "distCoefs"
#define PARAM_KEY_CAM_RESOLUTION "resolution"
#define PARAM_KEY_R "R"
#define PARAM_KEY_P "P"

    long unsigned int Calibration::nNextId=0;

    std::shared_ptr<Calibration> Calibration::getNewInstance(const ChannelPtr &pChannel, const ParamPtr &pParams) {

        CalibPtr pCalib;
        string calibDistType = Parameter::mergeKey({string(PKEY_CAM_CALIB), string(PARAM_KEY_DIST_TYPE)});
        auto pParamDistType = find_param<ParamType<string>>(calibDistType, pParams);
        if (pParamDistType) {
            string distType = pParamDistType->getValue();
            if (distType == "none") {
                // calibrated pinhole
                pCalib = make_shared<CalibPinhole>(pChannel);
            }
            else if (distType == "radial-tangential") {
                // distorted pinhole
                pCalib = make_shared<CalibPinholeRadTan>(pChannel);
            }
            else if (distType == "kannala-brandt-8") {
                // Kannala-Brandt-8 model (Fisheye)
                pCalib = make_shared<CalibFisheye>(pChannel);
            }
        }

        if (pCalib) {
            auto msgConfig = make_shared<MsgConfig>(ID_CH_PARAMS, pParams);
            pCalib->receive(msgConfig);
        }

        return pCalib;
    }

    Calibration::Calibration(const ChannelPtr &pChannel) : MsgCallback(pChannel),
            mnId(nNextId++), mDistType(GENERIC), mImWidth(0), mImHeight(0), mImSize() {}

    void Calibration::receive(const MsgPtr &msg) {

        if (msg) {
            if (dynamic_pointer_cast<MsgConfig>(msg)) {
                this->setup(msg);
            }
        }
    }

    void Calibration::setup(const MsgPtr &msg) {

        if (msg && dynamic_pointer_cast<MsgConfig>(msg)) {
            auto pParams = dynamic_pointer_cast<MsgConfig>(msg)->getConfig();

            if (!pParams) {
                DLOG(WARNING) << "Calibration::setup, Null param detected, abort\n";
                return;
            }

            // set image resolution
            auto pParamRes = find_param<ParamSeq<int>>(PARAM_KEY_CAM_RESOLUTION, pParams);
            if (pParamRes) {
                vector<int> vImgRes = pParamRes->getValue();
                if (vImgRes.size() >= 2) {
                    mImWidth = vImgRes[0];
                    mImHeight = vImgRes[1];
                    mImSize = cv::Size(mImWidth, mImHeight);
                }
            }
        }
    }

    void Calibration::handleRequest(const MsgPtr &) {}
    void Calibration::run() {}

    std::string Calibration::printStr(const std::string& prefix) const {

        ostringstream oss;

        oss << prefix << "ID: " << mnId << "\n";
        oss << prefix << "Distortion Type: " << mDistType << "\n";
        oss << prefix << "Image Size: " << mImSize << "\n";

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

    bool Calibration::isInImage(const float x, const float y) const {

        return (x >= 0 && x < float(mImWidth)) && (y >= 0 && y < float(mImHeight));
    }

    bool Calibration::isInImage(const float x, const float y, const int imWidth, const int imHeight) {

        return (x >= 0 && x < float(imWidth)) && (y >= 0 && y < float(imHeight));
    }

    bool Calibration::isInImage(const float x, const float y, const cv::Scalar &imageSize) {

        return (x >= 0 && x < imageSize[0]) && (y >= 0 && y < imageSize[1]);
    }

    std::vector<float> Calibration::computeImageBounds(const cv::Mat &image) const {

        vector<float> res(4);

        if(mDistType != PINHOLE) {

            vector<OB::ObsPtr> vpImageCorners;
            vpImageCorners.push_back(make_shared<OB::Point2D>(0.f, 0.f));
            vpImageCorners.push_back(make_shared<OB::Point2D>(image.cols, 0.f));
            vpImageCorners.push_back(make_shared<OB::Point2D>(0.f, image.rows));
            vpImageCorners.push_back(make_shared<OB::Point2D>(image.cols, image.rows));

            this->undistort(vpImageCorners, vpImageCorners);

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

    bool Calibration::wo2vec(const WO::WoPtr &p3d, Eigen::Vector3f &p3d_eig) {

        bool operationSupported = false;

        if (dynamic_pointer_cast<WO::Point3D>(p3d)) {
            auto pWo = dynamic_pointer_cast<WO::Point3D>(p3d)->getPoint();
            p3d_eig << (float)pWo.x, (float)pWo.y, (float)pWo.z;
            operationSupported = true;
        }

        return operationSupported;
    }

    bool Calibration::obs2vec(const OB::ObsPtr &p2d, Eigen::Vector2f &p2d_eig) {

        bool operationSupported = false;

        auto pP2d = dynamic_pointer_cast<OB::Point2D>(p2d);
        if (pP2d) {
            cv::Point2f pObsIn;
            pObsIn = pP2d->getPoint();
            p2d_eig << pObsIn.x, pObsIn.y;
            operationSupported = true;
        }

        return operationSupported;
    }

    void Calibration::vec2wo(const Eigen::Vector3f &, WO::WoPtr &) {

    }

    void Calibration::vec2obs(const Eigen::Vector2f &, OB::ObsPtr &) {

    }

    /* ============================================================================================================== */

    CalibPinhole::CalibPinhole(const ChannelPtr &pChannel) : Calibration(pChannel),
            fx(1.f), fy(1.f), cx(1.f), cy(1.f), fx_1(1.f), fy_1(1.f), mK_cv(), mK_ei() {

        mDistType = PINHOLE;
    }

    void CalibPinhole::setup(const MsgPtr &msg) {
        Calibration::setup(msg);

        if (msg && dynamic_pointer_cast<MsgConfig>(msg)) {
            auto pParams = dynamic_pointer_cast<MsgConfig>(msg)->getConfig();
            if (pParams) {
                auto pCalibParams = pParams->read(PKEY_CAM_CALIB);
                if (pCalibParams) {
                    // Load intrinsics
                    auto pParamIntrinsics = find_param<ParamSeq<double>>(PARAM_KEY_INTRINSICS, pCalibParams);
                    if (pParamIntrinsics) {
                        vector<double> vIntrinsics = pParamIntrinsics->getValue();

                        if (vIntrinsics.size() >= 4) {
                            fx = static_cast<float>(vIntrinsics[0]);
                            fx_1 = 1.f / fx;
                            fy = static_cast<float>(vIntrinsics[1]);
                            fy_1 = 1.f / fy;
                            cx = static_cast<float>(vIntrinsics[2]);
                            cy = static_cast<float>(vIntrinsics[3]);

                            mK_cv = (cv::Mat_<float>(3, 3) << fx, 0.f, cx, 0.f, fy, cy, 0.f, 0.f, 1.f);
                            mK_ei << fx, 0.f, cx, 0.f, fy, cy, 0.f, 0.f, 1.f;
                        }
                    }
                }
            }
        }
    }

    OB::ObsPtr CalibPinhole::project(const WO::WoPtr &Pc) const {

        OB::ObsPtr pObs = nullptr;
        Eigen::Vector3f p3d;
        bool operationSupported = wo2vec(Pc, p3d);

        if (operationSupported) {
            float u = fx * p3d.x() / p3d.z() + cx;
            float v = fy * p3d.y() / p3d.z() + cy;
            pObs = make_shared<OB::Point2D>(u, v);
        }

        return pObs;
    }

    WO::WoPtr CalibPinhole::unproject(const OB::ObsPtr &pt2d) const {

        WO::WoPtr pWobj;
        Eigen::Vector2f p2d;
        bool operationSupported = obs2vec(pt2d, p2d);

        if (operationSupported) {
            float Xc = (p2d.x() - cx) / fx;
            float Yc = (p2d.y() - cy) / fy;
            pWobj = make_shared<WO::Point3D>(Xc, Yc, 1.f);
        }

        return pWobj;
    }

    Eigen::Matrix<double, 2, 3> CalibPinhole::projectJac(const WO::WoPtr& pt3d) const {

        Eigen::Matrix<double, 2, 3> Jac;
        Eigen::Vector3f v3D;
        bool operationSupported = wo2vec(pt3d, v3D);

        if (operationSupported) {
            Jac(0, 0) = fx / v3D[2];
            Jac(0, 1) = 0.f;
            Jac(0, 2) = -fx * v3D[0] / (v3D[2] * v3D[2]);
            Jac(1, 0) = 0.f;
            Jac(1, 1) = fy / v3D[2];
            Jac(1, 2) = -fy * v3D[1] / (v3D[2] * v3D[2]);
        }

        return Jac;
    }

    cv::Mat CalibPinhole::unprojectJac(const OB::ObsPtr&) const {

        cv::Mat Jac(3, 2, CV_32F);
        Jac.at<float>(0, 0) = 1 / fx;
        Jac.at<float>(0, 1) = 0.f;
        Jac.at<float>(1, 0) = 0.f;
        Jac.at<float>(1, 1) = 1 / fy;
        Jac.at<float>(2, 0) = 0.f;
        Jac.at<float>(2, 1) = 0.f;

        return Jac;
    }

    bool CalibPinhole::epipolarConstrain(const CalibPtrRO &pCalib2,
                                         const OB::ObsPtr& kp1, const OB::ObsPtr& kp2,
                                         const PosePtr& pPose_12, float, float unc) const {

        auto pCalibPH2 = dynamic_pointer_cast<const CalibPinhole>(pCalib2);
        if (!pCalibPH2) {
            return false;
        }

        Eigen::Matrix4d T12 = pPose_12->getPose();
        Eigen::Vector3d t12_eig = T12.block<3,1>(0,3);
        Eigen::Matrix3d R12_eig = T12.block<3,3>(0,0);
        cv::Mat t12 = Converter::toCvMat(t12_eig);
        cv::Mat R12 = Converter::toCvMat(R12_eig);

        Eigen::Vector2f pt1, pt2;
        bool res1 = obs2vec(kp1, pt1);
        bool res2 = obs2vec(kp2, pt2);
        if (!res1 || !res2) {
            return false;
        }

        //Compute Fundamental Matrix
        cv::Mat t12x = skewSymmetricMatrix(t12);
        cv::Mat K1 = mK_cv.clone();
        cv::Mat K2 = pCalibPH2->mK_cv.clone();
        cv::Mat F12 = K1.t().inv()*t12x*R12*K2.inv();

        // Epipolar line in second image l = x1'F12 = [a b c]
        const float a = pt1[0]*F12.at<float>(0,0)+pt1[1]*F12.at<float>(1,0)+F12.at<float>(2,0);
        const float b = pt1[0]*F12.at<float>(0,1)+pt1[1]*F12.at<float>(1,1)+F12.at<float>(2,1);
        const float c = pt1[0]*F12.at<float>(0,2)+pt1[1]*F12.at<float>(1,2)+F12.at<float>(2,2);

        const float num = a*pt2[0]+b*pt2[1]+c;

        const float den = a*a+b*b;

        if(den==0)
            return false;

        const float dsqr = num*num/den;

        return dsqr<DEF_EC_DIST_COEF*unc;
    }

    float CalibPinhole::uncertainty2(const Eigen::Matrix<double,2,1> &) const {
        return 1.0;
    }

    cv::Mat CalibPinhole::skewSymmetricMatrix(const cv::Mat &v) {

        return (cv::Mat_<float>(3,3) << 0, -v.at<float>(2), v.at<float>(1),
                                        v.at<float>(2), 0, -v.at<float>(0),
                                        -v.at<float>(1),  v.at<float>(0), 0);
    }

    void CalibPinhole::generateUndistMaps() {}
    void CalibPinhole::undistort(const vector <OB::ObsPtr> &, vector <OB::ObsPtr> &) const {}
    void CalibPinhole::distort(const vector <OB::ObsPtr> &, std::vector<OB::ObsPtr> &) const {}
    void CalibPinhole::undistortMaps(const vector<OB::ObsPtr>&, vector<OB::ObsPtr> &) const {}
    void CalibPinhole::undistImageMaps(const cv::Mat &, cv::Mat &) const {}

    string CalibPinhole::printStr(const string &prefix) const {
        ostringstream oss{Calibration::printStr(prefix)};

        oss << prefix << "Intrinsics [fx, fy, cx, cy]: " << "[" <<
            fx << ", " << fy << ", " << cx << ", " << cy << "]\n";

        return oss.str();
    }

    std::ostream & operator<<(std::ostream &os, const CalibPinhole &ph) {
        os << ph.printStr("");
        return os;
    }

    std::istream & operator>>(std::istream &is, CalibPinhole &ph) {
        float nextParam;
        vector<float> mvParameters(4);
        for(size_t i = 0; i < 4; i++){
            assert(is.good());  //Make sure the input stream is good
            is >> nextParam;
            mvParameters[i] = nextParam;
        }
        // todo: this makes no sense! -> don't need this operator??
        ph.fx = mvParameters[0];
        ph.fy = mvParameters[1];
        ph.cx = mvParameters[2];
        ph.cy = mvParameters[3];
        return is;
    }

    /* ============================================================================================================== */

    CalibPinholeRadTan::CalibPinholeRadTan(const ChannelPtr &pChannel) : CalibPinhole(pChannel),
            mD_cv(), mUndistMapX(), mUndistMapY(), mNewCamMatrix(), mR(), mP() {

        mDistType = PINHOLE_RAD_TAN;
    }

    void CalibPinholeRadTan::setup(const MsgPtr &msg) {
        CalibPinhole::setup(msg);

        if (msg && dynamic_pointer_cast<MsgConfig>(msg)) {
            auto pParams = dynamic_pointer_cast<MsgConfig>(msg)->getConfig();
            if (pParams) {
                auto pCalibParams = pParams->read(PKEY_CAM_CALIB);
                if (pCalibParams) {
                    // Load intrinsics
                    auto pParamIntrinsics = find_param<ParamSeq<double>>(PARAM_KEY_DIST_COEFS, pCalibParams);
                    if (pParamIntrinsics) {
                        vector<double> vD = pParamIntrinsics->getValue();

                        if (vD.size() >= 4) {
                            mD_cv = (cv::Mat_<float>(4, 1) << vD[0], vD[1], vD[2], vD[3]);
                        }
                    }

                    // Rectification Matrix
                    auto pParamR = find_param<ParamType<cv::Mat>>(PARAM_KEY_R, pCalibParams);
                    if (pParamR) {
                        mR = pParamR->getValue();
                    }

                    // Projection Matrix
                    auto pParamP = find_param<ParamType<cv::Mat>>(PARAM_KEY_P, pCalibParams);
                    if (pParamP) {
                        mP = pParamP->getValue();
                    }

                    this->generateUndistMaps();
                }
            }
        }
    }

    void CalibPinholeRadTan::generateUndistMaps() {

        //cv::initUndistortRectifyMap(mK, mDistCoefs, mR, mP,
        //        mImSize, CV_32FC1, mUndistMapX, mUndistMapY);

        mUndistMapX = cv::Mat(mImHeight, mImWidth, CV_32FC1);
        mUndistMapY = cv::Mat(mImHeight, mImWidth, CV_32FC1);

        for (int x = 0; x < mImWidth; x++) {
            for (int y = 0; y < mImHeight; y++) {

                auto pObsSrc = make_shared<OB::Point2D>(x, y);
                auto pObsDst = make_shared<OB::Point2D>(x, y);
                vector<OB::ObsPtr> vpObsDst{pObsDst};
                this->undistort({pObsSrc}, vpObsDst);

                mUndistMapX.at<float>(y, x) = pObsDst->getPoint().x;
                mUndistMapY.at<float>(y, x) = pObsDst->getPoint().y;
            }
        }
    }

    void CalibPinholeRadTan::undistort(const vector<OB::ObsPtr> &vpObsDist, vector<OB::ObsPtr> &vpObs) const {

        size_t nObs = vpObsDist.size();
        vpObs.resize(nObs);
        for (size_t i = 0; i < vpObsDist.size(); i++) {

            auto pObsDist = dynamic_pointer_cast<OB::Point2D>(vpObsDist[i]);
            Eigen::Vector2f p2d;
            bool res = obs2vec(pObsDist, p2d);

            if (res && pObsDist) {
                vector<cv::Point2f> vPts{cv::Point2f(p2d[0], p2d[1])};
                // Some datasets do provide mR, mP, but if you use these you don't get a normalized result
                // fixed this by setting R, P empty Mat, explicitly provide them for other use
                if (mR.empty()) {
                    cv::undistortPoints(vPts, vPts, mK_cv, mD_cv, mK_cv);
                }
                else {
                    cv::undistortPoints(vPts, vPts, mK_cv, mD_cv, mR, mP);
                }
                pObsDist->setPointUd(vPts[0]);
                pObsDist->updateDistorted(true);
                vpObs[i] = pObsDist;
            }
        }
    }

    void CalibPinholeRadTan::distort(const vector <OB::ObsPtr> &pObs, vector <OB::ObsPtr> &vpObsDist) const {
        // todo: implement distort
        vpObsDist = pObs;
    }

    // Attention!! cv undistMaps are like image: size = (height, width)!
    void CalibPinholeRadTan::undistortMaps(const vector<OB::ObsPtr> &srcPts, vector<OB::ObsPtr> &dstPts) const {

        int rowsX = mUndistMapX.rows;
        int colsX = mUndistMapX.cols;
        int rowsY = mUndistMapY.rows;
        int colsY = mUndistMapY.cols;

        dstPts.resize(srcPts.size());

        for (size_t i = 0; i < srcPts.size(); i++) {

            auto pObs = dynamic_pointer_cast<OB::Point2D>(srcPts[i]);
            Eigen::Vector2f p2d;
            bool res = obs2vec(pObs, p2d);

            if (res && pObs) {
                int x = static_cast<int>(p2d[0]);
                int y = static_cast<int>(p2d[1]);

                if (!(rowsX == rowsY && colsX == colsY && x >= 0 && x < colsX && y >= 0 && y < rowsX)) {
                    continue;
                }

                pObs->setPointUd(cv::Point2f(mUndistMapX.at<float>(y, x), mUndistMapY.at<float>(y, x)));
                pObs->updateDistorted(false);
                dstPts[i] = pObs;
            }
        }
    }

    void CalibPinholeRadTan::undistImageMaps(const cv::Mat &srcImage, cv::Mat &dstImage) const {
        cv::remap(srcImage, dstImage, mUndistMapX, mUndistMapY, cv::INTER_LINEAR);
    }

    string CalibPinholeRadTan::printStr(const string &prefix) const {
        return CalibPinhole::printStr(prefix);
    }

    /* ============================================================================================================== */

    CalibFisheye::CalibFisheye(const ChannelPtr &pChannel, const float precision_) : CalibPinholeRadTan(pChannel),
            mvLappingArea(2, 0), precision(precision_) {

        mDistType = KANNALA_BRANDT_8;
    }

    OB::ObsPtr CalibFisheye::project(const WO::WoPtr &Pc) const {

        Eigen::Vector3f p3D;
        bool res = wo2vec(Pc, p3D);
        if (!res) {
            return nullptr;
        }

        const float x2_plus_y2 = p3D.x() * p3D.x() + p3D.y() * p3D.y();
        const float theta = atan2f(sqrtf(x2_plus_y2), p3D.z());
        const float psi = atan2f(p3D.y(), p3D.x());

        const float theta2 = theta * theta;
        const float theta3 = theta * theta2;
        const float theta5 = theta3 * theta2;
        const float theta7 = theta5 * theta2;
        const float theta9 = theta7 * theta2;
        const float r = theta + mD_cv.at<float>(0) * theta3 + mD_cv.at<float>(1) * theta5
                        + mD_cv.at<float>(2) * theta7 + mD_cv.at<float>(3) * theta9;

        return make_shared<OB::Point2D>(static_cast<float>(fx * r * (float)cos(psi) + cx),
                static_cast<float>(fy * r * (float)sin(psi) + cy));
    }

    WO::WoPtr CalibFisheye::unproject(const OB::ObsPtr &pt2d) const {

        Eigen::Vector2f p2D;
        bool res = obs2vec(pt2d, p2D);
        if (!res) {
            return nullptr;
        }

        //Use Newton method to solve for theta with good precision (err ~ e-6)
        cv::Point2f pw((p2D.x() - cx) / fx, (p2D.y() - cy) / fy);
        float scale = 1.f;
        float theta_d = sqrtf(pw.x * pw.x + pw.y * pw.y);
        theta_d = fminf(fmaxf(-CV_PI / 2.f, theta_d), CV_PI / 2.f);

        if (theta_d > 1e-8) {
            //Compensate distortion iteratively
            float theta = theta_d;

            for (int j = 0; j < 10; j++) {
                float theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2, theta8 =
                        theta4 * theta4;
                float k0_theta2 = mD_cv.at<float>(0) * theta2, k1_theta4 = mD_cv.at<float>(1) * theta4;
                float k2_theta6 = mD_cv.at<float>(2) * theta6, k3_theta8 = mD_cv.at<float>(3) * theta8;
                float theta_fix = (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - theta_d) /
                                  (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8);
                theta = theta - theta_fix;
                if (fabsf(theta_fix) < precision)
                    break;
            }
            //scale = theta - theta_d;
            scale = std::tan(theta) / theta_d;
        }

        return make_shared<WO::Point3D>(pw.x * scale, pw.y * scale, 1.f);
    }

    Eigen::Matrix<double, 2, 3> CalibFisheye::projectJac(const WO::WoPtr &pt3d) const {

        Eigen::Vector3f v3D;
        bool res = wo2vec(pt3d, v3D);
        if (!res) {
            return {};
        }

        double x2 = v3D[0] * v3D[0], y2 = v3D[1] * v3D[1], z2 = v3D[2] * v3D[2];
        double r2 = x2 + y2;
        double r = sqrt(r2);
        double r3 = r2 * r;
        double theta = atan2(r, v3D[2]);

        double theta2 = theta * theta, theta3 = theta2 * theta;
        double theta4 = theta2 * theta2, theta5 = theta4 * theta;
        double theta6 = theta2 * theta4, theta7 = theta6 * theta;
        double theta8 = theta4 * theta4, theta9 = theta8 * theta;

        double f = theta + theta3 * mD_cv.at<float>(0) + theta5 * mD_cv.at<float>(1) +
                   theta7 * mD_cv.at<float>(2) + theta9 * mD_cv.at<float>(3);
        double fd = 1 + 3 * mD_cv.at<float>(0) * theta2 + 5 * mD_cv.at<float>(1) * theta4 +
                    7 * mD_cv.at<float>(2) * theta6 + 9 * mD_cv.at<float>(3) * theta8;

        Eigen::Matrix<double, 2, 3> JacGood;
        JacGood(0, 0) = fx * (fd * v3D[2] * x2 / (r2 * (r2 + z2)) + f * y2 / r3);
        JacGood(1, 0) = fy * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) - f * v3D[1] * v3D[0] / r3);

        JacGood(0, 1) = fx * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) - f * v3D[1] * v3D[0] / r3);
        JacGood(1, 1) = fy * (fd * v3D[2] * y2 / (r2 * (r2 + z2)) + f * x2 / r3);

        JacGood(0, 2) = -fx * fd * v3D[0] / (r2 + z2);
        JacGood(1, 2) = -fy * fd * v3D[1] / (r2 + z2);

        return JacGood;
    }

    bool CalibFisheye::epipolarConstrain(const CalibPtrRO &, const OB::ObsPtr &, const OB::ObsPtr &,
                                         const PosePtr &, float, float) const {
        cv::Mat p3D;
        //return this->TriangulateMatches(pCamera2,kp1,kp2,R12,t12,sigmaLevel,unc,p3D) > KB8_DEF_TH_EPC;
        return false;
    }

    float CalibFisheye::uncertainty2(const Eigen::Matrix<double, 2, 1> &) const {
        /*Eigen::Matrix<double,2,1> c;
        c << mvParameters[2], mvParameters[3];
        if ((p2D-c).squaredNorm()>57600) // 240*240 (256)
            return 100.f;
        else
            return 1.0f;*/
        return 1.f;
    }

    void CalibFisheye::generateUndistMaps() {
        //cv::fisheye::initUndistortRectifyMap(mK, mDistCoefs, mR, mP,
        //        mImSize, CV_32FC1, mUndistMapX, mUndistMapY);

        mUndistMapX = cv::Mat(mImHeight, mImWidth, CV_32FC1);
        mUndistMapY = cv::Mat(mImHeight, mImWidth, CV_32FC1);

        for (int x = 0; x < mImWidth; x++) {
            for (int y = 0; y < mImHeight; y++) {

                auto srcPt = make_shared<OB::Point2D>(x, y);
                auto dstPt = make_shared<OB::Point2D>(x, y);
                vector<OB::ObsPtr> dstPts{dstPt};
                this->undistort({srcPt}, dstPts);

                mUndistMapX.at<float>(y, x) = dstPt->getPoint().x;
                mUndistMapY.at<float>(y, x) = dstPt->getPoint().y;
            }
        }
    }

    void CalibFisheye::undistort(const vector <OB::ObsPtr> &vpObsDist, vector <OB::ObsPtr> &vpObs) const {

        vpObs.resize(vpObsDist.size());

        for (size_t i = 0; i < vpObs.size(); i++) {

            auto pObs = dynamic_pointer_cast<OB::Point2D>(vpObsDist[i]);
            Eigen::Vector2f p2D;
            bool res = obs2vec(pObs, p2D);
            if (!res) continue;

            cv::Point2f pDist(p2D[0], p2D[1]), pUndist;
            vector<cv::Point2f> vpDist{pDist}, vpUndist{pUndist};

            cv::fisheye::undistortPoints(vpDist, vpUndist, mK_cv, mD_cv, mR, mP);

            // todo: copy pObs??
            pObs->setPointUd(vpUndist[0]);
            pObs->updateDistorted(false);
            vpObs[i] = pObs;
        }
    }

    string CalibFisheye::printStr(const string &prefix) const {
        ostringstream oss{CalibPinholeRadTan::printStr(prefix)};

        oss << prefix << "precision: " << precision << "\n";
        oss << prefix << "overlapping area: " << mvLappingArea[0] << ", " << mvLappingArea[1] << "\n";

        return oss.str();
    }

    /*
    float CalibFisheye::TriangulateMatches(Calibration *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
            const cv::Mat &R12, const cv::Mat &t12, const float sigmaLevel, const float unc, cv::Mat& p3D) {
        cv::Mat r1 = this->unprojectMat(kp1.pt);
        cv::Mat r2 = pCamera2->unprojectMat(kp2.pt);

        //Check parallax
        cv::Mat r21 = R12*r2;

        const float cosParallaxRays = r1.dot(r21)/(cv::norm(r1)*cv::norm(r21));

        if(cosParallaxRays > KB8_DEF_MIN_PLX){
            return -1;
        }

        //Parallax is good, so we try to triangulate
        cv::Point2f p11,p22;
        const float* pr1 = r1.ptr<float>();
        const float* pr2 = r2.ptr<float>();

        p11.x = pr1[0];
        p11.y = pr1[1];

        p22.x = pr2[0];
        p22.y = pr2[1];

        cv::Mat x3D;
        cv::Mat Tcw1 = (cv::Mat_<float>(3,4) << 1.f,0.f,0.f,0.f,
                                                           0.f,1.f,0.f,0.f,
                                                           0.f,0.f,1.f,0.f);
        cv::Mat Tcw2;
        cv::Mat R21 = R12.t();
        cv::Mat t21 = -R21*t12;
        cv::hconcat(R21,t21,Tcw2);

        Triangulate(p11,p22,Tcw1,Tcw2,x3D);
        cv::Mat x3Dt = x3D.t();

        float z1 = x3D.at<float>(2);
        if(z1 <= 0){
            return -1;
        }

        float z2 = R21.row(2).dot(x3Dt)+t21.at<float>(2);
        if(z2<=0){
            return -1;
        }

        //Check reprojection error
        cv::Point2f uv1 = this->project(x3D);

        float errX1 = uv1.x - kp1.pt.x;
        float errY1 = uv1.y - kp1.pt.y;

        if((errX1*errX1+errY1*errY1)>KB8_DEF_CHISQ_COEF * sigmaLevel){   //Reprojection error is high
            return -1;
        }

        cv::Mat x3D2 = R21 * x3D + t21;
        cv::Point2f uv2 = pCamera2->project(x3D2);

        float errX2 = uv2.x - kp2.pt.x;
        float errY2 = uv2.y - kp2.pt.y;

        if((errX2*errX2+errY2*errY2)>KB8_DEF_CHISQ_COEF * unc){   //Reprojection error is high
            return -1;
        }

        p3D = x3D.clone();

        return z1;
    }
*/
    std::ostream & operator<<(std::ostream &os, const CalibFisheye &kb) {
        os << kb.printStr("");
        return os;
    }

    std::istream & operator>>(std::istream &is, CalibFisheye &) {
        float nextParam;
        for(size_t i = 0; i < 8; i++){
            assert(is.good());  //Make sure the input stream is good
            is >> nextParam;
//            kb.mvParameters[i] = nextParam;
        }
        return is;
    }

/*
    void CalibFisheye::Triangulate(const cv::Point2f &p1, const cv::Point2f &p2, const cv::Mat &Tcw1, const cv::Mat &Tcw2, cv::Mat &x3D)
    {
        cv::Mat A(4,4,CV_32F);

        A.row(0) = p1.x*Tcw1.row(2)-Tcw1.row(0);
        A.row(1) = p1.y*Tcw1.row(2)-Tcw1.row(1);
        A.row(2) = p2.x*Tcw2.row(2)-Tcw2.row(0);
        A.row(3) = p2.y*Tcw2.row(2)-Tcw2.row(1);

        cv::Mat u,w,vt;
        cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
        x3D = vt.row(3).t();
        x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
    }
    */
}   //NAV24
