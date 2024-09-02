//
// Created by masoud on 2/6/24.
//

#include "Calibration.hpp"

#include <glog/logging.h>
#include <Eigen/Eigen>

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

    Calibration::Calibration(const ParamPtr& pParams) {
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


    OB::ObsPtr Calibration::undistort(const OB::ObsPtr &pObs) {

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
        for (int i = 0; i < vpObs.size(); i++) {
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

    bool Calibration::isCalibrated() {

        return distType != "radial-tangential" && distType != "kannala-brandt8";
    }


}   //NAV24
