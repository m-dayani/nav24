//
// Created by masoud on 2/6/24.
//

#include "Calibration.hpp"

#include <glog/logging.h>
#include <opencv2/calib3d.hpp>
#include <Eigen/Eigen>

#include "DataConversion.hpp"
#include "Point2D.hpp"
#include "Point3D.hpp"


using namespace std;

namespace NAV24 {

#define PARAM_KEY_INTRINSICS "intrinsics"
#define PARAM_KEY_DIST_TYPE "distType"
#define PARAM_KEY_DIST_COEFS "distCoefs"

    Calibration::Calibration(const ParamPtr& pParams) {
        this->loadParams(pParams);
    }

    void Calibration::loadParams(const ParamPtr& pParams) {

        if (!pParams) {
            DLOG(WARNING) << "Calibration::setup, Null param detected, abort\n";
            return;
        }

        // Load intrinsics
        auto pParamIntrinsics = find_param<ParamSeq<double>>(PARAM_KEY_INTRINSICS, pParams);
        if (pParamIntrinsics) {
            vector<double> vIntrinsics = pParamIntrinsics->getValue();
            if (vIntrinsics.size() >= 4) {

                fx = (float) vIntrinsics[0];
                fy = (float) vIntrinsics[1];
                cx = (float) vIntrinsics[2];
                cy = (float) vIntrinsics[3];

                K_ei << fx, 0.f, cx,
                        0.f, fy, cy,
                        0.f, 0.f, 1.f;

                Eigen::Matrix<double, 3, 3> convK(K_ei.cast<double>());
                K_cv = Converter::toCvMat(convK);
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
            D_cv = cv::Mat((int) vDistCoefs.size(), 1, CV_64FC1);
            int i = 0;
            for (const auto& d : vDistCoefs) {
                D.push_back(d);
                D_cv.at<double>(i, 0) = d;
                i++;
            }
        }
    }

    std::string Calibration::printStr(const std::string& prefix) {

        ostringstream oss;

        oss << prefix << "K: " << K_cv << "\n";
        oss << prefix << "Distortion Type: " << distType << "\n";
        oss << prefix << "D: " << Converter::toString(D) << "\n";

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


    OB::obsPtr Calibration::undistort(const OB::obsPtr &pObs) {

        OB::obsPtr pObsOut = nullptr;

        if (dynamic_pointer_cast<OB::Point2D>(pObs)) {
            auto pObsIn = dynamic_pointer_cast<OB::Point2D>(pObs);
            cv::Mat ptOrig(1, 2, CV_32FC1), ptUndist(1, 2, CV_32FC1);
            ptOrig.at<float>(0, 0) = pObsIn->x;
            ptOrig.at<float>(0, 1) = pObsIn->y;
            // todo: distortion might be more complex
            // todo: this won't work for older versions of opencv
            cv::undistortPoints(ptOrig, ptUndist, K_cv, D_cv);

            pObsOut = make_shared<OB::Point2D>(ptUndist.at<float>(0, 0), ptUndist.at<float>(0, 1));
        }

        return pObsOut;
    }

    OB::obsPtr Calibration::distort(const OB::obsPtr &pObs) {

        // todo: implement distort
        return pObs;
    }

    WO::woPtr Calibration::unproject(const OB::obsPtr& pt2d) {

        WO::woPtr pWobj = nullptr;

        if (dynamic_pointer_cast<OB::Point2D>(pt2d)) {
            auto pObs = dynamic_pointer_cast<OB::Point2D>(pt2d);
            Eigen::Vector3f pt3d, Pt3d;
            pt3d << pObs->x, pObs->y, 1.f;
            Pt3d = K_ei.inverse() * pt3d;
            pWobj = make_shared<WO::Point3D>(Pt3d.x(), Pt3d.y(), Pt3d.z());
        }

        return pWobj;
    }

    OB::obsPtr Calibration::project(const WO::woPtr& pt3d) {

        OB::obsPtr pObs = nullptr;

        vector<cv::Point2d> ptsOut;
        vector<cv::Point3d> ptsTemp;
        cv::Mat rtemp, ttemp;
        rtemp.create( 3, 1, CV_32F );
        rtemp.setTo( 0 );
        rtemp.copyTo( ttemp );

        if (dynamic_pointer_cast<WO::Point3D>(pt3d)) {
            auto pWo = dynamic_pointer_cast<WO::Point3D>(pt3d);

            ptsTemp = {pWo->getPoint()};
            projectPoints(ptsTemp, rtemp, ttemp, K_cv, D_cv, ptsOut);
            pObs = make_shared<OB::Point2D>(ptsOut[0].x, ptsOut[0].y);

//            Eigen::Vector3f Pt3d;
//            Pt3d << pt3d.x, pt3d.y, pt3d.z;
//            Pt3d /= Pt3d[2];
//            Eigen::Vector3f projPt = K_ei * Pt3d;
        }

        return pObs;
    }


}   //NAV24
