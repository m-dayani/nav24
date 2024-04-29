//
// Created by masoud on 2/6/24.
//

#include "Calibration.hpp"

#include <glog/logging.h>

#include "DataConversion.hpp"


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
            DLOG(WARNING) << "Calibration::loadParams, Null param detected, abort\n";
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
            for (const auto& d : vDistCoefs) {
                D.push_back(d);
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
        for (size_t i = 0; i < dist.size(); i++) dist[i] = D.at<double>(i, 0);
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


}   //NAV24