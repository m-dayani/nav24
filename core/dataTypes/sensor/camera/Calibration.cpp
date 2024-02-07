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


}   //NAV24