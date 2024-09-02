//
// Created by masoud on 8/30/24.
//

#include "OP_FtDt.hpp"
#include "ParameterBlueprint.h"
#include "OP_FtDtOrbSlam.hpp"

using namespace std;

namespace NAV24::OP {

#define OP_FT_DT "ft_detector"
#define PKEY_OP_FT_DT_TYPE "type"
#define OP_FT_DT_TYPE_ORB "orb"
#define OP_FT_DT_TYPE_ORB_CV "cv_orb"

    std::shared_ptr<FtDt> FtDt::create(const ParamPtr &pParam, ChannelPtr &pChannel) {

        shared_ptr<FtDt> pDetector = nullptr;

        if (pParam && pChannel) {

            auto pParamName = find_param<ParamType<string>>(PKEY_NAME, pParam);
            if (pParamName) {
                string opName = pParamName->getValue();
                if (opName == OP_FT_DT) {
                    auto pType = find_param<ParamType<string>>(PKEY_OP_FT_DT_TYPE, pParam);
                    if (pType) {
                        string detectorType = pType->getValue();
                        if (detectorType == OP_FT_DT_TYPE_ORB) {
                            // todo: add these parsing inside the OrbSlam detector
                            auto pNft = find_param<ParamType<int>>("nFeatures", pParam);
                            int nFeatures = (pNft) ? pNft->getValue() : 1000;

                            auto pNl = find_param<ParamType<int>>("nLevels", pParam);
                            int nLevels = (pNl) ? pNl->getValue() : 8;

                            auto pIniThFast = find_param<ParamType<int>>("iniThFast", pParam);
                            int iniThFast = (pIniThFast) ? pIniThFast->getValue() : 20;

                            auto pMinThFast = find_param<ParamType<int>>("minThFast", pParam);
                            int minThFast = (pMinThFast) ? pMinThFast->getValue() : 7;

                            auto pScaleFactor = find_param<ParamType<double>>("nFeatures", pParam);
                            float scaleFactor = (pScaleFactor) ? (float) pScaleFactor->getValue() : 1.2f;

                            pDetector = make_shared<FtDtOrbSlam>(nFeatures, scaleFactor, nLevels, iniThFast, minThFast);
                        }
                        else if (detectorType == OP_FT_DT_TYPE_ORB_CV) {
                            auto pNft = find_param<ParamType<int>>("nFeatures", pParam);
                            int nFeatures = (pNft) ? pNft->getValue() : 1000;

                            pDetector = make_shared<FtDtOCV>(nFeatures);
                        }
                    }
                }
            }
        }

        return pDetector;
    }

    int FtDtOCV::detect(FramePtr &pFrame) {

        if (!pFrame || !dynamic_pointer_cast<FrameImgMono>(pFrame)) {
            return -1;
        }

        auto pImgFrame = dynamic_pointer_cast<FrameImgMono>(pFrame);
        cv::Mat img = pImgFrame->getImage()->mImage;

        std::vector<cv::KeyPoint> keyPoints;
        cv::Mat descriptors;
        mpDetector->detectAndCompute( img, cv::noArray(), keyPoints, descriptors );

        size_t nPoints = keyPoints.size();
        assert(nPoints == descriptors.rows);

        vector<OB::ObsPtr> vpObs(nPoints);
        for (int i = 0; i < nPoints; i++) {
            auto pObs = make_shared<OB::KeyPoint2D>(keyPoints[i], descriptors.row(i).clone());
            pObs->setFrameId(pFrame->getId());
            vpObs[i] = pObs;
        }
        pFrame->setObservations(vpObs);

        return nPoints;
    }

    FtDtOCV::FtDtOCV(int nFt) : FtDt(nFt) {

        mpDetector = cv::ORB::create(mnFeatures);
    }
} // NAV24::OP

