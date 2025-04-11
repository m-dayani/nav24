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
                            pDetector = make_shared<FtDtOrbSlam>(pChannel);
                        }
                        else if (detectorType == OP_FT_DT_TYPE_ORB_CV) {
                            pDetector = make_shared<FtDtOCV>(pChannel);
                        }
                    }
                }
            }

            pChannel->registerChannel(ID_CH_OP, pDetector);
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
        for (size_t i = 0; i < nPoints; i++) {
            auto pObs = make_shared<OB::KeyPoint2D>(keyPoints[i], descriptors.row(i).clone());
            pObs->setFrame(pFrame);
            vpObs[i] = pObs;
        }
        pFrame->setObservations(vpObs);

        return nPoints;
    }

    FtDtOCV::FtDtOCV(const ChannelPtr& pChannel, int nFt) : FtDt(pChannel, nFt) {

        mpDetector = cv::ORB::create(mnFeatures);
    }

    void FtDtOCV::setup(const MsgPtr &configMsg) {
//        Operator::setup(configMsg);

        if (configMsg && dynamic_pointer_cast<MsgConfig>(configMsg)) {

            auto pParam = dynamic_pointer_cast<MsgConfig>(configMsg)->getConfig();
            if (pParam) {
                auto pNft = find_param<ParamType<int>>("nFeatures", pParam);
                mnFeatures = (pNft) ? pNft->getValue() : 1000;
            }
        }
    }
} // NAV24::OP

