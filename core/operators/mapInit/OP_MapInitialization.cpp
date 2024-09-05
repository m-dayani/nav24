//
// Created by masoud on 9/3/24.
//

#include "OP_MapInitialization.hpp"
#include "Point2D.hpp"
#include "DataConversion.hpp"
#include "Point3D.hpp"

using namespace std;

namespace NAV24::OP {

    MapInitializer::MapInitializer(const cv::Mat &k, float sigma, int iterations) {
        mpTVR = make_shared<TwoViewReconstruction>(k, sigma, iterations);
    }

    MapInitializer::MapInitializer(const cv::Mat &k, const Params2VR &params2VR) {
        mpTVR = make_shared<TwoViewReconstruction>(k, params2VR);
    }

    bool MapInitializer::reconstruct(const std::vector<std::pair<OB::ObsPtr, OB::ObsPtr>> &vpObs,
                                     FramePtr& pFrame1, FramePtr& pFrame2,
                                     std::vector<WO::WoPtr>& vpPt3D) {

        size_t nMatches = vpObs.size();
        vector<cv::KeyPoint> vkpt1(nMatches), vkpt2(nMatches);
        vector<int> vMatches12(nMatches);
        for (size_t i = 0; i < nMatches; i++) {
            vMatches12[i] = (int)i;
            auto pObs1 = vpObs[i].first;
            if (pObs1 && dynamic_pointer_cast<OB::KeyPoint2D>(pObs1)) {
                auto pKpt = dynamic_pointer_cast<OB::KeyPoint2D>(pObs1);
                cv::KeyPoint kpt = pKpt->getKeyPoint();
                kpt.pt = pKpt->getPointUd();
                vkpt1[i] = kpt;
            }
            auto pObs2 = vpObs[i].second;
            if (pObs2 && dynamic_pointer_cast<OB::KeyPoint2D>(pObs2)) {
                auto pKpt = dynamic_pointer_cast<OB::KeyPoint2D>(pObs2);
                cv::KeyPoint kpt = pKpt->getKeyPoint();
                kpt.pt = pKpt->getPointUd();
                vkpt2[i] = kpt;
            }
        }

        cv::Mat R21, t21;
        vector<cv::Point3f> vP3D;
        vector<bool> vbTriangulated;

        bool res = mpTVR->Reconstruct(vkpt1, vkpt2, vMatches12, R21, t21, vP3D, vbTriangulated);

        if (res) {
//            cout << R21 << endl;
//            cout << t21 << endl;
            pFrame1->setPose(make_shared<TF::PoseSE3>("w0", "c0", pFrame1->getTs(), Eigen::Matrix4d::Identity()));
            auto R21_ei = Converter::toMatrix3d(R21);
            auto t21_ei = Converter::toVector3d(t21);
            pFrame2->setPose(make_shared<TF::PoseSE3>("c0", "c1", pFrame2->getTs(), R21_ei, t21_ei));

            assert(vP3D.size() == vbTriangulated.size() && vbTriangulated.size() == vpObs.size());
            vpPt3D.reserve(vP3D.size());
            for (size_t i = 0; i < vbTriangulated.size(); i++) {
                if (!vbTriangulated[i]) {
                    continue;
                }
                auto pt3d = vP3D[i];
                auto pPt3d = make_shared<WO::Point3D>(pt3d.x, pt3d.y, pt3d.z);
                auto pObs1 = vpObs[i].first;
                pObs1->setWorldObject(pPt3d);
                pPt3d->addObservation(pObs1);
                auto pObs2 = vpObs[i].second;
                pObs2->setWorldObject(pPt3d);
                pPt3d->addObservation(pObs2);
                vpPt3D.push_back(pPt3d);
            }
        }
        return res;
    }
} // NAV24::OP