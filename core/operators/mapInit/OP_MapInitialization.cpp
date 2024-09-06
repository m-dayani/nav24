//
// Created by masoud on 9/3/24.
//

#include "OP_MapInitialization.hpp"
#include "Point2D.hpp"
#include "DataConversion.hpp"
#include "Point3D.hpp"
#include "Problem.hpp"
#include "BE_GraphOptim.hpp"

using namespace std;

namespace NAV24::OP {

#define DEF_TH_MATCHES 100
#define DEF_TH_DIST_FRAMES 3

    MapInitializer::MapInitializer(const CalibPtr& pCalib, float sigma, int iterations) : mpCalib(pCalib) {
        if (pCalib)
            mpTVR = make_shared<TwoViewReconstruction>(pCalib->getK_cv(), sigma, iterations);
    }

    MapInitializer::MapInitializer(const CalibPtr& pCalib, const Params2VR &params2VR) : mpCalib(pCalib) {
        if (pCalib)
            mpTVR = make_shared<TwoViewReconstruction>(pCalib->getK_cv(), params2VR);
    }

    bool MapInitializer::reconstruct(const std::vector<std::pair<OB::ObsPtr, OB::ObsPtr>> &vpObs,
                                     FramePtr& pFrame1, FramePtr& pFrame2,
                                     std::vector<WO::WoPtr>& vpPt3D) {

        vector<cv::KeyPoint> vkpt1, vkpt2;
        vector<int> vMatches12;
        preprocObsPairs(vpObs, vkpt1, vkpt2, vMatches12);

        cv::Mat R21, t21;
        vector<cv::Point3f> vP3D;
        vector<bool> vbTriangulated;

        bool res = mpTVR->Reconstruct(vkpt1, vkpt2, vMatches12, R21, t21, vP3D, vbTriangulated);

        if (res) {
            postProcMap(pFrame1, pFrame2, vpPt3D, vpObs, R21, t21, vP3D, vbTriangulated);
        }
        return res;
    }

    bool MapInitializer::reconstruct(FramePtr &pFrame1, FramePtr &pFrame2, vector <WO::WoPtr> &vpPt3D) {

        if (!mpTVR || !pFrame1 || !pFrame2) {
            // bad input/state
            return false;
        }

        if (pFrame2->getId() - pFrame1->getId() < DEF_TH_DIST_FRAMES) {
            // frames too close
            return false;
        }

        vector<cv::KeyPoint> vkpt1, vkpt2;
        vector<int> vMatches12;
        int nMch = preprocFrames(pFrame1, pFrame2, vkpt1, vkpt2, vMatches12);
        if (nMch < DEF_TH_MATCHES) {
            // not enough matches
            return false;
        }

        cv::Mat R21, t21;
        vector<cv::Point3f> vP3D;
        vector<bool> vbTriangulated;
        bool res = mpTVR->Reconstruct(vkpt1, vkpt2, vMatches12, R21, t21, vP3D, vbTriangulated);
        if (!res) {
            // bad reconstruction
            return res;
        }

        postProcMap(pFrame1, pFrame2, vpPt3D, R21, t21, vP3D, vbTriangulated);
        return res;
    }

    void MapInitializer::preprocObsPairs(const OB::VecObsPair &vpObs,
                                         std::vector<cv::KeyPoint>& vkpt1, std::vector<cv::KeyPoint>& vkpt2,
                                         std::vector<int>& vMatches12) {

        size_t nMatches = vpObs.size();
        vkpt1.resize(nMatches);
        vkpt2.resize(nMatches);
        vMatches12.resize(nMatches);

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
    }

    void MapInitializer::postProcMap(FramePtr &pFrame1, FramePtr &pFrame2, std::vector<WO::WoPtr>& vpPt3D,
                                     const OB::VecObsPair& vpObs,
                                     const cv::Mat& R21, const cv::Mat& t21, const std::vector<cv::Point3f>& vP3D,
                                     const std::vector<bool>& vbTriangulated) {

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

    int MapInitializer::preprocFrames(const FramePtr &pFrame1, const FramePtr &pFrame2,
                                       std::vector<cv::KeyPoint>& vkpt1, std::vector<cv::KeyPoint>& vkpt2,
                                       std::vector<int>& vMatches12) {
        int nMatches = 0;
        OB::MatchedObs::getMatches(pFrame2, vMatches12, nMatches);
        vkpt1 = OB::KeyPoint2D::toCvKeyPointUd(pFrame1->getObservations());
        vkpt2 = OB::KeyPoint2D::toCvKeyPointUd(pFrame2->getObservations());
        return nMatches;
    }

    void
    MapInitializer::postProcMap(FramePtr &pFrame1, FramePtr &pFrame2, vector <WO::WoPtr> &vpPt3D, const cv::Mat &R21,
                                const cv::Mat &t21, const vector <cv::Point3f> &vP3D,
                                const vector<bool> &vbTriangulated) {

        if (!pFrame1 || !pFrame2 || !dynamic_pointer_cast<FrameImgMono>(pFrame2)) {
            return;
        }
        auto pImgFrame2 = dynamic_pointer_cast<FrameImgMono>(pFrame2);
        auto pMatches12 = pImgFrame2->getMatches();
        if (!pMatches12) {
            return;
        }
        vector<int> vMatches12 = pMatches12->mvMatches12;

        pFrame1->setPose(make_shared<TF::PoseSE3>("w0", "c0", pFrame1->getTs(), Eigen::Matrix4d::Identity()));
        auto R21_ei = Converter::toMatrix3d(R21);
        auto t21_ei = Converter::toVector3d(t21);
        pFrame2->setPose(make_shared<TF::PoseSE3>("c0", "c1", pFrame2->getTs(), R21_ei, t21_ei));

        assert(vP3D.size() == vbTriangulated.size() && vbTriangulated.size() == vMatches12.size());

        auto vpObs1 = pFrame1->getObservations();
        auto vpObs2 = pFrame2->getObservations();

        vpPt3D.reserve(vP3D.size());

        for (size_t i = 0; i < vMatches12.size(); i++) {

            int i2 = vMatches12[i];
            if (i2 < 0 || !vbTriangulated[i]) {
                continue;
            }

            auto pt3d = vP3D[i];
            auto pPt3d = make_shared<WO::Point3D>(pt3d.x, pt3d.y, pt3d.z);
            auto pObs1 = vpObs1[i];
            pObs1->setWorldObject(pPt3d);
            pPt3d->addObservation(pObs1);
            auto pObs2 = vpObs2[i2];
            pObs2->setWorldObject(pPt3d);
            pPt3d->addObservation(pObs2);
            vpPt3D.push_back(pPt3d);
        }

//      cout << "Map Points Triangulated successfully!\n";
        // Init. GBA
        // Formulate BA problem
        pFrame1->setOptFixed(true);
        auto vpFrames = {pFrame1, pFrame2};
        auto pProblem = make_shared<PR_VBA>(vpFrames, mpCalib);
        pProblem->setNumIter(10);
        // Publish to the BA backend
        auto pVbaSolver = make_shared<BE::GraphOptim>();
        pVbaSolver->solve(pProblem);
//      BE::GraphOptim::static_solve(pProblem);
        // Establish links, publish vars
//      cout << "MapInit Problem Solved!\n";

        // todo: normalize scale
    }

} // NAV24::OP