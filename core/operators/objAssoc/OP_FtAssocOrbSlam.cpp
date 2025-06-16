//
// Created by masoud on 8/30/24.
//

#include <glog/logging.h>
#include <DBoW2.h>

#include "OP_FtAssocOrbSlam.hpp"
#include "Point3D.hpp"

using namespace std;

namespace NAV24::OP {

    const int FtAssocOrbSlam::TH_HIGH = 100;
    const int FtAssocOrbSlam::TH_LOW = 50;
    const int FtAssocOrbSlam::HISTO_LENGTH = 30;

    FtAssocOrbSlam::FtAssocOrbSlam(float nnratio, bool checkOri):
            mfNNratio(nnratio), mbCheckOrientation(checkOri), windowSize(100.f) {}

    FtAssocOrbSlam::FtAssocOrbSlam(const ChannelPtr& pChannel) : FtAssoc(pChannel),
            mfNNratio(0.6), mbCheckOrientation(false), windowSize(100.f) {}

    float FtAssocOrbSlam::RadiusByViewingCos(const float &viewCos)
    {
        if(viewCos>0.998)
            return 2.5;
        else
            return 4.0;
    }

    void FtAssocOrbSlam::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
    {
        int max1=0;
        int max2=0;
        int max3=0;

        for(int i=0; i<L; i++)
        {
            const int s = histo[i].size();
            if(s>max1)
            {
                max3=max2;
                max2=max1;
                max1=s;
                ind3=ind2;
                ind2=ind1;
                ind1=i;
            }
            else if(s>max2)
            {
                max3=max2;
                max2=s;
                ind3=ind2;
                ind2=i;
            }
            else if(s>max3)
            {
                max3=s;
                ind3=i;
            }
        }

        if(max2<0.1f*(float)max1)
        {
            ind2=-1;
            ind3=-1;
        }
        else if(max3<0.1f*(float)max1)
        {
            ind3=-1;
        }
    }

    // Bit set count operation from
    // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
    int FtAssocOrbSlam::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
    {
        const int *pa = a.ptr<int32_t>();
        const int *pb = b.ptr<int32_t>();

        int dist=0;

        for(int i=0; i<8; i++, pa++, pb++)
        {
            unsigned  int v = *pa ^ *pb;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }

        return dist;
    }

    std::vector<int> FtAssocOrbSlam::matchV(const FramePtr &pFrame1, const FramePtr &pFrame2) {

        int nmatches=0;
        auto vpObs1 = pFrame1->getObservations();
        int nObs1 = vpObs1.size();
        vector<int> vnMatches12 = vector<int>(nObs1,-1);

        vector<int> rotHist[HISTO_LENGTH];
        for(int i=0;i<HISTO_LENGTH;i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f/HISTO_LENGTH;

        auto pGridFrame2 = dynamic_pointer_cast<FrameMonoOS>(pFrame2);
        if (!pGridFrame2) {
            LOG(WARNING) << "FtAssocOrbSlam::match, no grid frame\n";
            return {};
        }
        auto vpObs2 = pFrame2->getObservations();
        int nObs2 = vpObs2.size();
        vector<int> vMatchedDistance(nObs2,INT_MAX);
        vector<int> vnMatches21(nObs2,-1);

        for(size_t i1=0, iend1=nObs1; i1<iend1; i1++)
        {
            auto pKpt1 = dynamic_pointer_cast<OB::KeyPoint2D>(vpObs1[i1]);
            if (!pKpt1) {
                continue;
            }
            cv::KeyPoint kpt1 = pKpt1->getKeyPoint();
            int level1 = kpt1.octave;
            if(level1>0)
                continue;

            vector<size_t> vIndices2 = pGridFrame2->getFeaturesInArea(pKpt1, windowSize, level1, level1);

            if(vIndices2.empty())
                continue;

            cv::Mat d1 = pKpt1->getDescriptor();

            int bestDist = INT_MAX;
            int bestDist2 = INT_MAX;
            int bestIdx2 = -1;

            for(unsigned long i2 : vIndices2)
            {
                auto pKpt2 = dynamic_pointer_cast<OB::KeyPoint2D>(vpObs2[i2]);
                cv::KeyPoint kpt2 = pKpt2->getKeyPoint();
                if (!pKpt2) {
                    continue;
                }
                cv::Mat d2 = pKpt2->getDescriptor();

                int dist = DescriptorDistance(d1,d2);

                if(vMatchedDistance[i2]<=dist)
                    continue;

                if(dist<bestDist)
                {
                    bestDist2=bestDist;
                    bestDist=dist;
                    bestIdx2=i2;
                }
                else if(dist<bestDist2)
                {
                    bestDist2=dist;
                }
            }

            if(bestDist<=TH_LOW)
            {
                if(bestDist<(float)bestDist2*mfNNratio)
                {
                    if(vnMatches21[bestIdx2]>=0)
                    {
                        vnMatches12[vnMatches21[bestIdx2]]=-1;
                        nmatches--;
                    }
                    vnMatches12[i1]=bestIdx2;
                    vnMatches21[bestIdx2]=i1;
                    vMatchedDistance[bestIdx2]=bestDist;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        float rot = kpt1.angle - dynamic_pointer_cast<OB::KeyPoint2D>(vpObs2[bestIdx2])->getKeyPoint().angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(i1);
                    }
                }
            }

        }

        if(mbCheckOrientation)
        {
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                if(i==ind1 || i==ind2 || i==ind3)
                    continue;
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    int idx1 = rotHist[i][j];
                    if(vnMatches12[idx1]>=0)
                    {
                        vnMatches12[idx1]=-1;
                        nmatches--;
                    }
                }
            }

        }

        //Update prev matched
//        for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
//            if(vnMatches12[i1]>=0)
//                vbPrevMatched[i1]=F2.mvKeysUn[vnMatches12[i1]].pt;

//        return nmatches;
        return vnMatches12;
    }

    int FtAssocOrbSlam::match(const FramePtr &pFrame1, const FramePtr &pFrame2, OB::FtTracksPtr &pTracks) {

        vector<int> vMatches12 = this->matchV(pFrame1, pFrame2);
        auto vpObs1 = pFrame1->getObservations();
        auto vpObs2 = pFrame2->getObservations();
        if (vMatches12.empty()) {
            return 0;
        }
        assert(vMatches12.size() == vpObs1.size());

        int cnt = 0;
        for (size_t i = 0; i < vMatches12.size(); i++) {
            int idx2 = vMatches12[i];
            if (idx2 >= 0) {
                pTracks->addMatch(vpObs1[i], vpObs2[idx2]);
                cnt++;
            }
        }

        return cnt;
    }

    void FtAssocOrbSlam::match(const FramePtr &pFrame1, const FramePtr &pFrame2) {

        vector<int> matches12 = this->matchV(pFrame1, pFrame2);
        int nMatches = 0;
        for (const auto& m : matches12) {
            if (m >= 0) {
                nMatches++;
            }
        }
        auto pMatchedObs = make_shared<OB::MatchedObs>(pFrame1, matches12, nMatches);
        if (dynamic_pointer_cast<FrameImgMono>(pFrame2)) {
            dynamic_pointer_cast<FrameImgMono>(pFrame2)->setMatches(pMatchedObs);
        }
    }

    void FtAssocOrbSlam::receive(const MsgPtr &msg) {
        Operator::receive(msg);

        if (msg) {
            if (dynamic_pointer_cast<MsgConfig>(msg)) {
                this->setup(msg);
            }
        }
    }

    void FtAssocOrbSlam::setup(const MsgPtr &msg) {
        Operator::setup(msg);

        if (msg && dynamic_pointer_cast<MsgConfig>(msg)) {
            const auto pParams = dynamic_pointer_cast<MsgConfig>(msg)->getConfig();
            if (pParams) {
                auto pOpName = find_param<ParamType<string>>("name", pParams);
                if (!pOpName || pOpName->getValue() != "ft_detector") {
                    return;
                }

                auto pParamNLevels = find_param<ParamType<int>>("nLevels", pParams);
                int nLevels = (pParamNLevels) ? pParamNLevels->getValue() : 1;

                auto pParamScaleFactor = find_param<ParamType<double>>("scaleFactor", pParams);
                float scaleFactor = (pParamScaleFactor) ? (float) pParamScaleFactor->getValue() : 1.f;

                mPInfo = ImgPyramidInfo(nLevels, scaleFactor);
            }
        }
    }

    int FtAssocOrbSlam::searchForTriangulation(const FramePtr &mpKF1, const FramePtr &mpKF2, const CalibPtrRO& pCalib,
                                               vector <std::pair<std::size_t, std::size_t>> &vMatchedPairs,
                                               const bool bOnlyStereo, const bool bCoarse)  {

        auto pKF1 = dynamic_pointer_cast<FrameMonoOS>(mpKF1);
        auto pKF2 = dynamic_pointer_cast<FrameMonoOS>(mpKF2);

        if (!pKF1 || !pKF2 || !pCalib) {
            DLOG(WARNING) << "FtAssocOrbSlam::searchForTriangulation, Keyframes or Calibration is NULL, abort...\n";
            return 0;
        }

        const DBoW2::FeatureVector &vFeatVec1 = pKF1->getFtVecDBoW2();
        const DBoW2::FeatureVector &vFeatVec2 = pKF2->getFtVecDBoW2();

        //Compute epipole in second image
        Eigen::Matrix4f Tw1 = pKF1->getPose()->getPose().cast<float>();
        Eigen::Matrix4f T1w = Tw1.inverse();
        Eigen::Matrix4f Tw2 = pKF2->getPose()->getPose().cast<float>();
        Eigen::Matrix4f T2w = Tw2.inverse();
//        Sophus::SE3f Tw2 = pKF2->GetPoseInverse(); // for convenience
        Eigen::Vector3f Cw = Tw1.block<3,1>(0,3);
        Eigen::Vector4f Cw_h(Cw.x(), Cw.y(), Cw.z(), 1.f);
        Eigen::Vector4f C2_h = T2w * Cw_h;
        auto pC2 = make_shared<WO::Point3D>(C2_h.x(), C2_h.y(), C2_h.z());

        auto ep = dynamic_pointer_cast<OB::Point2D>(pCalib->project(pC2));
        Eigen::Matrix4f T12;
//        Eigen::Matrix4f Tll, Tlr, Trl, Trr;
        Eigen::Matrix3f R12; // for fastest computation
        Eigen::Vector3f t12; // for fastest computation

//        GeometricCamera* pCamera1 = pKF1->mpCamera, *pCamera2 = pKF2->mpCamera;

        // only consider the monocular case
//        if(!pKF1->mpCamera2 && !pKF2->mpCamera2){
            T12 = T1w * Tw2;
            R12 = T12.block<3,3>(0,0);
            t12 = T12.block<3,1>(0,3);
            auto pPose_12 = make_shared<TF::PoseSE3>(-1.0, T12.cast<double>());
//        }
//        else{
//            Sophus::SE3f Tr1w = pKF1->GetRightPose();
//            Sophus::SE3f Twr2 = pKF2->GetRightPoseInverse();
//            Tll = T1w * Tw2;
//            Tlr = T1w * Twr2;
//            Trl = Tr1w * Tw2;
//            Trr = Tr1w * Twr2;
//        }

//        Eigen::Matrix3f Rll = Tll.rotationMatrix(), Rlr  = Tlr.rotationMatrix(), Rrl  = Trl.rotationMatrix(), Rrr  = Trr.rotationMatrix();
//        Eigen::Vector3f tll = Tll.translation(), tlr = Tlr.translation(), trl = Trl.translation(), trr = Trr.translation();

        auto vpObs1 = pKF1->getObservations();
        auto vpObs2 = pKF2->getObservations();

        // Find matches between not tracked key points
        // Matching speed-up by ORB Vocabulary
        // Compare only ORB that share the same node
        int nmatches=0;
        vector<bool> vbMatched2(vpObs2.size(),false);
        vector<int> vMatches12(vpObs1.size(),-1);

        vector<int> rotHist[HISTO_LENGTH];
        for(auto & i : rotHist)
            i.reserve(500);

        const float factor = 1.0f/HISTO_LENGTH;

        auto f1it = vFeatVec1.begin();
        auto f2it = vFeatVec2.begin();
        auto f1end = vFeatVec1.end();
        auto f2end = vFeatVec2.end();

        while(f1it!=f1end && f2it!=f2end)
        {
            if(f1it->first == f2it->first)
            {
                for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
                {
                    const size_t idx1 = f1it->second[i1];

                    const auto& kp1 = dynamic_pointer_cast<OB::KeyPoint2D>(vpObs1[idx1]);
                    if (!kp1) {
                        continue;
                    }

                    auto pMP1 = kp1->getWorldObject();

                    // If there is already a MapPoint skip
                    if(pMP1)
                    {
                        continue;
                    }

//                    const bool bStereo1 = (!pKF1->mpCamera2 && pKF1->mvuRight[idx1]>=0);
//                    if(bOnlyStereo)
//                        if(!bStereo1)
//                            continue;

//                    const bool bRight1 = (pKF1 -> NLeft == -1 || idx1 < pKF1 -> NLeft) ? false : true;

                    const cv::Mat &d1 = kp1->getDescriptor();

                    int bestDist = TH_LOW;
                    int bestIdx2 = -1;

                    for(unsigned long idx2 : f2it->second) {

                        const auto& kp2 = dynamic_pointer_cast<OB::KeyPoint2D>(vpObs2[idx2]);
                        if (!kp2) {
                            continue;
                        }

                        auto pMP2 = kp2->getWorldObject();

                        // If we have already matched or there is a MapPoint skip
                        if(vbMatched2[idx2] || pMP2)
                            continue;

//                        const bool bStereo2 = (!pKF2->mpCamera2 &&  pKF2->mvuRight[idx2]>=0);
//                        if(bOnlyStereo)
//                            if(!bStereo2)
//                                continue;

                        const cv::Mat &d2 = kp2->getDescriptor();

                        const int dist = DescriptorDistance(d1,d2);

                        if (dist > TH_LOW || dist > bestDist) {
                            continue;
                        }

                        // check indices are correct!
//                        const bool bRight2 = (pKF2 -> NLeft == -1 || idx2 < pKF2 -> NLeft) ? false : true;

//                        if(!bStereo1 && !bStereo2 && !pKF1->mpCamera2)
//                        {
                            const float distex = ep->getPoint().x-kp2->getPointUd().x;
                            const float distey = ep->getPoint().y-kp2->getPointUd().y;
                            // pKF2->mvScaleFactors[kp2.octave] => kp2->getScaleFactor()?
                            if(distex*distex+distey*distey<100*mPInfo.mvScaleFactor[kp2->getOctave()])
                            {
                                continue;
                            }
//                        }

//                        if(pKF1->mpCamera2 && pKF2->mpCamera2){
//                            if(bRight1 && bRight2){
//                                R12 = Rrr;
//                                t12 = trr;
//                                T12 = Trr;
//
//                                pCamera1 = pKF1->mpCamera2;
//                                pCamera2 = pKF2->mpCamera2;
//                            }
//                            else if(bRight1 && !bRight2){
//                                R12 = Rrl;
//                                t12 = trl;
//                                T12 = Trl;
//
//                                pCamera1 = pKF1->mpCamera2;
//                                pCamera2 = pKF2->mpCamera;
//                            }
//                            else if(!bRight1 && bRight2){
//                                R12 = Rlr;
//                                t12 = tlr;
//                                T12 = Tlr;
//
//                                pCamera1 = pKF1->mpCamera;
//                                pCamera2 = pKF2->mpCamera2;
//                            }
//                            else{
//                                R12 = Rll;
//                                t12 = tll;
//                                T12 = Tll;
//
//                                pCamera1 = pKF1->mpCamera;
//                                pCamera2 = pKF2->mpCamera;
//                            }
//                        }

                        // pKF->mvLevelSigma2[kp.octave] => getUncertainty()
                        if(bCoarse || pCalib->epipolarConstrain(pCalib,kp1,kp2,pPose_12,
                                                                mPInfo.mvLevelSigma2[kp1->getOctave()],
                                                                mPInfo.mvLevelSigma2[kp2->getOctave()])) {
                            bestIdx2 = static_cast<int>(idx2);
                            bestDist = dist;
                        }
                    }

                    if(bestIdx2>=0)
                    {
                        const auto& kp2 = dynamic_pointer_cast<OB::KeyPoint2D>(vpObs2[bestIdx2]);

                        vMatches12[idx1]=bestIdx2;
                        nmatches++;

                        if(mbCheckOrientation)
                        {
                            float rot = kp1->getAngle() - kp2->getAngle();
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = static_cast<int>(round(rot*factor));
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(static_cast<int>(idx1));
                        }
                    }
                }

                f1it++;
                f2it++;
            }
            else if(f1it->first < f2it->first) {
                f1it = vFeatVec1.lower_bound(f2it->first);
            }
            else {
                f2it = vFeatVec2.lower_bound(f1it->first);
            }
        }

        if(mbCheckOrientation) {
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++) {

                if(i==ind1 || i==ind2 || i==ind3)
                    continue;

                for(int j : rotHist[i]) {
                    vMatches12[j]=-1;
                    nmatches--;
                }
            }

        }

        vMatchedPairs.clear();
        vMatchedPairs.reserve(nmatches);

        for(size_t i=0, iend=vMatches12.size(); i<iend; i++) {

            if(vMatches12[i]<0)
                continue;

            vMatchedPairs.emplace_back(i, vMatches12[i]);
        }

        return nmatches;
    }
} // NAV24::OP
