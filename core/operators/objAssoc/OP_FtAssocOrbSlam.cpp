//
// Created by masoud on 8/30/24.
//

#include <glog/logging.h>

#include "OP_FtAssocOrbSlam.hpp"

using namespace std;

namespace NAV24::OP {

    const int FtAssocOrbSlam::TH_HIGH = 100;
    const int FtAssocOrbSlam::TH_LOW = 50;
    const int FtAssocOrbSlam::HISTO_LENGTH = 30;

    FtAssocOrbSlam::FtAssocOrbSlam(float nnratio, bool checkOri):
        mfNNratio(nnratio), mbCheckOrientation(checkOri), windowSize(100.f) {}

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

        auto pGridFrame2 = dynamic_pointer_cast<FrameMonoGrid>(pFrame2);
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
} // NAV24::OP
