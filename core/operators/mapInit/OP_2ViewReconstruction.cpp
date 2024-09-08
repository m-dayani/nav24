/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <thread>
#include <Eigen/Dense>

#include "../thirdparty/DBoW2/DUtils/Random.h"

#include "OP_2ViewReconstruction.hpp"


using namespace std;

namespace NAV24::OP {

std::string ReconstInfo::print() const {

    ostringstream oss;
    oss << "{mBestGood: " << mnBestGood << ", mSecondBest: " << mnSecondBest << ", mMinGood: " << mnMinGood;
    oss << ", mBestPar: " << mBestParallax << ", mSecondPar: " << mSecondBestPar;
    oss << ", isHomography: " << isHomography << ", HScore: " << mHScore << ", FScore: " << mFScore;
    oss << ", HFRatios: " << mHFRatio << ", nSimilar: " << mnSimilar << ", all nGood: {";
    for (int val : mvnGood) {
        oss << val << ((val == mvnGood.size()-1) ? "}" : ", ");
    }
    oss << endl;

    return oss.str();
}

TwoViewReconstruction::TwoViewReconstruction(const cv::Mat& K, float sigma, int iterations)
{
    mK = K.clone();

    mParams2VR = Params2VR();
    mParams2VR.mSigma = sigma;
    mParams2VR.mMaxRansacIter = iterations;
    mSigma = sigma;
    mSigma2 = sigma*sigma;
    mMaxIterations = iterations;
}

TwoViewReconstruction::TwoViewReconstruction(const cv::Mat& K, const Params2VR& params2VR)
{
    mK = K.clone();

    mParams2VR = params2VR;

    mSigma = mParams2VR.mSigma;
    mSigma2 = mSigma*mSigma;
    mMaxIterations = mParams2VR.mMaxRansacIter;
}

bool TwoViewReconstruction::Reconstruct(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2,
        const vector<int> &vMatches12, cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated)
{
    mvKeys1.clear();
    mvKeys2.clear();

    mvKeys1 = vKeys1;
    mvKeys2 = vKeys2;

    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2
    mvMatches12.clear();
    mvMatches12.reserve(mvKeys2.size());
    mvbMatched1.resize(mvKeys1.size());
    for(size_t i=0, iend=vMatches12.size();i<iend; i++)
    {
        if(vMatches12[i]>=0)
        {
            mvMatches12.emplace_back(i,vMatches12[i]);
            mvbMatched1[i]=true;
        }
        else
            mvbMatched1[i]=false;
    }

    const int N = mvMatches12.size();

    // Indices for minimum set selection
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }

    // Generate sets of 8 points for each RANSAC iteration
    mvSets = vector< vector<size_t> >(mMaxIterations,vector<size_t>(8,0));

    DUtils::Random::SeedRandOnce(0);

    for(int it=0; it<mMaxIterations; it++)
    {
        vAvailableIndices = vAllIndices;

        // Select a minimum set
        for(size_t j=0; j<8; j++)
        {
            int randi = DUtils::Random::RandomInt(0,vAvailableIndices.size()-1);
            int idx = vAvailableIndices[randi];

            mvSets[it][j] = idx;

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
    }

    // Launch threads to compute in parallel a fundamental matrix and a homography
    vector<bool> vbMatchesInliersH, vbMatchesInliersF;
    float SH, SF;
    cv::Mat H, F;

    thread threadH(&TwoViewReconstruction::FindHomography,this,ref(vbMatchesInliersH), ref(SH), ref(H));
    thread threadF(&TwoViewReconstruction::FindFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F));

    // Wait until both threads have finished
    threadH.join();
    threadF.join();

    // Compute ratio of scores
    if(SH+SF == 0.f) return false;
    float RH = SH/(SH+SF);

    float minParallax = mParams2VR.mDefMinParallax;
    int minTriangulated = mParams2VR.mMinTriangulated;

    // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
    if(RH > mParams2VR.mThHFRatio) // if(RH>0.40)
    {
        //cout << "Initialization from Homography" << endl;
        return ReconstructH(vbMatchesInliersH,H, mK,R21,t21,vP3D,
                vbTriangulated,minParallax,minTriangulated);
    }
    else //if(pF_HF>0.6)
    {
        //cout << "Initialization from Fundamental" << endl;
        return ReconstructF(vbMatchesInliersF,F,mK,R21,t21,vP3D,
                vbTriangulated,minParallax,minTriangulated);
    }
}

ReconstInfo TwoViewReconstruction::dummyReconstInfo = ReconstInfo();

bool TwoViewReconstruction::Reconstruct(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2,
        const vector<int> &vMatches12, std::vector<cv::Mat>& R21, std::vector<cv::Mat>& t21,
        std::vector<std::vector<cv::Point3f>>& vP3D, std::vector<std::vector<bool>>& vbTriangulated,
        vector<bool> &vbTransInliers, ReconstInfo& reconstInfo)
{
    mvKeys1.clear();
    mvKeys2.clear();

    mvKeys1 = vKeys1;
    mvKeys2 = vKeys2;

    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2
    mvMatches12.clear();
    mvMatches12.reserve(mvKeys2.size());
    mvbMatched1.resize(mvKeys1.size());
    for(size_t i=0, iend=vMatches12.size();i<iend; i++)
    {
        if(vMatches12[i]>=0)
        {
            mvMatches12.emplace_back(i,vMatches12[i]);
            mvbMatched1[i]=true;
        }
        else
            mvbMatched1[i]=false;
    }

    const int N = mvMatches12.size();

    // Indices for minimum set selection
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }

    // Generate sets of 8 points for each RANSAC iteration
    mvSets = vector< vector<size_t> >(mMaxIterations,vector<size_t>(8,0));

    DUtils::Random::SeedRandOnce(0);

    for(int it=0; it<mMaxIterations; it++)
    {
        vAvailableIndices = vAllIndices;

        // Select a minimum set
        for(size_t j=0; j<8; j++)
        {
            int randi = DUtils::Random::RandomInt(0,vAvailableIndices.size()-1);
            int idx = vAvailableIndices[randi];

            mvSets[it][j] = idx;

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
    }

    // Launch threads to compute in parallel a fundamental matrix and a homography
    vector<bool> vbMatchesInliersH, vbMatchesInliersF;
    float SH, SF;
    cv::Mat H, F;

    thread threadH(&TwoViewReconstruction::FindHomography,this,ref(vbMatchesInliersH), ref(SH), ref(H));
    thread threadF(&TwoViewReconstruction::FindFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F));

    // Wait until both threads have finished
    threadH.join();
    threadF.join();

    // Compute ratio of scores
    if(SH+SF == 0.f) {
        vbTransInliers = vbMatchesInliersF;
        return false;
    }
    float RH = SH/(SH+SF);

    reconstInfo.mHScore = SH;
    reconstInfo.mFScore = SF;
    reconstInfo.mHFRatio = RH;

    float minParallax = mParams2VR.mDefMinParallax;
    int minTriangulated = mParams2VR.mMinTriangulated;

    // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
    if(RH > mParams2VR.mThHFRatio) // if(RH>0.40)
    {
        //cout << "Initialization from Homography" << endl;
        return ReconstructH(vbMatchesInliersH,H, mK,R21,t21,vP3D,
                            vbTriangulated, vbTransInliers, minParallax, minTriangulated, reconstInfo);
    }
    else //if(pF_HF>0.6)
    {
        //cout << "Initialization from Fundamental" << endl;
        return ReconstructF(vbMatchesInliersF,F,mK,R21,t21,vP3D,
                            vbTriangulated, vbTransInliers, minParallax, minTriangulated, reconstInfo);
    }
}

void TwoViewReconstruction::FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21)
{
    // Number of putative matches
    const int N = mvMatches12.size();

    // Normalize coordinates
    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);
    cv::Mat T2inv = T2.inv();

    // Best Results variables
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    cv::Mat H21i, H12i;
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for(int it=0; it<mMaxIterations; it++)
    {
        // Select a minimum set
        for(size_t j=0; j<8; j++)
        {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }

        cv::Mat Hn = ComputeH21(vPn1i,vPn2i);
        H21i = T2inv*Hn*T1;
        H12i = H21i.inv();

        currentScore = CheckHomography(H21i, H12i, vbCurrentInliers, mSigma);

        if(currentScore>score)
        {
            H21 = H21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}


void TwoViewReconstruction::FindFundamental(vector<bool> &vbMatchesInliers, float &score, cv::Mat &F21)
{
    // Number of putative matches
    const int N = vbMatchesInliers.size();

    // Normalize coordinates
    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);
    cv::Mat T2t = T2.t();

    // Best Results variables
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    cv::Mat F21i;
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for(int it=0; it<mMaxIterations; it++)
    {
        // Select a minimum set
        for(int j=0; j<8; j++)
        {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }

        cv::Mat Fn = ComputeF21(vPn1i,vPn2i);

        F21i = T2t*Fn*T1;

        currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);

        if(currentScore>score)
        {
            F21 = F21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}


cv::Mat TwoViewReconstruction::ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    cv::Mat A(2*N,9,CV_32F);

    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(2*i,0) = 0.0;
        A.at<float>(2*i,1) = 0.0;
        A.at<float>(2*i,2) = 0.0;
        A.at<float>(2*i,3) = -u1;
        A.at<float>(2*i,4) = -v1;
        A.at<float>(2*i,5) = -1;
        A.at<float>(2*i,6) = v2*u1;
        A.at<float>(2*i,7) = v2*v1;
        A.at<float>(2*i,8) = v2;

        A.at<float>(2*i+1,0) = u1;
        A.at<float>(2*i+1,1) = v1;
        A.at<float>(2*i+1,2) = 1;
        A.at<float>(2*i+1,3) = 0.0;
        A.at<float>(2*i+1,4) = 0.0;
        A.at<float>(2*i+1,5) = 0.0;
        A.at<float>(2*i+1,6) = -u2*u1;
        A.at<float>(2*i+1,7) = -u2*v1;
        A.at<float>(2*i+1,8) = -u2;

    }

    cv::Mat u,w,vt;

    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    return vt.row(8).reshape(0, 3);
}

cv::Mat TwoViewReconstruction::ComputeF21(const vector<cv::Point2f> &vP1,const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    cv::Mat A(N,9,CV_32F);

    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(i,0) = u2*u1;
        A.at<float>(i,1) = u2*v1;
        A.at<float>(i,2) = u2;
        A.at<float>(i,3) = v2*u1;
        A.at<float>(i,4) = v2*v1;
        A.at<float>(i,5) = v2;
        A.at<float>(i,6) = u1;
        A.at<float>(i,7) = v1;
        A.at<float>(i,8) = 1;
    }

    cv::Mat u,w,vt;

    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    cv::Mat Fpre = vt.row(8).reshape(0, 3);

    cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    w.at<float>(2)=0;

    return  u*cv::Mat::diag(w)*vt;
}

float TwoViewReconstruction::CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma)
{   
    const int N = mvMatches12.size();

    const float h11 = H21.at<float>(0,0);
    const float h12 = H21.at<float>(0,1);
    const float h13 = H21.at<float>(0,2);
    const float h21 = H21.at<float>(1,0);
    const float h22 = H21.at<float>(1,1);
    const float h23 = H21.at<float>(1,2);
    const float h31 = H21.at<float>(2,0);
    const float h32 = H21.at<float>(2,1);
    const float h33 = H21.at<float>(2,2);

    const float h11inv = H12.at<float>(0,0);
    const float h12inv = H12.at<float>(0,1);
    const float h13inv = H12.at<float>(0,2);
    const float h21inv = H12.at<float>(1,0);
    const float h22inv = H12.at<float>(1,1);
    const float h23inv = H12.at<float>(1,2);
    const float h31inv = H12.at<float>(2,0);
    const float h32inv = H12.at<float>(2,1);
    const float h33inv = H12.at<float>(2,2);

    vbMatchesInliers.resize(N);

    float score = 0;

    const float th = mParams2VR.mThChiSqScore;

    const float invSigmaSquare = 1.0f/(sigma*sigma);

    for(int i=0; i<N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in first image
        // x2in1 = H12*x2

        const float w2in1inv = 1.0f/(h31inv*u2+h32inv*v2+h33inv);
        const float u2in1 = (h11inv*u2+h12inv*v2+h13inv)*w2in1inv;
        const float v2in1 = (h21inv*u2+h22inv*v2+h23inv)*w2in1inv;

        const float squareDist1 = (u1-u2in1)*(u1-u2in1)+(v1-v2in1)*(v1-v2in1);

        const float chiSquare1 = squareDist1*invSigmaSquare;

        if(chiSquare1>th)
            bIn = false;
        else
            score += th - chiSquare1;

        // Reprojection error in second image
        // x1in2 = H21*x1

        const float w1in2inv = 1.0f/(h31*u1+h32*v1+h33);
        const float u1in2 = (h11*u1+h12*v1+h13)*w1in2inv;
        const float v1in2 = (h21*u1+h22*v1+h23)*w1in2inv;

        const float squareDist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);

        const float chiSquare2 = squareDist2*invSigmaSquare;

        if(chiSquare2>th)
            bIn = false;
        else
            score += th - chiSquare2;

        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }

    return score;
}

float TwoViewReconstruction::CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma)
{
    const int N = mvMatches12.size();

    const float f11 = F21.at<float>(0,0);
    const float f12 = F21.at<float>(0,1);
    const float f13 = F21.at<float>(0,2);
    const float f21 = F21.at<float>(1,0);
    const float f22 = F21.at<float>(1,1);
    const float f23 = F21.at<float>(1,2);
    const float f31 = F21.at<float>(2,0);
    const float f32 = F21.at<float>(2,1);
    const float f33 = F21.at<float>(2,2);

    vbMatchesInliers.resize(N);

    float score = 0;

    const float th = mParams2VR.mThChiSqF;
    const float thScore = mParams2VR.mThChiSqScore;

    const float invSigmaSquare = 1.0f/(sigma*sigma);

    for(int i=0; i<N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in second image
        // l2=F21x1=(a2,b2,c2)

        const float a2 = f11*u1+f12*v1+f13;
        const float b2 = f21*u1+f22*v1+f23;
        const float c2 = f31*u1+f32*v1+f33;

        const float num2 = a2*u2+b2*v2+c2;

        const float squareDist1 = num2*num2/(a2*a2+b2*b2);

        const float chiSquare1 = squareDist1*invSigmaSquare;

        if(chiSquare1>th)
            bIn = false;
        else
            score += thScore - chiSquare1;

        // Reprojection error in second image
        // l1 =x2tF21=(a1,b1,c1)

        const float a1 = f11*u2+f21*v2+f31;
        const float b1 = f12*u2+f22*v2+f32;
        const float c1 = f13*u2+f23*v2+f33;

        const float num1 = a1*u1+b1*v1+c1;

        const float squareDist2 = num1*num1/(a1*a1+b1*b1);

        const float chiSquare2 = squareDist2*invSigmaSquare;

        if(chiSquare2>th)
            bIn = false;
        else
            score += thScore - chiSquare2;

        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }

    return score;
}

bool TwoViewReconstruction::ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K, cv::Mat &R21,
        cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    int N=0;
    for(const bool matchInlier : vbMatchesInliers)
        if(matchInlier) N++;

    // Compute Essential Matrix from Fundamental Matrix
    cv::Mat E21 = K.t()*F21*K;

    cv::Mat R1, R2, t;

    // Recover the 4 motion hypotheses
    DecomposeE(E21,R1,R2,t);  

    cv::Mat t1=t;
    cv::Mat t2=-t;

    // Reconstruct with the 4 hyphoteses and check
    vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
    vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;
    float parallax1,parallax2, parallax3, parallax4;

    int nGood1 = CheckRT(R1,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D1, 4.0f*mSigma2, vbTriangulated1, parallax1);
    int nGood2 = CheckRT(R2,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D2, 4.0f*mSigma2, vbTriangulated2, parallax2);
    int nGood3 = CheckRT(R1,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D3, 4.0f*mSigma2, vbTriangulated3, parallax3);
    int nGood4 = CheckRT(R2,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D4, 4.0f*mSigma2, vbTriangulated4, parallax4);

    int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

    R21 = cv::Mat();
    t21 = cv::Mat();

    int nMinGood = max(static_cast<int>(mParams2VR.mThMinGoodR*N),minTriangulated);

    float thMaxGoodR = mParams2VR.mThMaxGoodR_F;
    int nsimilar = 0;
    if(nGood1>thMaxGoodR*maxGood)
        nsimilar++;
    if(nGood2>thMaxGoodR*maxGood)
        nsimilar++;
    if(nGood3>thMaxGoodR*maxGood)
        nsimilar++;
    if(nGood4>thMaxGoodR*maxGood)
        nsimilar++;

    // If there is not a clear winner or not enough triangulated points reject initialization
    if(maxGood<nMinGood || nsimilar>1)
    {
        return false;
    }

    // If best reconstruction has enough parallax initialize
    if(maxGood==nGood1)
    {
        if(parallax1>minParallax)
        {
            vP3D = vP3D1;
            vbTriangulated = vbTriangulated1;

            R1.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood2)
    {
        if(parallax2>minParallax)
        {
            vP3D = vP3D2;
            vbTriangulated = vbTriangulated2;

            R2.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood3)
    {
        if(parallax3>minParallax)
        {
            vP3D = vP3D3;
            vbTriangulated = vbTriangulated3;

            R1.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood4)
    {
        if(parallax4>minParallax)
        {
            vP3D = vP3D4;
            vbTriangulated = vbTriangulated4;

            R2.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }

    return false;
}

bool TwoViewReconstruction::ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K, cv::Mat &R21,
        cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    int N=0;
    for(const bool inlier : vbMatchesInliers)
        if(inlier) N++;

    // We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence, 1988
    cv::Mat invK = K.inv();
    cv::Mat A = invK*H21*K;

    cv::Mat U,w,Vt,V;
    cv::SVD::compute(A,w,U,Vt,cv::SVD::FULL_UV);
    V=Vt.t();

    float s = cv::determinant(U)*cv::determinant(Vt);

    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);

    float thRDH = mParams2VR.mThRDHomo;
    if(d1/d2<thRDH || d2/d3<thRDH)
    {
        return false;
    }

    vector<cv::Mat> vR, vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    float aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
    float aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
    float x1[] = {aux1,aux1,-aux1,-aux1};
    float x3[] = {aux3,-aux3,aux3,-aux3};

    //case d'=d2
    float aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);

    float ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=ctheta;
        Rp.at<float>(0,2)=-stheta[i];
        Rp.at<float>(2,0)=stheta[i];
        Rp.at<float>(2,2)=ctheta;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=-x3[i];
        tp*=d1-d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }

    //case d'=-d2
    float aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);

    float cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=cphi;
        Rp.at<float>(0,2)=sphi[i];
        Rp.at<float>(1,1)=-1;
        Rp.at<float>(2,0)=sphi[i];
        Rp.at<float>(2,2)=-cphi;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=x3[i];
        tp*=d1+d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }


    int bestGood = 0;
    int secondBestGood = 0;    
    int bestSolutionIdx = -1;
    float bestParallax = -1;
    vector<cv::Point3f> bestP3D;
    vector<bool> bestTriangulated;

    // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
    // We reconstruct all hypotheses and check in terms of triangulated points and parallax
    for(size_t i=0; i<8; i++)
    {
        float parallaxi;
        vector<cv::Point3f> vP3Di;
        vector<bool> vbTriangulatedi;
        int nGood = CheckRT(vR[i],vt[i],mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K,vP3Di,
                4.0f*mSigma2,vbTriangulatedi, parallaxi);

        if(nGood>bestGood)
        {
            secondBestGood = bestGood;
            bestGood = nGood;
            bestSolutionIdx = i;
            bestParallax = parallaxi;
            bestP3D = vP3Di;
            bestTriangulated = vbTriangulatedi;
        }
        else if(nGood>secondBestGood)
        {
            secondBestGood = nGood;
        }
    }


    if(secondBestGood< mParams2VR.mThMaxGoodR_H * bestGood && bestParallax >= minParallax &&
        bestGood>minTriangulated && bestGood>mParams2VR.mThMinGoodR*N)
    {
        vR[bestSolutionIdx].copyTo(R21);
        vt[bestSolutionIdx].copyTo(t21);
        vP3D = bestP3D;
        vbTriangulated = bestTriangulated;

        return true;
    }

    return false;
}

// TODO: Devise better reconst. state decision
bool TwoViewReconstruction::resolveReconstStatF(ReconstInfo& reconstInfo, const int& N,
        const float& minParallax, const int& minTriangulated) const {

    //int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));
    int maxGood = reconstInfo.mnBestGood;
    int nMinGood = max(static_cast<int>(mParams2VR.mThMinGoodR*N),minTriangulated);
    reconstInfo.mnMinGood = nMinGood;

    float thMaxGoodR = mParams2VR.mThMaxGoodR_F;
    int nsimilar = 0;
    for (const int& nGood : reconstInfo.mvnGood) {
        if ((float)nGood > thMaxGoodR*float(maxGood)) {
            nsimilar++;
        }
    }
    reconstInfo.mnSimilar = static_cast<unsigned char>(nsimilar);

    // If there is not a clear winner or not enough triangulated points reject initialization
    bool distinctTrans = !(maxGood<nMinGood || nsimilar>1);

    // If best reconstruction has enough parallax initialize
    bool plxGood = reconstInfo.mBestParallax > minParallax;

    return plxGood && distinctTrans;
}

bool TwoViewReconstruction::ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K, std::vector<cv::Mat>& R21,
        std::vector<cv::Mat>& t21, std::vector<std::vector<cv::Point3f>>& vP3D, std::vector<std::vector<bool>>& vbTriangulated,
        vector<bool> &vbTransInliers, float minParallax, int minTriangulated, ReconstInfo& reconstInfo)
{
    vbTransInliers = vbMatchesInliers;

    int N=0;
    for(const bool inlier : vbMatchesInliers)
        if(inlier) N++;

    // Compute Essential Matrix from Fundamental Matrix
    cv::Mat E21 = K.t()*F21*K;

    cv::Mat R1, R2, t;

    // Recover the 4 motion hypotheses
    DecomposeE(E21,R1,R2,t);

    cv::Mat t1=t;
    cv::Mat t2=-t;

    // Reconstruct with the 4 hyphoteses and check
    vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
    vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;
    float parallax1,parallax2, parallax3, parallax4;

    int nGood1 = CheckRT(R1,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D1,
            4.0f*mSigma2, vbTriangulated1, parallax1);
    int nGood2 = CheckRT(R2,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D2,
            4.0f*mSigma2, vbTriangulated2, parallax2);
    int nGood3 = CheckRT(R1,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D3,
            4.0f*mSigma2, vbTriangulated3, parallax3);
    int nGood4 = CheckRT(R2,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D4,
            4.0f*mSigma2, vbTriangulated4, parallax4);

    multimap<int, pair<float, pair<pair<cv::Mat, cv::Mat>, pair<vector<cv::Point3f>&, vector<bool>&>>>, greater<int>> mReconst;
    mReconst.insert(make_pair(nGood1, make_pair(parallax1, make_pair(make_pair(R1, t1),
            make_pair(ref(vP3D1), ref(vbTriangulated1))))));
    mReconst.insert(make_pair(nGood2, make_pair(parallax2, make_pair(make_pair(R2, t1),
            make_pair(ref(vP3D2), ref(vbTriangulated2))))));
    mReconst.insert(make_pair(nGood3, make_pair(parallax3, make_pair(make_pair(R1, t2),
            make_pair(ref(vP3D3), ref(vbTriangulated3))))));
    mReconst.insert(make_pair(nGood4, make_pair(parallax4, make_pair(make_pair(R2, t2),
            make_pair(ref(vP3D4), ref(vbTriangulated4))))));

    // Only give the 2 best
    R21.resize(2, cv::Mat());
    t21.resize(2, cv::Mat());
    vP3D.resize(2);
    vbTriangulated.resize(2);

    multimap<int, pair<float, pair<pair<cv::Mat, cv::Mat>, pair<vector<cv::Point3f>&, vector<bool>&>>>, greater<int>>::iterator mReconstIter;
    int i;
    for (i = 0, mReconstIter = mReconst.begin(); mReconstIter != mReconst.end() && i < 2; mReconstIter++, i++) {

        R21[i] = mReconstIter->second.second.first.first.clone();
        t21[i] = mReconstIter->second.second.first.second.clone();
        vP3D[i] = mReconstIter->second.second.second.first;
        vbTriangulated[i] = mReconstIter->second.second.second.second;

        if (i == 0) {
            reconstInfo.mnBestGood = mReconstIter->first;
            reconstInfo.mBestParallax = mReconstIter->second.first;
        }
        else {
            reconstInfo.mnSecondBest = mReconstIter->first;
            reconstInfo.mSecondBestPar = mReconstIter->second.first;
        }

    }
    for (i = 0, mReconstIter = mReconst.begin(); mReconstIter != mReconst.end() && i < 4; mReconstIter++, i++) {

        reconstInfo.mvnGood[i] = mReconstIter->first;
    }
    reconstInfo.isHomography = false;

    return this->resolveReconstStatF(reconstInfo, N, minParallax, minTriangulated);
}

bool TwoViewReconstruction::resolveReconstStatH(ReconstInfo& reconstInfo, const int& N,
                                                const float& minParallax, const int& minTriangulated) const {

    auto bestGood = static_cast<float>(reconstInfo.mnBestGood);
    auto secondBest = static_cast<float>(reconstInfo.mnSecondBest);
    return (secondBest < mParams2VR.mThMaxGoodR_H * bestGood &&
            reconstInfo.mBestParallax >= minParallax && reconstInfo.mnBestGood > minTriangulated &&
            bestGood > mParams2VR.mThMinGoodR*float(N));
}

bool TwoViewReconstruction::ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K, std::vector<cv::Mat>& R21,
        std::vector<cv::Mat>& t21, std::vector<std::vector<cv::Point3f>>& vP3D, std::vector<std::vector<bool>>& vbTriangulated,
        vector<bool> &vbTransInliers, float minParallax, int minTriangulated, ReconstInfo& reconstInfo)
{
    vbTransInliers = vbMatchesInliers;

    int N=0;
    for(const bool inlier : vbMatchesInliers)
        if(inlier) N++;

    // We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence, 1988
    cv::Mat invK = K.inv();
    cv::Mat A = invK*H21*K;

    cv::Mat U,w,Vt,V;
    cv::SVD::compute(A,w,U,Vt,cv::SVD::FULL_UV);
    V=Vt.t();

    float s = cv::determinant(U)*cv::determinant(Vt);

    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);

    float thRDH = mParams2VR.mThRDHomo;
    if(d1/d2<thRDH || d2/d3<thRDH)
    {
        return false;
    }

    vector<cv::Mat> vR, vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    float aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
    float aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
    float x1[] = {aux1,aux1,-aux1,-aux1};
    float x3[] = {aux3,-aux3,aux3,-aux3};

    //case d'=d2
    float aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);

    float ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=ctheta;
        Rp.at<float>(0,2)=-stheta[i];
        Rp.at<float>(2,0)=stheta[i];
        Rp.at<float>(2,2)=ctheta;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=-x3[i];
        tp*=d1-d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }

    //case d'=-d2
    float aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);

    float cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=cphi;
        Rp.at<float>(0,2)=sphi[i];
        Rp.at<float>(1,1)=-1;
        Rp.at<float>(2,0)=sphi[i];
        Rp.at<float>(2,2)=-cphi;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=x3[i];
        tp*=d1+d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }


    int bestGood = 0, secondBestGood = 0;
    int bestSolutionIdx = -1, secondBestSolutionIdx = -1;
    float bestParallax = -1, secondBestParallax = -1;
    vector<cv::Point3f> bestP3D, secondBestP3D;
    vector<bool> bestTriangulated, secondBestTriangulated;

    // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
    // We reconstruct all hypotheses and check in terms of triangulated points and parallax
    for(size_t i=0; i<8; i++)
    {
        float parallaxi;
        vector<cv::Point3f> vP3Di;
        vector<bool> vbTriangulatedi;
        int nGood = CheckRT(vR[i],vt[i],mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K,vP3Di,
                            4.0f*mSigma2,vbTriangulatedi, parallaxi);

        if(nGood>bestGood)
        {
            secondBestGood = bestGood;
            bestGood = nGood;
            secondBestSolutionIdx = bestSolutionIdx;
            bestSolutionIdx = i;
            secondBestParallax = bestParallax;
            bestParallax = parallaxi;
            secondBestP3D = bestP3D;
            bestP3D = vP3Di;
            secondBestTriangulated = bestTriangulated;
            bestTriangulated = vbTriangulatedi;
        }
        else if(nGood>secondBestGood)
        {
            secondBestGood = nGood;
            secondBestSolutionIdx = i;
            secondBestParallax = parallaxi;
            secondBestP3D = vP3Di;
            secondBestTriangulated = vbTriangulatedi;
        }
        reconstInfo.mvnGood[i] = nGood;
    }

    // Return up to 2 results
    R21.clear();
    R21.reserve(2);
    t21.clear();
    t21.reserve(2);
    vP3D.clear();
    vP3D.reserve(2);
    vbTriangulated.clear();
    vbTriangulated.reserve(2);

    R21.push_back(vR[bestSolutionIdx].clone());
    t21.push_back(vt[bestSolutionIdx].clone());
    vP3D.push_back(bestP3D);
    vbTriangulated.push_back(bestTriangulated);
    reconstInfo.mnBestGood = bestGood;
    reconstInfo.mBestParallax = bestParallax;

    R21.push_back(vR[secondBestSolutionIdx].clone());
    t21.push_back(vt[secondBestSolutionIdx].clone());
    vP3D.push_back(secondBestP3D);
    vbTriangulated.push_back(secondBestTriangulated);
    reconstInfo.mnSecondBest = secondBestGood;
    reconstInfo.mSecondBestPar = secondBestParallax;

    return this->resolveReconstStatH(reconstInfo, N, minParallax, minTriangulated);
}

void TwoViewReconstruction::Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
        const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
{
    cv::Mat A(4,4,CV_32F);

    A.row(0) = kp1.pt.x*P1.row(2)-P1.row(0);
    A.row(1) = kp1.pt.y*P1.row(2)-P1.row(1);
    A.row(2) = kp2.pt.x*P2.row(2)-P2.row(0);
    A.row(3) = kp2.pt.y*P2.row(2)-P2.row(1);

    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
}

void TwoViewReconstruction::Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
{
    float meanX = 0;
    float meanY = 0;
    const int N = vKeys.size();

    vNormalizedPoints.resize(N);

    for(int i=0; i<N; i++)
    {
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }

    meanX = meanX/N;
    meanY = meanY/N;

    float meanDevX = 0;
    float meanDevY = 0;

    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

        meanDevX += fabs(vNormalizedPoints[i].x);
        meanDevY += fabs(vNormalizedPoints[i].y);
    }

    meanDevX = meanDevX/N;
    meanDevY = meanDevY/N;

    float sX = 1.0f/meanDevX;
    float sY = 1.0f/meanDevY;

    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }

    T = cv::Mat::eye(3,3,CV_32F);
    T.at<float>(0,0) = sX;
    T.at<float>(1,1) = sY;
    T.at<float>(0,2) = -meanX*sX;
    T.at<float>(1,2) = -meanY*sY;
}


int TwoViewReconstruction::CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
{
    // Calibration parameters
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);

    vbGood = vector<bool>(vKeys1.size(),false);
    vP3D.resize(vKeys1.size());

    vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());

    // Camera 1 Projection Matrix K[I|0]
    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
    K.copyTo(P1.rowRange(0,3).colRange(0,3));

    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);

    // Camera 2 Projection Matrix K[R|t]
    cv::Mat P2(3,4,CV_32F);
    R.copyTo(P2.rowRange(0,3).colRange(0,3));
    t.copyTo(P2.rowRange(0,3).col(3));
    P2 = K*P2;

    cv::Mat O2 = -R.t()*t;

    int nGood=0;

    float thMinParallex = mParams2VR.mMinParallaxCRT;
    for(size_t i=0, iend=vMatches12.size();i<iend;i++)
    {
        if(!vbMatchesInliers[i])
            continue;

        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];
        cv::Mat p3dC1;

        Triangulate(kp1,kp2,P1,P2,p3dC1);

        if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
        {
            vbGood[vMatches12[i].first]=false;
            continue;
        }

        // Check parallax
        cv::Mat normal1 = p3dC1 - O1;
        float dist1 = cv::norm(normal1);

        cv::Mat normal2 = p3dC1 - O2;
        float dist2 = cv::norm(normal2);

        float cosParallax = normal1.dot(normal2)/(dist1*dist2);

        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        if(p3dC1.at<float>(2)<=0 && cosParallax<thMinParallex)
            continue;

        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        cv::Mat p3dC2 = R*p3dC1+t;

        if(p3dC2.at<float>(2)<=0 && cosParallax<thMinParallex)
            continue;

        // Check reprojection error in first image
        float im1x, im1y;
        float invZ1 = 1.0f/p3dC1.at<float>(2);
        im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
        im1y = fy*p3dC1.at<float>(1)*invZ1+cy;

        float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

        if(squareError1>th2)
            continue;

        // Check reprojection error in second image
        float im2x, im2y;
        float invZ2 = 1.0f/p3dC2.at<float>(2);
        im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
        im2y = fy*p3dC2.at<float>(1)*invZ2+cy;

        float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

        if(squareError2>th2)
            continue;

        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
        nGood++;

        if(cosParallax<thMinParallex)
            vbGood[vMatches12[i].first]=true;
    }

    if(nGood>0)
    {
        sort(vCosParallax.begin(),vCosParallax.end());

        size_t idx = min(mParams2VR.mMinTriangulated,int(vCosParallax.size()-1));
        parallax = static_cast<float>(acos(vCosParallax[idx])*180/CV_PI);
    }
    else
        parallax=0;

    return nGood;
}

void TwoViewReconstruction::DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
{
    cv::Mat u,w,vt;
    cv::SVD::compute(E,w,u,vt);

    u.col(2).copyTo(t);
    t=t/cv::norm(t);

    cv::Mat W(3,3,CV_32F,cv::Scalar(0));
    W.at<float>(0,1)=-1;
    W.at<float>(1,0)=1;
    W.at<float>(2,2)=1;

    R1 = u*W*vt;
    if(cv::determinant(R1)<0)
        R1=-R1;

    R2 = u*W.t()*vt;
    if(cv::determinant(R2)<0)
        R2=-R2;
}

    /*void TwoViewReconstruction::CreateNewMapPoints(const std::vector<FramePtr>& vpKeyFrames, const CalibPtr& pCalib,
                                                   const std::shared_ptr<FtAssocOrbSlam>& matcher,
                                                   const bool mbMonocular, const bool mbInertial)
    {
        // This method assumes all keyframes have the same camera model
        // Retrieve neighbor keyframes in co-visibility graph
        int nn = 10;
        // For stereo inertial case
        if(mbMonocular)
            nn=30;
        FramePtr mpCurrentKeyFrame = vpKeyFrames.back();
//        vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

//        if (mbInertial)
//        {
//            KeyFrame* pKF = mpCurrentKeyFrame;
//            int count=0;
//            while((vpNeighKFs.size()<=nn)&&(pKF->mPrevKF)&&(count++<nn))
//            {
//                vector<KeyFrame*>::iterator it = std::find(vpNeighKFs.begin(), vpNeighKFs.end(), pKF->mPrevKF);
//                if(it==vpNeighKFs.end())
//                    vpNeighKFs.push_back(pKF->mPrevKF);
//                pKF = pKF->mPrevKF;
//            }
//        }

        float th = 0.6f;

//        ORBmatcher matcher(th,false);

//        Sophus::SE3<float> sophTcw1 = mpCurrentKeyFrame->GetPose();
        auto pPose1 = mpCurrentKeyFrame->getPose();
        Eigen::Matrix4d Tc1w = pPose1->getPose();
        Eigen::Matrix<float,3,4> eigTcw1 = Tc1w.block<3, 4>(0, 0).cast<float>(); //sophTcw1.matrix3x4();
        Eigen::Matrix<float,3,3> Rcw1 = eigTcw1.block<3,3>(0,0);
        Eigen::Matrix<float,3,3> Rwc1 = Rcw1.transpose();
        Eigen::Vector3f tcw1 = Tc1w.block<3,1>(0,3).cast<float>(); //sophTcw1.translation();
        Eigen::Vector3f Ow1 = -Rwc1 * tcw1; //mpCurrentKeyFrame->GetCameraCenter();

        cv::Mat mK = pCalib->getK_cv();
        const float &fx1 = mK.at<float>(0,0); //mpCurrentKeyFrame->fx;
        const float &fy1 = mK.at<float>(1,1);//mpCurrentKeyFrame->fy;
        const float &cx1 = mK.at<float>(0,2);//mpCurrentKeyFrame->cx;
        const float &cy1 = mK.at<float>(1,2);//mpCurrentKeyFrame->cy;
        const float &invfx1 = 1.f / fx1; //mpCurrentKeyFrame->invfx;
        const float &invfy1 = 1.f / fy1; //mpCurrentKeyFrame->invfy;

        // todo: get scale factor from params
        const float scaleFactor = 1.2f;
        const float ratioFactor = 1.5f*scaleFactor;//mpCurrentKeyFrame->mfScaleFactor;
        int countStereo = 0;
        int countStereoGoodProj = 0;
        int countStereoAttempt = 0;
        int totalStereoPts = 0;
        // Search matches with epipolar restriction and triangulate
        for(size_t i=0; i<vpKeyFrames.size(); i++)
        {
//            if(i>0 && CheckNewKeyFrames())
//                return;
            if (i == vpKeyFrames.size() - 1) {
                // last keyframe is the current keyframe
                break;
            }

//            KeyFrame* pKF2 = vpNeighKFs[i];
            FramePtr pKF2 = vpKeyFrames[i];

            //GeometricCamera* pCamera1 = mpCurrentKeyFrame->mpCamera, *pCamera2 = pKF2->mpCamera;

            // Check first that baseline is not too short
            auto pPose2 = pKF2->getPose();
            Eigen::Vector3f Ow2 = pPose2->getPose().inverse().block<3,1>(0,3).cast<float>();//pKF2->GetCameraCenter();
            Eigen::Vector3f vBaseline = Ow2-Ow1;
            const float baseline = vBaseline.norm();

            if(mbMonocular)
//            {
//                if(baseline<pKF2->mb)
//                    continue;
//            }
//            else
            {
                const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
                const float ratioBaselineDepth = baseline/medianDepthKF2;

                if(ratioBaselineDepth<0.01)
                    continue;
            }

            // Search matches that fulfill epipolar constraint
            vector<pair<size_t,size_t> > vMatchedIndices;
            bool bCoarse = false; //mbInertial && mpTracker->mState==Tracking::RECENTLY_LOST && mpCurrentKeyFrame->GetMap()->GetIniertialBA2();

            matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,vMatchedIndices,false,bCoarse);

            //Sophus::SE3<float> sophTcw2 = pKF2->GetPose();
            Eigen::Matrix<float,3,4> eigTcw2 = pPose2->getPose().block<3,4>(0,0).cast<float>();//sophTcw2.matrix3x4();
            Eigen::Matrix<float,3,3> Rcw2 = eigTcw2.block<3,3>(0,0);
            Eigen::Matrix<float,3,3> Rwc2 = Rcw2.transpose();
            Eigen::Vector3f tcw2 = eigTcw2.block<3,1>(0,3); //sophTcw2.translation();

            const float &fx2 = fx1;//pKF2->fx;
            const float &fy2 = fy1;//pKF2->fy;
            const float &cx2 = cx1;//pKF2->cx;
            const float &cy2 = cy1;//pKF2->cy;
            const float &invfx2 = invfx1;//pKF2->invfx;
            const float &invfy2 = invfy1;//pKF2->invfy;

            auto vpObs1 = mpCurrentKeyFrame->getObservations();
            auto vpObs2 = pKF2->getObservations();

            // Triangulate each match
            const int nmatches = vMatchedIndices.size();
            for(int ikp=0; ikp<nmatches; ikp++)
            {
                const int &idx1 = vMatchedIndices[ikp].first;
                const int &idx2 = vMatchedIndices[ikp].second;

//                const cv::KeyPoint &kp1 = (mpCurrentKeyFrame -> NLeft == -1) ? mpCurrentKeyFrame->mvKeysUn[idx1]
//                                                                             : (idx1 < mpCurrentKeyFrame -> NLeft) ? mpCurrentKeyFrame -> mvKeys[idx1]
//                                                                                                                   : mpCurrentKeyFrame -> mvKeysRight[idx1 - mpCurrentKeyFrame -> NLeft];
                cv::KeyPoint kp1;
                auto pObs1 = vpObs1[idx1];
                if (pObs1 && dynamic_pointer_cast<OB::KeyPoint2D>(pObs1)) {
                    kp1 = dynamic_pointer_cast<OB::KeyPoint2D>(pObs1)->getKeyPointUd();
                }
                //const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
                bool bStereo1 = false; //(!mpCurrentKeyFrame->mpCamera2 && kp1_ur>=0);
                const bool bRight1 = false; //(mpCurrentKeyFrame -> NLeft == -1 || idx1 < mpCurrentKeyFrame -> NLeft) ? false : true;

//                const cv::KeyPoint &kp2 = (pKF2 -> NLeft == -1) ? pKF2->mvKeysUn[idx2]
//                                                                : (idx2 < pKF2 -> NLeft) ? pKF2 -> mvKeys[idx2]
//                                                                                         : pKF2 -> mvKeysRight[idx2 - pKF2 -> NLeft];
                cv::KeyPoint kp2;
                auto pObs2 = vpObs2[idx2];
                if (pObs2 && dynamic_pointer_cast<OB::KeyPoint2D>(pObs2)) {
                    kp2 = dynamic_pointer_cast<OB::KeyPoint2D>(pObs2)->getKeyPointUd();
                }

//                const float kp2_ur = pKF2->mvuRight[idx2];
                bool bStereo2 = false; //(!pKF2->mpCamera2 && kp2_ur>=0);
                const bool bRight2 = false; //(pKF2 -> NLeft == -1 || idx2 < pKF2 -> NLeft) ? false : true;

                if(mpCurrentKeyFrame->mpCamera2 && pKF2->mpCamera2){
//                    if(bRight1 && bRight2) {
//                        sophTcw1 = mpCurrentKeyFrame->GetRightPose();
//                        Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();
//
//                        sophTcw2 = pKF2->GetRightPose();
//                        Ow2 = pKF2->GetRightCameraCenter();
//
//                        pCamera1 = mpCurrentKeyFrame->mpCamera2;
//                        pCamera2 = pKF2->mpCamera2;
//                    }
//                    else if(bRight1 && !bRight2){
//                        sophTcw1 = mpCurrentKeyFrame->GetRightPose();
//                        Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();
//
//                        sophTcw2 = pKF2->GetPose();
//                        Ow2 = pKF2->GetCameraCenter();
//
//                        pCamera1 = mpCurrentKeyFrame->mpCamera2;
//                        pCamera2 = pKF2->mpCamera;
//                    }
//                    else if(!bRight1 && bRight2){
//                        sophTcw1 = mpCurrentKeyFrame->GetPose();
//                        Ow1 = mpCurrentKeyFrame->GetCameraCenter();
//
//                        sophTcw2 = pKF2->GetRightPose();
//                        Ow2 = pKF2->GetRightCameraCenter();
//
//                        pCamera1 = mpCurrentKeyFrame->mpCamera;
//                        pCamera2 = pKF2->mpCamera2;
//                    }
//                    else{
//                        sophTcw1 = mpCurrentKeyFrame->GetPose();
//                        Ow1 = mpCurrentKeyFrame->GetCameraCenter();
//
//                        sophTcw2 = pKF2->GetPose();
//                        Ow2 = pKF2->GetCameraCenter();
//
//                        pCamera1 = mpCurrentKeyFrame->mpCamera;
//                        pCamera2 = pKF2->mpCamera;
//                    }
                    eigTcw1 = sophTcw1.matrix3x4();
                    Rcw1 = eigTcw1.block<3,3>(0,0);
                    Rwc1 = Rcw1.transpose();
                    tcw1 = sophTcw1.translation();

                    eigTcw2 = sophTcw2.matrix3x4();
                    Rcw2 = eigTcw2.block<3,3>(0,0);
                    Rwc2 = Rcw2.transpose();
                    tcw2 = sophTcw2.translation();
                }

                // Check parallax between rays
                Eigen::Vector3f xn1 = pCamera1->unprojectEig(kp1.pt);
                Eigen::Vector3f xn2 = pCamera2->unprojectEig(kp2.pt);

                Eigen::Vector3f ray1 = Rwc1 * xn1;
                Eigen::Vector3f ray2 = Rwc2 * xn2;
                const float cosParallaxRays = ray1.dot(ray2)/(ray1.norm() * ray2.norm());

                float cosParallaxStereo = cosParallaxRays+1;
                float cosParallaxStereo1 = cosParallaxStereo;
                float cosParallaxStereo2 = cosParallaxStereo;

//                if(bStereo1)
//                    cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
//                else if(bStereo2)
//                    cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

//                if (bStereo1 || bStereo2) totalStereoPts++;

                cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

                Eigen::Vector3f x3D;

                bool goodProj = false;
                bool bPointStereo = false;
                if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 ||
                                                                              (cosParallaxRays<0.9996 && mbInertial) || (cosParallaxRays<0.9998 && !mbInertial)))
                {
                    goodProj = GeometricTools::Triangulate(xn1, xn2, eigTcw1, eigTcw2, x3D);
                    if(!goodProj)
                        continue;
                }
                else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
                {
                    countStereoAttempt++;
                    bPointStereo = true;
                    goodProj = mpCurrentKeyFrame->UnprojectStereo(idx1, x3D);
                }
                else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
                {
                    countStereoAttempt++;
                    bPointStereo = true;
                    goodProj = pKF2->UnprojectStereo(idx2, x3D);
                }
                else
                {
                    continue; //No stereo and very low parallax
                }

                if(goodProj && bPointStereo)
                    countStereoGoodProj++;

                if(!goodProj)
                    continue;

                //Check triangulation in front of cameras
                float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
                if(z1<=0)
                    continue;

                float z2 = Rcw2.row(2).dot(x3D) + tcw2(2);
                if(z2<=0)
                    continue;

                //Check reprojection error in first keyframe
                const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
                const float x1 = Rcw1.row(0).dot(x3D)+tcw1(0);
                const float y1 = Rcw1.row(1).dot(x3D)+tcw1(1);
                const float invz1 = 1.0/z1;

                if(!bStereo1)
                {
                    cv::Point2f uv1 = pCamera1->project(cv::Point3f(x1,y1,z1));
                    float errX1 = uv1.x - kp1.pt.x;
                    float errY1 = uv1.y - kp1.pt.y;

                    if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                        continue;

                }
                else
                {
                    float u1 = fx1*x1*invz1+cx1;
                    float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                    float v1 = fy1*y1*invz1+cy1;
                    float errX1 = u1 - kp1.pt.x;
                    float errY1 = v1 - kp1.pt.y;
                    float errX1_r = u1_r - kp1_ur;
                    if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                        continue;
                }

                //Check reprojection error in second keyframe
                const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
                const float x2 = Rcw2.row(0).dot(x3D)+tcw2(0);
                const float y2 = Rcw2.row(1).dot(x3D)+tcw2(1);
                const float invz2 = 1.0/z2;
                if(!bStereo2)
                {
                    cv::Point2f uv2 = pCamera2->project(cv::Point3f(x2,y2,z2));
                    float errX2 = uv2.x - kp2.pt.x;
                    float errY2 = uv2.y - kp2.pt.y;
                    if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                        continue;
                }
                else
                {
                    float u2 = fx2*x2*invz2+cx2;
                    float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                    float v2 = fy2*y2*invz2+cy2;
                    float errX2 = u2 - kp2.pt.x;
                    float errY2 = v2 - kp2.pt.y;
                    float errX2_r = u2_r - kp2_ur;
                    if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                        continue;
                }

                //Check scale consistency
                Eigen::Vector3f normal1 = x3D - Ow1;
                float dist1 = normal1.norm();

                Eigen::Vector3f normal2 = x3D - Ow2;
                float dist2 = normal2.norm();

                if(dist1==0 || dist2==0)
                    continue;

                if(mbFarPoints && (dist1>=mThFarPoints||dist2>=mThFarPoints)) // MODIFICATION
                    continue;

                const float ratioDist = dist2/dist1;
                const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

                if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                    continue;

                // Triangulation is succesfull
                MapPoint* pMP = new MapPoint(x3D, mpCurrentKeyFrame, mpAtlas->GetCurrentMap());
                if (bPointStereo)
                    countStereo++;

                pMP->AddObservation(mpCurrentKeyFrame,idx1);
                pMP->AddObservation(pKF2,idx2);

                mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
                pKF2->AddMapPoint(pMP,idx2);

                pMP->ComputeDistinctiveDescriptors();

                pMP->UpdateNormalAndDepth();

                mpAtlas->AddMapPoint(pMP);
                mlpRecentAddedMapPoints.push_back(pMP);
            }
        }
    }*/

} //namespace NAV24::OP
