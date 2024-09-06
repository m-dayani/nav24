//
// Created by masoud on 8/30/24.
//

#ifndef NAV24_OP_FTASSOCORBSLAM_HPP
#define NAV24_OP_FTASSOCORBSLAM_HPP

#include "OP_FtAssoc.hpp"
#include "Frame.hpp"

namespace NAV24::OP {

    class FtAssocOrbSlam : public FtAssoc {
    public:
        explicit FtAssocOrbSlam(float nnratio=0.6, bool checkOri=true);

        std::vector<int> matchV(const FramePtr& pFrame1, const FramePtr& pFrame2) override;//, MatchedObs<OB::ObsPtr>& pMatchedObs);
        int match(const FramePtr &pFrame1, const FramePtr &pFrame2, OB::FtTracksPtr &pTracks) override;

        void match(const FramePtr &pFrame1, const FramePtr &pFrame2) override;

        // Computes the Hamming distance between two ORB descriptors
        static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    protected:
        static const int TH_LOW;
        static const int TH_HIGH;
        static const int HISTO_LENGTH;

    protected:
        float RadiusByViewingCos(const float &viewCos);

        void ComputeThreeMaxima(std::vector<int>* histo, int L, int &ind1, int &ind2, int &ind3);

        float mfNNratio;
        bool mbCheckOrientation;
        float windowSize;
    };
} // NAV24::OP

#endif //NAV24_OP_FTASSOCORBSLAM_HPP
