//
// Created by masoud on 8/30/24.
//

#ifndef NAV24_OP_FTDTORBSLAM_HPP
#define NAV24_OP_FTDTORBSLAM_HPP

#include <opencv2/opencv.hpp>

#include "OP_FtDt.hpp"
#include "Frame.hpp"

namespace NAV24::OP {

    class ExtractorNode{
    public:
        ExtractorNode():bNoMore(false){}

        void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

        std::vector<cv::KeyPoint> vKeys;
        cv::Point2i UL, UR, BL, BR;
        std::list<ExtractorNode>::iterator lit;
        bool bNoMore;
    };

    class FtDtOrbSlam : public FtDt {
    public:
        enum {HARRIS_SCORE=0, FAST_SCORE=1 };

        FtDtOrbSlam(int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFAST);

        int detect(FramePtr& pFrame) override;

        int inline GetLevels(){
            return nlevels;}

        float inline GetScaleFactor(){
            return scaleFactor;}

        std::vector<float> inline GetScaleFactors(){
            return mvScaleFactor;
        }

        std::vector<float> inline GetInverseScaleFactors(){
            return mvInvScaleFactor;
        }

        std::vector<float> inline GetScaleSigmaSquares(){
            return mvLevelSigma2;
        }

        std::vector<float> inline GetInverseScaleSigmaSquares(){
            return mvInvLevelSigma2;
        }

        void setNumFeatures(const int nFt) override;

    protected:
        void ComputePyramid(cv::Mat image);
        void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint>>& allKeypoints);
        std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                                    const int &maxX, const int &minY, const int &maxY, const int &nFeatures,
                                                    const int &level) const;

    protected:
        std::vector<cv::Point> pattern;

        double scaleFactor;
        int nlevels;
        int iniThFAST;
        int minThFAST;

        std::vector<int> mnFeaturesPerLevel;

        std::vector<int> umax;

        std::vector<float> mvScaleFactor;
        std::vector<float> mvInvScaleFactor;
        std::vector<float> mvLevelSigma2;
        std::vector<float> mvInvLevelSigma2;

        std::vector<cv::Mat> mvImagePyramid;
    };
} // NAV24::OP

#endif //NAV24_OP_FTDTORBSLAM_HPP
