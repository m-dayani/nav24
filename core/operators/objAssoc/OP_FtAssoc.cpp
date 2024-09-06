//
// Created by masoud on 9/1/24.
//

#include <iostream>

#include <opencv2/highgui.hpp>

#include "OP_FtAssoc.hpp"

using namespace std;

namespace NAV24::OP {

    FtAssocOCV::FtAssocOCV() {

        auto indexParams = cv::makePtr<cv::flann::KDTreeIndexParams>(5);
        auto searchParams = cv::makePtr<cv::flann::SearchParams>(50);
//        mpMatcher = cv::makePtr<cv::FlannBasedMatcher>(indexParams, searchParams);
        mpMatcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
//        mpMatcher = cv::makePtr<cv::BFMatcher>(cv::NORM_HAMMING, true);
    }

    cv::Mat FtAssocOCV::getAllDescriptors(const FramePtr &pFrame) {

        cv::Mat descriptors;
        const auto& vpObs = pFrame->getObservations();
        const size_t nFt = vpObs.size();
        for (size_t i = 0; i < nFt; i++) {
            const auto& pObs = vpObs[i];
            if (pObs && dynamic_pointer_cast<OB::KeyPoint2D>(pObs)) {
                auto desc = dynamic_pointer_cast<OB::KeyPoint2D>(pObs)->getDescriptor();
                if (i == 0) {
                    descriptors = cv::Mat((int)nFt, desc.cols, CV_8U);
                }
                descriptors.row((int)i) = desc.clone();
            }
        }
        return descriptors;
    }

    std::vector<int> FtAssocOCV::matchV(const FramePtr &pFrame1, const FramePtr &pFrame2) {

        cv::Mat descriptors1 = getAllDescriptors(pFrame1);
        cv::Mat descriptors2 = getAllDescriptors(pFrame2);

        std::vector<std::vector<cv::DMatch>> knn_matches;
//        cout << descriptors1 << endl;
        mpMatcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);

        vector<int> matches12(descriptors1.rows, -1);
        for (auto & knn_match : knn_matches) {
            int idx1 = knn_match[0].trainIdx;
            int idx2 = knn_match[1].trainIdx;
            if (idx1 < descriptors1.rows) {
                matches12[idx1] = idx2;
            }
        }

        return matches12;
    }

    int FtAssocOCV::match(const FramePtr &pFrame1, const FramePtr &pFrame2, OB::FtTracksPtr &pTracks) {

        cv::Mat descriptors1 = getAllDescriptors(pFrame1);
        auto vpObs1 = pFrame1->getObservations();
        cv::Mat descriptors2 = getAllDescriptors(pFrame2);
        auto vpObs2 = pFrame2->getObservations();

        std::vector<std::vector<cv::DMatch>> knn_matches;
        mpMatcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);

        const float ratio_thresh = 0.7f;
        int nMch = 0;
//        std::vector<cv::DMatch> good_matches;
        for (auto & knn_match : knn_matches) {
            int idx1 = knn_match[0].queryIdx;
            int idx2 = knn_match[0].trainIdx;
            if (knn_match[0].distance < ratio_thresh * knn_match[1].distance &&
                    idx1 < vpObs1.size() && idx2 < vpObs2.size()) {
                pTracks->addMatch(vpObs1[idx1], vpObs2[idx2]);
//                good_matches.push_back(knn_match[0]);
                nMch++;
            }
        }

        //-- Draw matches
//        cv::Mat img_matches;
//        cv::Mat img1 = dynamic_pointer_cast<FrameImgMono>(pFrame1)->getImage()->mImage;
//        cv::Mat img2 = dynamic_pointer_cast<FrameImgMono>(pFrame2)->getImage()->mImage;
//        auto keypoints1 = getAllKeyPoints(pFrame1);
//        auto keypoints2 = getAllKeyPoints(pFrame2);
//        cv::drawMatches( img1, keypoints1, img2, keypoints2, good_matches, img_matches, cv::Scalar::all(-1),
//                     cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//        //-- Show detected matches
//        cv::imshow("Good Matches", img_matches );

        return nMch;
    }

    std::vector<cv::KeyPoint> FtAssocOCV::getAllKeyPoints(const FramePtr &pFrame) {
        vector<cv::KeyPoint> vkpts;
        const auto& vpObs = pFrame->getObservations();
        const size_t nFt = vpObs.size();
        vkpts.reserve(nFt);
        for (size_t i = 0; i < nFt; i++) {
            const auto& pObs = vpObs[i];
            if (pObs && dynamic_pointer_cast<OB::KeyPoint2D>(pObs)) {
                auto kpt = dynamic_pointer_cast<OB::KeyPoint2D>(pObs)->getKeyPoint();
                vkpts.push_back(kpt);
            }
        }
        return vkpts;
    }


} // NAV::OP