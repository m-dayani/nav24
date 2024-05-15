//
// Created by masoud on 2/11/24.
//

#include <iostream>
#include <opencv2/calib3d.hpp>
#include <glog/logging.h>
#include <Eigen/Eigen>

#include "BE_CalibCamCv.hpp"
#include "Point3D.hpp"
#include "DataConversion.hpp"

using namespace std;

namespace NAV24::BE {
    void CalibCamCv::solve(const ProblemPtr &problem) {

        if (!problem || !static_pointer_cast<PR_CamCalibCV>(problem)) {
            DLOG(WARNING) << "CalibCamCv::solve, bad problem, abort\n";
        }

        auto prob = static_pointer_cast<PR_CamCalibCV>(problem);

        vector<vector<cv::Point2f>> imgPoints;
        vector<vector<cv::Point3f>> objPoints;
        cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
        vector<cv::Mat> rvecs, tvecs;

        cv::Size imSize = prob->mImgSize;
        auto vFrames = prob->mvpFrames;
        size_t nFrames = vFrames.size();
        imgPoints.resize(nFrames);
        objPoints.resize(nFrames);
        //bool filledPt3d = false;

        auto vpPts3d = prob->mvpWorldObjs;

        for (size_t j = 0; j < nFrames; j++) {
            auto frame = vFrames[j];
            auto vpPts2d = frame->getObservations();

            for (size_t i = 0; i < vpPts2d.size(); i++) {

                auto pt2d = static_pointer_cast<OB::Point2D>(vpPts2d[i]);
                auto pt3d = static_pointer_cast<WO::Point3D>(vpPts3d[i]);

                if (pt2d) imgPoints[j].emplace_back(pt2d->x, pt2d->y);
                if (pt3d) objPoints[j].emplace_back(pt3d->getPoint());
            }
        }

        cv::calibrateCamera(objPoints, imgPoints, imSize, K, distCoeffs, rvecs, tvecs);

        prob->mK = K.clone();
        prob->mDistCoeffs = distCoeffs.clone();

        for (size_t i = 0; i < rvecs.size(); i++) {

            cv::Mat rvec = rvecs[i];
            cv::Mat tvec = tvecs[i];

            auto t_wc = Converter::toVector3d(tvec);
            auto R_wc = Converter::rotVecToRotMat(rvec);
            auto pPose = make_shared<PoseSE3>("w", "c", -1.0, R_wc, t_wc);

            vFrames[i]->setPose(pPose);
        }

        prob->solved = true;
    }
} // NAV24::BE