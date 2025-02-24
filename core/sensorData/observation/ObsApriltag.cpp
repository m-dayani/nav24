//
// Created by masoud on 2/23/25.
//

#include <opencv2/opencv.hpp>

#include "ObsApriltag.hpp"

using namespace std;


namespace NAV24::OB {

#ifdef LIB_APRILTAG_FOUND
    ObsApriltag::ObsApriltag(apriltag_detection_t*& pDetection) :
            Point2D(0, 0), mpDetection(pDetection) {

        if (pDetection) {
            double* center = pDetection->c;
            mPoint.x = (float) center[0];
            mPoint.y = (float) center[1];
        }
    }

    ObsApriltag::~ObsApriltag() {

        if (mpDetection) {
            apriltag_detection_destroy(mpDetection);
            mpDetection = nullptr;
        }
    }

    void ObsApriltag::draw(cv::Mat &img) {
        Point2D::draw(img);

        vector<cv::Point> vpts;
        vpts.reserve(4);
        for (auto pt : mpDetection->p) {
            vpts.emplace_back(pt[0], pt[1]);
        }

//        cv::polylines(img, vpts, true, cv::Scalar(255, 0, 0));
        cv::line(img, vpts[0], vpts[1], cv::Scalar(0, 0xff, 0), 2);
        cv::line(img, vpts[0], vpts[3], cv::Scalar(0, 0, 0xff), 2);
        cv::line(img, vpts[1], vpts[2], cv::Scalar(0xff, 0, 0), 2);
        cv::line(img, vpts[2], vpts[3], cv::Scalar(0xff, 0, 0), 2);

        stringstream ss;
        ss << mpDetection->id;
        cv::String text = ss.str();
        int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
        double fontScale = 1.0;
        int baseline;
        cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, 2,
                                            &baseline);
        cv::putText(img, text, cv::Point((int) mpDetection->c[0]-textSize.width/2,
                                         (int) mpDetection->c[1]+textSize.height/2),
                    fontFace, fontScale, cv::Scalar(0xff, 0x99, 0), 2);
    }

#endif

} // NAV24::OB
//