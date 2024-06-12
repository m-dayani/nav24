//
// Created by masoud on 6/12/24.
//

#include "Visualization.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


using namespace std;

namespace NAV24 {


    void Visualization::drawGrid(cv::Mat& img, const cv::Mat& K, const cv::Mat& D,
                                 const PosePtr& pPose_cw, int nx, int ny, float scale) {

        cv::Mat rtemp, ttemp;
        rtemp.create( 3, 1, CV_32F );
        rtemp.setTo( 0 );
        rtemp.copyTo( ttemp );
        Eigen::Matrix3d H_cw = Eigen::Matrix3d::Identity();
        H_cw.block<3, 2>(0, 0) = pPose_cw->getPose().block<3, 2>(0, 0);
        H_cw.block<3, 1>(0, 2) = pPose_cw->getPose().block<3, 1>(0, 3);

        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < ny; j++) {

                Eigen::Vector3d Pw;
                Pw << static_cast<double>((float)j * scale), static_cast<double>((float)i * scale), 1.0;
                Eigen::Vector3d Pc = H_cw * Pw;
                Pc /= Pc[2];

                vector<cv::Point2d> ptsOut;
                vector<cv::Point3d> ptsTemp{cv::Point3d(Pc.x(), Pc.y(), 1.f)};

                cv::projectPoints(ptsTemp, rtemp, ttemp, K, D, ptsOut);

                cv::circle(img, ptsOut[0], 2, cv::Scalar(255, 0, 0), 2);
            }
        }
    }
} // NAV24
