//
// Created by masoud on 6/12/24.
//

#include "Visualization.hpp"
#include "Camera.hpp"
#include "Point2D.hpp"
#include "Point3D.hpp"

#include <opencv2/imgproc.hpp>


using namespace std;

namespace NAV24 {


    /*void Visualization::drawGrid(cv::Mat& img, const cv::Mat& K, const cv::Mat& D,
                                 const PosePtr& pPose_cw, int nx, int ny, float scale) {

        cv::Mat rtemp, ttemp;
        rtemp.create( 3, 1, CV_32F );
        rtemp.setTo( 0 );
        rtemp.copyTo( ttemp );
        Eigen::Matrix3d H_cw = Eigen::Matrix3d::Identity();
        H_cw.block<3, 2>(0, 0) = pPose_cw->getPose().block<3, 2>(0, 0);
        H_cw.block<3, 1>(0, 2) = pPose_cw->getPose().block<3, 1>(0, 3);
        cv::Point2f orig, ax, ay;
        int cInt = 220;

        for (int i = 0; i < nx; i++) {
            for (int j = 0; j < ny; j++) {

                // unify this project/unproject procedure
                Eigen::Vector3d Pw;
                Pw << static_cast<double>((float)i * scale), static_cast<double>((float)j * scale), 1.0;
                Eigen::Vector3d Pc = H_cw * Pw;
                Pc /= Pc[2];

                vector<cv::Point2d> ptsOut;
                vector<cv::Point3d> ptsTemp{cv::Point3d(Pc.x(), Pc.y(), 1.f)};

                cv::projectPoints(ptsTemp, rtemp, ttemp, K, D, ptsOut);

                cv::circle(img, ptsOut[0], 2, cv::Scalar(cInt, 0, 0), 2);

                if (i == 0 && j == 0) orig = ptsOut[0];
                if (i == 0 && j == ny-1) ay = ptsOut[0];
                if (i == nx-1 && j == 0) ax = ptsOut[0];
            }
        }

        cv::arrowedLine(img, orig, ax, cv::Scalar(0, 0, cInt), 2);
        cv::putText(img, "X", ax, cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    cv::Scalar(0, 0, cInt), 2);
        cv::arrowedLine(img, orig, ay, cv::Scalar(0, cInt, 0), 2);
        cv::putText(img, "Y", ay, cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    cv::Scalar(0, cInt, 0), 2);
    }*/

    void Visualization::projectMap(cv::Mat &img, const PosePtr &pPose_cw, const CalibPtr &pCalib,
                                   const vector<WO::woPtr> &vpMapPts) {

        cv::Point2f orig, ax, ay;
        int cInt = 220;

        for (const auto& pPt3d : vpMapPts) {
            auto pt2d = Camera::project(pPt3d, pPose_cw, pCalib);
            auto pt = dynamic_pointer_cast<OB::Point2D>(pt2d);
            cv::Point2f pt_cv((float)pt->x, (float)pt->y);

            cv::circle(img, pt_cv, 2, cv::Scalar(cInt, 0, 0), 2);

            auto pt3d = dynamic_pointer_cast<WO::Point3D>(pPt3d);
            double i = pt3d->getPoint().x;
            double j = pt3d->getPoint().y;
            if (i == 0 && j == 0) orig = pt_cv;
            if (i == 0 && j != 0) ay = pt_cv;
            if (i != 0 && j == 0) ax = pt_cv;
        }

        cv::arrowedLine(img, orig, ax, cv::Scalar(0, 0, cInt), 2);
        cv::putText(img, "X", ax, cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    cv::Scalar(0, 0, cInt), 2);
        cv::arrowedLine(img, orig, ay, cv::Scalar(0, cInt, 0), 2);
        cv::putText(img, "Y", ay, cv::FONT_HERSHEY_SIMPLEX, 1.0,
                    cv::Scalar(0, cInt, 0), 2);
    }
} // NAV24
