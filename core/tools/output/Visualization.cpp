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

    std::vector<cv::Scalar> Visualization::mColorPallet = {
            cv::Scalar(255, 0, 0),
            cv::Scalar(0, 255, 0),
            cv::Scalar(0, 0, 255),
            cv::Scalar(255, 255, 0),
            cv::Scalar(255, 0, 255),
            cv::Scalar(0, 255, 255),
            cv::Scalar(255, 255, 255),
            cv::Scalar(0, 0, 0),
            cv::Scalar(25, 100, 40),
            cv::Scalar(25, 10, 40),
            cv::Scalar(250, 10, 70),
            cv::Scalar(25, 60, 140),
            cv::Scalar(205, 10, 44),
            cv::Scalar(25, 23, 85)
    };

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
                                   const vector<WO::WoPtr> &vpMapPts) {

        cv::Point2f orig, ax, ay;
        int cInt = 220;

        for (const auto& pPt3d : vpMapPts) {
            auto pt2d = Camera::project(pPt3d, pPose_cw, pCalib);
            auto pt = dynamic_pointer_cast<OB::Point2D>(pt2d);
            cv::Point2f pt_cv = pt->getPoint();

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

    void Visualization::drawKeyPoints(cv::Mat &image, const FramePtr &pFrame, const bool drawDistorted) {

        if (!pFrame) {
            return;
        }

        if (image.empty()) {
            return;
        }

        if (image.channels() == 1) {
            cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
        }

        for (const auto& pObs : pFrame->getObservations()) {
            if (pObs) {
                auto pKeyPoint = dynamic_pointer_cast<OB::Point2D>(pObs);
                if (pKeyPoint) {
                    cv::Point2f kpt = pKeyPoint->getPoint();
                    cv::Point2i pti((int)kpt.x, (int)kpt.y);
                    cv::drawMarker(image, pti, cv::Scalar(0, 0, 255),
                                   cv::MARKER_CROSS, 8);
                    if (drawDistorted && pKeyPoint->isDistorted()) {
                        kpt = pKeyPoint->getPointUd();
                        cv::Point2i pti2 = cv::Point2i((int)kpt.x, (int)kpt.y);
                        cv::drawMarker(image, pti2, cv::Scalar(0, 255, 0),
                                       cv::MARKER_CROSS, 8);
                        cv::line(image, pti, pti2, cv::Scalar(255, 0, 0));
                    }
                }
            }
        }
    }

    void Visualization::drawMatchedTracks(cv::Mat &image, const OB::FtTracksPtr &pTracks) {

        if (image.empty() || !pTracks) {
            return;
        }

        if (image.channels() == 1) {
            cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
        }

        int colorIdx = 0;
        for (const auto& pTrackPair : pTracks->getAllTracks()) {
            const auto& pTrack = pTrackPair.second;
            OB::ObsPtr pLastObs = nullptr;
            for (const auto& pObs : pTrack->getAllFeatures()) {

                if (pLastObs) {
                    if (pObs && dynamic_pointer_cast<OB::Point2D>(pObs)) {
                        const auto& pKpt1 = dynamic_pointer_cast<OB::Point2D>(pLastObs);
                        const auto& pKpt2 = dynamic_pointer_cast<OB::Point2D>(pObs);
                        cv::line(image, pKpt1->getPoint(), pKpt2->getPoint(),
                                 mColorPallet[colorIdx % mColorPallet.size()]);
                    }
                }
                pLastObs = pObs;
            }
            colorIdx++;
        }
    }
} // NAV24
