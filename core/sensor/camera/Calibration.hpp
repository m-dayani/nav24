//
// Created by masoud on 2/6/24.
//

#ifndef NAV24_CALIBRATION_HPP
#define NAV24_CALIBRATION_HPP

#include <vector>
#include <memory>

#include <opencv2/core.hpp>
#include <eigen3/Eigen/Dense>

#include "Parameter.hpp"
#include "WorldObject.hpp"
#include "GeometricCamera.h"
#include "Pose.hpp"


namespace NAV24 {

    class Calibration {
    public:
        enum CameraType {
            PINHOLE,
            FISHEYE
        };

        explicit Calibration(const ParamPtr& pParams);

        void loadParams(const ParamPtr& pParams);

        [[nodiscard]] virtual OB::ObsPtr undistort(const OB::ObsPtr& pObs) const;
        virtual std::vector<OB::ObsPtr> undistort(const std::vector<OB::ObsPtr>& vpObs);
        virtual OB::ObsPtr distort(const OB::ObsPtr& pObs);

        virtual WO::WoPtr unproject(const OB::ObsPtr& pt2d);
        virtual OB::ObsPtr project(const WO::WoPtr& pt3d);
        virtual Eigen::Vector2d project(const Eigen::Vector3d& pt3d);
        virtual Eigen::Matrix<double, 2, 3> projectJac(const Eigen::Vector3d& pt3d);

        cv::Mat getK_cv() { return mpCamModel->getK_cv(); }
        Eigen::Matrix3f getK_ei() { return mpCamModel->getK_ei(); }
        cv::Mat getD_cv() { return mpCamModel->getDist(); }

        virtual std::string printStr(const std::string& prefix);

        static ParamPtr getCalibParams(const cv::Mat& K, const cv::Mat& D, const std::string& distType,
                                       std::vector<ParamPtr>& vpParamHolder);

        std::vector<float> computeImageBounds(const cv::Mat &image);

        bool isCalibrated() const;

        static bool isInImage(float x, float y, const cv::Scalar& imageSize);
        static bool isInImage(float x, float y, int imWidth, int imHeight);
        bool isInImage(float x, float y) const;

        static bool isDistorted(const cv::Mat& distCoefs);

        bool isFishEye() const { return mCamType == FISHEYE; }
        bool isPinhole() const { return mCamType == PINHOLE; }


        void generateUndistMaps();
        void generateUndistMapsPinhole();
        void generateUndistMapsFishEye();


        void undistKeyPoints(const std::vector<cv::KeyPoint>& vDistKPts, std::vector<cv::KeyPoint>& vUndistKPts);

        void undistKeyPointsPinhole(const std::vector<cv::KeyPoint>& vDistKPts, std::vector<cv::KeyPoint>& vUndistKPts);
        static void undistKeyPointsPinhole(const std::vector<cv::KeyPoint>& vDistKPts,
                                           std::vector<cv::KeyPoint>& vUndistKPts, const cv::Mat& K, const cv::Mat& distCoefs,
                                           const cv::Mat& R = cv::Mat(), const cv::Mat& P = cv::Mat());

        void undistKeyPointsFishEye(const std::vector<cv::KeyPoint>& vDistKPts, std::vector<cv::KeyPoint>& vUndistKPts);
        static void undistKeyPointsFishEye(const std::vector<cv::KeyPoint>& vDistKPts,
                                           std::vector<cv::KeyPoint>& vUndistKPts, const cv::Mat& K, const cv::Mat& distCoefs,
                                           const cv::Mat& R = cv::Mat(), const cv::Mat& P = cv::Mat());


        void undistPoint(const cv::Point2f& srcPt, cv::Point2f& dstPt);

        void undistPointPinhole(const cv::Point2f& srcPt, cv::Point2f& dstPt);
        static void undistPointPinhole(const cv::Point2f& srcPt, cv::Point2f& dstPt, const cv::Mat& K,
                                       const cv::Mat& distCoefs, const cv::Mat& R = cv::Mat(), const cv::Mat& P = cv::Mat());

        void undistPointFishEye(const cv::Point2f& srcPt, cv::Point2f& dstPt);
        static void undistPointFishEye(const cv::Point2f& srcPt, cv::Point2f& dstPt, const cv::Mat& K,
                                       const cv::Mat& distCoefs, const cv::Mat& R, const cv::Mat& P = cv::Mat());


        // This is not so easy because it requires interpolation
        //void undistKeyPointsMaps(const std::vector<cv::KeyPoint>& vDistKPts, std::vector<cv::KeyPoint>& vUndistKPts);
        //static void undistKeyPointsMaps(const std::vector<cv::KeyPoint>& vDistKPts,
        //        std::vector<cv::KeyPoint>& vUndistKPts, const cv::Mat& mapX, const cv::Mat& mapY);


        // Only use these for integer points!
        void undistPointMaps(const cv::Point2f& srcPt, cv::Point2f& dstPt);
        static void undistPointMaps(const cv::Point2f& srcPt, cv::Point2f& dstPt, const cv::Mat& mapX, const cv::Mat& mapY);

        void undistImageMaps(const cv::Mat& srcImage, cv::Mat& dstImage);
        static void undistImageMaps(const cv::Mat& srcImage, const cv::Mat& mapX, const cv::Mat& mapY, cv::Mat& dstImage);

    protected:
        std::string distType;
        // Camera model (intrinsics)
        std::shared_ptr<GeometricCamera> mpCamModel;
        // Camera to body transformation (extrinsic params)
        PosePtr mpTcb;

        CameraType mCamType;
        int mImWidth, mImHeight;
        cv::Size mImSize;
        cv::Mat mK;
        cv::Mat mDistCoefs;
        cv::Mat mR; // Rectification Matrix
        cv::Mat mP; // Projection Matrix
        cv::Mat mUndistMapX, mUndistMapY;
        cv::Mat mNewCamMatrix;
    };
    typedef std::shared_ptr<Calibration> CalibPtr;
    typedef std::shared_ptr<const Calibration> CalibPtrRO;

}   //NAV24

#endif //NAV24_CALIBRATION_HPP
