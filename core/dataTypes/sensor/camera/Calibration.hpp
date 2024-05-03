//
// Created by masoud on 2/6/24.
//

#ifndef NAV24_CALIBRATION_HPP
#define NAV24_CALIBRATION_HPP

#include <vector>
#include <memory>

#include <opencv2/core.hpp>
//#include <Eigen/Dense>
#include <eigen3/Eigen/Core>

#include "Parameter.hpp"


namespace NAV24 {

    class Calibration {
    public:
        explicit Calibration(const ParamPtr& pParams);

        void loadParams(const ParamPtr& pParams);

        std::string printStr(const std::string& prefix = "");

        static ParamPtr getCalibParams(const cv::Mat& K, const cv::Mat& D, const std::string& distType,
                                       std::vector<ParamPtr>& vpParamHolder);

        cv::Point2f undistPoint(const cv::Point2f& distPt);

        cv::Point3f unproject(const cv::Point2f& pt2d);
        cv::Point2f project(const cv::Point3f& pt3d);

        cv::Mat getK_cv() { return K_cv; }
        Eigen::Matrix3f getK_ei() { return K_ei; }
        cv::Mat getD_cv() { return D_cv; }

    protected:
        float fx{-1.f}, fy{-1.f}, cx{-1.f}, cy{-1.f};
        cv::Mat K_cv;
        Eigen::Matrix3f K_ei;

        std::string distType;
        std::vector<double> D{};
        cv::Mat D_cv;

        // Rectification Matrix
        cv::Mat mR;
        // Projection Matrix
        cv::Mat mP;
    };
    typedef std::shared_ptr<Calibration> CalibPtr;

}   //NAV24

#endif //NAV24_CALIBRATION_HPP
