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


namespace NAV24 {

    class Calibration {
    public:
        explicit Calibration(const ParamPtr& pParams);

        void loadParams(const ParamPtr& pParams);

        virtual OB::obsPtr undistort(const OB::obsPtr& pObs);
        virtual OB::obsPtr distort(const OB::obsPtr& pObs);

        virtual WO::woPtr unproject(const OB::obsPtr& pt2d);
        virtual OB::obsPtr project(const WO::woPtr& pt3d);

        cv::Mat getK_cv() { return K_cv; }
        Eigen::Matrix3f getK_ei() { return K_ei; }
        cv::Mat getD_cv() { return D_cv; }

        virtual std::string printStr(const std::string& prefix);

        static ParamPtr getCalibParams(const cv::Mat& K, const cv::Mat& D, const std::string& distType,
                                       std::vector<ParamPtr>& vpParamHolder);

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
