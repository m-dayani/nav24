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

    protected:
        float fx{-1.f}, fy{-1.f}, cx{-1.f}, cy{-1.f};
        cv::Mat K_cv;
        Eigen::Matrix3f K_ei;

        std::string distType;
        std::vector<double> D{};
    };
    typedef std::shared_ptr<Calibration> CalibPtr;

}   //NAV24

#endif //NAV24_CALIBRATION_HPP
