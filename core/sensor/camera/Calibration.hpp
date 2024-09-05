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


namespace NAV24 {

    class Calibration {
    public:
        explicit Calibration(const ParamPtr& pParams);

        void loadParams(const ParamPtr& pParams);

        virtual OB::ObsPtr undistort(const OB::ObsPtr& pObs);
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

        bool isCalibrated();

    protected:
        std::string distType;
        std::shared_ptr<GeometricCamera> mpCamModel;
    };
    typedef std::shared_ptr<Calibration> CalibPtr;

}   //NAV24

#endif //NAV24_CALIBRATION_HPP
