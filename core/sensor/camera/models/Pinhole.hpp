/**
* This file is originally part of ORB-SLAM3: <https://github.com/UZ-SLAMLab/ORB_SLAM3>.
*
* For licencing information, consult the original project and/or
*   <http://www.gnu.org/licenses/>.
*/

#ifndef CAMERAMODELS_PINHOLE_H
#define CAMERAMODELS_PINHOLE_H

#include <cassert>
#include <vector>

#include <opencv2/core/core.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/assume_abstract.hpp>

#include "GeometricCamera.h"


namespace NAV24 {

#define DEF_EC_DIST_COEF 3.84f

    class Pinhole : public GeometricCamera {

    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & boost::serialization::base_object<GeometricCamera>(*this);
    }

    public:
        Pinhole() : GeometricCamera() {

            mvParameters.resize(4);
            mnType = CAM_PINHOLE;
        }

        explicit Pinhole(const std::vector<float> _vParameters) : GeometricCamera(_vParameters) {

            assert(mvParameters.size() >= 4);
            mnType = CAM_PINHOLE;
        }

        explicit Pinhole(Pinhole* pPinhole) : GeometricCamera(pPinhole->mvParameters) {

            assert(mvParameters.size() == 4);
            mnType = CAM_PINHOLE;
        }

        ~Pinhole() override{
        }

        cv::Point2f project(const cv::Point3f &p3D);
        cv::Point2f project(const cv::Mat &m3D);
        Eigen::Vector2d project(const Eigen::Vector3d & v3D);
        cv::Mat projectMat(const cv::Point3f& p3D);

        float uncertainty2(const Eigen::Matrix<double,2,1> &p2D);

        cv::Point3f unproject(const cv::Point2f &p2D);
        cv::Mat unprojectMat(const cv::Point2f &p2D);

        cv::Mat projectJac(const cv::Point3f &p3D);
        Eigen::Matrix<double,2,3> projectJac(const Eigen::Vector3d& v3D);

        cv::Mat unprojectJac(const cv::Point2f &p2D);

        std::vector<cv::KeyPoint> UndistortKeyPoints(const std::vector<cv::KeyPoint>& vKPts) override {
            // Pinhole cameras have no distortion
            return vKPts;
        }

        cv::Mat toK();

        bool epipolarConstrain(GeometricCamera* pCamera2, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                const cv::Mat& R12, const cv::Mat& t12, float sigmaLevel, float unc);

        friend std::ostream& operator<<(std::ostream& os, const Pinhole& ph);
        friend std::istream& operator>>(std::istream& os, Pinhole& ph);
    private:
        cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

        //Parameters vector corresponds to
        //      [fx, fy, cx, cy]
    };
}

//BOOST_CLASS_EXPORT_KEY(ORBSLAM2::Pinhole)

#endif //CAMERAMODELS_PINHOLE_H
