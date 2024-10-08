/**
* This file is originally part of ORB-SLAM3: <https://github.com/UZ-SLAMLab/ORB_SLAM3>.
*
* For licencing information, consult the original project and/or
*   <http://www.gnu.org/licenses/>.
*/

#include "Pinhole.hpp"

#include <boost/serialization/export.hpp>

//BOOST_CLASS_EXPORT_IMPLEMENT(ORB_SLAM3::Pinhole)

namespace NAV24 {
//BOOST_CLASS_EXPORT_GUID(Pinhole, "Pinhole")

    long unsigned int GeometricCamera::nNextId=0;

    cv::Point2f Pinhole::project(const cv::Point3f &p3D) {
        return cv::Point2f(mvParameters[0] * p3D.x / p3D.z + mvParameters[2],
                           mvParameters[1] * p3D.y / p3D.z + mvParameters[3]);
    }

    cv::Point2f Pinhole::project(const cv::Mat &m3D) {
        const float* p3D = m3D.ptr<float>();

        return this->project(cv::Point3f(p3D[0],p3D[1],p3D[2]));
    }

    Eigen::Vector2d Pinhole::project(const Eigen::Vector3d &v3D) {
        Eigen::Vector2d res;
        res[0] = mvParameters[0] * v3D[0] / v3D[2] + mvParameters[2];
        res[1] = mvParameters[1] * v3D[1] / v3D[2] + mvParameters[3];

        return res;
    }

    cv::Mat Pinhole::projectMat(const cv::Point3f &p3D) {
        cv::Point2f point = this->project(p3D);
        return (cv::Mat_<float>(2,1) << point.x, point.y);
    }

    float Pinhole::uncertainty2(const Eigen::Matrix<double,2,1> &p2D)
    {
        return 1.0;
    }

    cv::Point3f Pinhole::unproject(const cv::Point2f &p2D) {
        return cv::Point3f((p2D.x - mvParameters[2]) / mvParameters[0], (p2D.y - mvParameters[3]) / mvParameters[1],
                           1.f);
    }

    cv::Mat Pinhole::unprojectMat(const cv::Point2f &p2D){
        cv::Point3f ray = this->unproject(p2D);
        return (cv::Mat_<float>(3,1) << ray.x, ray.y, ray.z);
    }

    cv::Mat Pinhole::projectJac(const cv::Point3f &p3D) {
        cv::Mat Jac(2, 3, CV_32F);
        Jac.at<float>(0, 0) = mvParameters[0] / p3D.z;
        Jac.at<float>(0, 1) = 0.f;
        Jac.at<float>(0, 2) = -mvParameters[0] * p3D.x / (p3D.z * p3D.z);
        Jac.at<float>(1, 0) = 0.f;
        Jac.at<float>(1, 1) = mvParameters[1] / p3D.z;
        Jac.at<float>(1, 2) = -mvParameters[1] * p3D.y / (p3D.z * p3D.z);

        return Jac;
    }

    Eigen::Matrix<double, 2, 3> Pinhole::projectJac(const Eigen::Vector3d &v3D) {
        Eigen::Matrix<double, 2, 3> Jac;
        Jac(0, 0) = mvParameters[0] / v3D[2];
        Jac(0, 1) = 0.f;
        Jac(0, 2) = -mvParameters[0] * v3D[0] / (v3D[2] * v3D[2]);
        Jac(1, 0) = 0.f;
        Jac(1, 1) = mvParameters[1] / v3D[2];
        Jac(1, 2) = -mvParameters[1] * v3D[1] / (v3D[2] * v3D[2]);

        return Jac;
    }

    cv::Mat Pinhole::unprojectJac(const cv::Point2f &p2D) {
        cv::Mat Jac(3, 2, CV_32F);
        Jac.at<float>(0, 0) = 1 / mvParameters[0];
        Jac.at<float>(0, 1) = 0.f;
        Jac.at<float>(1, 0) = 0.f;
        Jac.at<float>(1, 1) = 1 / mvParameters[1];
        Jac.at<float>(2, 0) = 0.f;
        Jac.at<float>(2, 1) = 0.f;

        return Jac;
    }

    cv::Mat Pinhole::toK() {
//        cv::Mat K = (cv::Mat_<float>(3, 3)
//                << mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f);
        return mK_cv.clone();
    }

    bool Pinhole::epipolarConstrain(GeometricCamera* pCamera2,  const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
            const cv::Mat &R12, const cv::Mat &t12, const float sigmaLevel, const float unc) {
        //Compute Fundamental Matrix
        cv::Mat t12x = SkewSymmetricMatrix(t12);
        cv::Mat K1 = this->toK();
        cv::Mat K2 = pCamera2->toK();
        cv::Mat F12 = K1.t().inv()*t12x*R12*K2.inv();

        // Epipolar line in second image l = x1'F12 = [a b c]
        const float a = kp1.pt.x*F12.at<float>(0,0)+kp1.pt.y*F12.at<float>(1,0)+F12.at<float>(2,0);
        const float b = kp1.pt.x*F12.at<float>(0,1)+kp1.pt.y*F12.at<float>(1,1)+F12.at<float>(2,1);
        const float c = kp1.pt.x*F12.at<float>(0,2)+kp1.pt.y*F12.at<float>(1,2)+F12.at<float>(2,2);

        const float num = a*kp2.pt.x+b*kp2.pt.y+c;

        const float den = a*a+b*b;

        if(den==0)
            return false;

        const float dsqr = num*num/den;

        return dsqr<DEF_EC_DIST_COEF*unc;
    }

    std::ostream & operator<<(std::ostream &os, const Pinhole &ph) {
        os << ph.mvParameters[0] << " " << ph.mvParameters[1] << " " << ph.mvParameters[2] << " " << ph.mvParameters[3];
        return os;
    }

    std::istream & operator>>(std::istream &is, Pinhole &ph) {
        float nextParam;
        for(size_t i = 0; i < 4; i++){
            assert(is.good());  //Make sure the input stream is good
            is >> nextParam;
            ph.mvParameters[i] = nextParam;

        }
        return is;
    }

    cv::Mat Pinhole::SkewSymmetricMatrix(const cv::Mat &v)
    {
        return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
                v.at<float>(2),               0,-v.at<float>(0),
                -v.at<float>(1),  v.at<float>(0),              0);
    }
}
