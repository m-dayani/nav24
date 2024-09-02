/**
* This file is originally part of ORB-SLAM3: <https://github.com/UZ-SLAMLab/ORB_SLAM3>.
*
* For licencing information, consult the original project and/or
*   <http://www.gnu.org/licenses/>.
*/

#ifndef CAMERAMODELS_GEOMETRICCAMERA_H
#define CAMERAMODELS_GEOMETRICCAMERA_H

#include <utility>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/assume_abstract.hpp>

#include <eigen3/Eigen/Geometry>


namespace NAV24 {
    class GeometricCamera {

        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive& ar, const unsigned int version)
        {
            ar & mnId;
            ar & mnType;
            ar & mvParameters;
        }

    public:
        GeometricCamera() : fx(0.f), fy(0.f), cx(0.f), cy(0.f) {

            mnId=nNextId++;

            mK_cv = cv::Mat::eye(3, 3, CV_32F);
            mD_cv = cv::Mat::zeros(4, 1, CV_32F);
            mR = cv::Mat::eye(3, 3, CV_32F);
            mP = mK_cv.clone();
        }

        explicit GeometricCamera(std::vector<float>  _vParameters) : mvParameters(std::move(_vParameters)) {

            assert(mvParameters.size() >= 4);

            mnId=nNextId++;

            fx = mvParameters[0];
            fy = mvParameters[1];
            cx = mvParameters[2];
            cy = mvParameters[3];

            mK_cv = (cv::Mat_<float>(3, 3) << fx, 0.f, cx, 0.f, fy, cy, 0.f, 0.f, 1.f);
            mK_ei << fx, 0.f, cx, 0.f, fy, cy, 0.f, 0.f, 1.f;
            mD_cv = cv::Mat::zeros(4, 1, CV_32F);
            mR = cv::Mat::eye(3, 3, CV_32F);
            mP = mK_cv.clone();
        }

        virtual ~GeometricCamera() = default;

        virtual cv::Point2f project(const cv::Point3f &p3D) = 0;
        virtual cv::Point2f project(const cv::Mat& m3D) = 0;
        virtual Eigen::Vector2d project(const Eigen::Vector3d & v3D) = 0;
        virtual cv::Mat projectMat(const cv::Point3f& p3D) = 0;

        virtual float uncertainty2(const Eigen::Matrix<double,2,1> &p2D) = 0;

        virtual cv::Point3f unproject(const cv::Point2f &p2D) = 0;
        virtual cv::Mat unprojectMat(const cv::Point2f &p2D) = 0;

        virtual cv::Mat projectJac(const cv::Point3f &p3D) = 0;
        virtual Eigen::Matrix<double,2,3> projectJac(const Eigen::Vector3d& v3D) = 0;

        virtual cv::Mat unprojectJac(const cv::Point2f &p2D) = 0;

        virtual std::vector<cv::KeyPoint> UndistortKeyPoints(const std::vector<cv::KeyPoint>& vKPts) = 0;
        //virtual std::vector<OB::ObsPtr> UndistortKeyPoints(const std::vector<OB::ObsPtr>& vKPts) = 0;
        //virtual OB::ObsPtr UndistortKeyPoint(const OB::ObsPtr& kpt) = 0;

        virtual cv::Mat toK() = 0;

        virtual bool epipolarConstrain(GeometricCamera* otherCamera, const cv::KeyPoint& kp1, const cv::KeyPoint& kp2,
                const cv::Mat& R12, const cv::Mat& t12, float sigmaLevel, float unc) = 0;

        float getParameter(const int i){return mvParameters[i];}
        void setParameter(const float p, const size_t i){mvParameters[i] = p;}

        size_t size() { return mvParameters.size(); }

        unsigned int GetId() { return mnId; }

        unsigned int GetType() { return mnType; }

        cv::Mat getK_cv() { return mK_cv; }
        Eigen::Matrix3f getK_ei() { return mK_ei; }
        cv::Mat getDist() { return mD_cv; }

        void setRectificationMat(const cv::Mat& R) { mR = R.clone(); }
        void setProjectionMat(const cv::Mat& P) { mP = P.clone(); }

        const unsigned int CAM_PINHOLE = 0;
        const unsigned int CAM_FISHEYE = 1;

        static long unsigned int nNextId;

    protected:
        std::vector<float> mvParameters;

        unsigned int mnId{};
        unsigned int mnType{};

        float fx{-1.f}, fy{-1.f}, cx{-1.f}, cy{-1.f};
        cv::Mat mK_cv;
        Eigen::Matrix3f mK_ei;

        cv::Mat mD_cv;

        // Rectification Matrix
        cv::Mat mR;
        // Projection Matrix
        cv::Mat mP;
    };
}


#endif //CAMERAMODELS_GEOMETRICCAMERA_H
