/**
* Most of this file is part of ORB-SLAM3: <https://github.com/UZ-SLAMLab/ORB_SLAM3>.
*
* For licencing information, consult the original project and/or
*   <http://www.gnu.org/licenses/>.
*/


#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include <Eigen/Dense>
//#include "../thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
//#include "../thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"


namespace NAV24 {

class Converter {
public:
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    //static g2o::SE3Quat toSE3Quat(const cv::Mat &CvT);
    //static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

    //static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    //static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    static cv::Mat toCvMat(const Eigen::MatrixXd &m);

    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);
    static cv::Mat tocvSkewMatrix(const cv::Mat &v);

    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);
    static Eigen::Matrix<double,4,4> toMatrix4d(const cv::Mat &cvMat4);
    static std::vector<float> toQuaternion(const cv::Mat &M);
    static Eigen::Matrix3d rotVecToRotMat(const cv::Mat& rvec);

    static bool isRotationMatrix(const cv::Mat &R);
    static std::vector<float> toEuler(const cv::Mat &R);

    static cv::Mat toCvSE3(const cv::Mat& R, const cv::Mat& t);
    static cv::Mat toCvSE3Inv(const cv::Mat& CvT);
    static void scaleSE3(cv::Mat& Tsc, const cv::Mat& Tscaler);
    static cv::Mat getTc1c0(const cv::Mat& Tc0w, const cv::Mat& Tc1w);
    static cv::Mat getCurrTcw(const cv::Mat& Tc0w, const cv::Mat& Tcc0);

    //static g2o::SE3Quat interpTcw(const g2o::SE3Quat& Tcw0, const g2o::SE3Quat& Tcw1, double dTs0, double dTs);

    static Eigen::Vector4d euler2homo(const Eigen::Vector3d& P_t_euler);
    static Eigen::Vector3d homo2euler(const Eigen::Vector4d& P_t_homo);

    /* -------------------------------------------------------------------------------------------------------------- */

    static std::string toString(const std::vector<double>& vData, const std::string& prefix = std::string());
    static std::string toString(const cv::Mat& pose, const std::string& prefix = std::string());
    //static std::string toString(const g2o::SE3Quat& pose, const std::string& prefix = std::string());
    //static std::string toString(const g2o::Sim3& pose, const std::string& prefix = std::string());
    static std::string toString(const Eigen::MatrixXd& pose, const std::string& prefix = std::string());

    static std::string toStringQuatRaw(const cv::Mat& pose, int prec=9, const std::string& delim=" ");
    static std::string toStringQuatRaw(const cv::Mat& t, const cv::Mat& R, int prec=9, const std::string& delim=" ");
    static std::string toStringQuat(const cv::Mat& pose, const std::string& prefix = std::string());
    //static std::string toStringQuat(const g2o::SE3Quat& pose, const std::string& prefix = std::string());
    //static std::string toStringQuat(const g2o::Sim3& pose, const std::string& prefix = std::string());
};

}// namespace ORB_SLAM

#endif // CONVERTER_H
