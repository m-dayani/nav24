/**
* Most of this file is part of ORB-SLAM3: <https://github.com/UZ-SLAMLab/ORB_SLAM3>.
*
* For licencing information, consult the original project and/or
*   <http://www.gnu.org/licenses/>.
*/

#include "DataConversion.hpp"

#include <iomanip>

using namespace std;

namespace NAV24 {

std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}

g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT)
{
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

    return g2o::SE3Quat(R,t);
}

cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    return toCvMat(eigMat);
}

cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3)
{
    Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
    Eigen::Vector3d eigt = Sim3.translation();
    double s = Sim3.scale();
    return toCvSE3(s*eigR,eigt);
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::MatrixXd &m)
{
    cv::Mat cvMat(m.rows(),m.cols(),CV_32F);
    for(int i=0;i<m.rows();i++)
        for(int j=0; j<m.cols(); j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
            cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

cv::Mat Converter::toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
{
    cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<float>(i,j)=R(i,j);
        }
    }
    for(int i=0;i<3;i++)
    {
        cvMat.at<float>(i,3)=t(i);
    }

    return cvMat.clone();
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Mat &cvVector)
{
    Eigen::Matrix<double,3,1> v;

    if (cvVector.type() == CV_32F) {
        v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);
    }
    else {
        v << cvVector.at<double>(0), cvVector.at<double>(1), cvVector.at<double>(2);
    }
    return v;
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Point3f &cvPoint)
{
    Eigen::Matrix<double,3,1> v;
    v << cvPoint.x, cvPoint.y, cvPoint.z;

    return v;
}

Eigen::Matrix<double,3,3> Converter::toMatrix3d(const cv::Mat &cvMat3)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat3.at<float>(0,0), cvMat3.at<float>(0,1), cvMat3.at<float>(0,2),
         cvMat3.at<float>(1,0), cvMat3.at<float>(1,1), cvMat3.at<float>(1,2),
         cvMat3.at<float>(2,0), cvMat3.at<float>(2,1), cvMat3.at<float>(2,2);

    return M;
}

Eigen::Matrix<double,4,4> Converter::toMatrix4d(const cv::Mat &cvMat4)
{
    Eigen::Matrix<double,4,4> M;

    M << cvMat4.at<float>(0,0), cvMat4.at<float>(0,1), cvMat4.at<float>(0,2), cvMat4.at<float>(0,3),
         cvMat4.at<float>(1,0), cvMat4.at<float>(1,1), cvMat4.at<float>(1,2), cvMat4.at<float>(1,3),
         cvMat4.at<float>(2,0), cvMat4.at<float>(2,1), cvMat4.at<float>(2,2), cvMat4.at<float>(2,3),
         cvMat4.at<float>(3,0), cvMat4.at<float>(3,1), cvMat4.at<float>(3,2), cvMat4.at<float>(3,3);
    return M;
}


std::vector<float> Converter::toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}

cv::Mat Converter::tocvSkewMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

bool Converter::isRotationMatrix(const cv::Mat &R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

    return  cv::norm(I, shouldBeIdentity) < 1e-6;

}

std::vector<float> Converter::toEuler(const cv::Mat &R)
{
    assert(isRotationMatrix(R));
    float sy = sqrt(R.at<float>(0,0) * R.at<float>(0,0) +  R.at<float>(1,0) * R.at<float>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<float>(2,1) , R.at<float>(2,2));
        y = atan2(-R.at<float>(2,0), sy);
        z = atan2(R.at<float>(1,0), R.at<float>(0,0));
    }
    else
    {
        x = atan2(-R.at<float>(1,2), R.at<float>(1,1));
        y = atan2(-R.at<float>(2,0), sy);
        z = 0;
    }

    std::vector<float> v_euler(3);
    v_euler[0] = x;
    v_euler[1] = y;
    v_euler[2] = z;

    return v_euler;
}

cv::Mat Converter::toCvSE3(const cv::Mat &R, const cv::Mat &t) {

    cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);

    R.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
    t.copyTo(Tcw.rowRange(0,3).col(3));

    return Tcw.clone();
}

cv::Mat Converter::toCvSE3Inv(const cv::Mat &CvT) {

    cv::Mat Tinv = cv::Mat::eye(4,4,CV_32F);

    cv::Mat Rt = CvT.rowRange(0,3).colRange(0,3).t();
    Rt.copyTo(Tinv.rowRange(0, 3).colRange(0, 3));

    cv::Mat tinv = - Rt * CvT.rowRange(0,3).col(3);
    tinv.copyTo(Tinv.rowRange(0,3).col(3));

    return Tinv.clone();
}

void Converter::scaleSE3(cv::Mat &Tsc, const cv::Mat &Tscaler) {

    float sc1 = cv::norm(Tscaler.rowRange(0,3).col(3));
    cv::Mat tsc = Tsc.rowRange(0,3).col(3).clone();
    float sc2 = cv::norm(tsc);

    tsc = (sc1/sc2) * tsc;

    tsc.copyTo(Tsc.rowRange(0,3).col(3));
}

// Use this carefully (care to what is relative to what)!
cv::Mat Converter::getTc1c0(const cv::Mat& Tc0w, const cv::Mat& Tc1w) {

    return Tc1w * toCvSE3Inv(Tc0w);
}

cv::Mat Converter::getCurrTcw(const cv::Mat &Tc0w, const cv::Mat &Tcc0) {

    return Tcc0 * Tc0w;
}

// TODO: This seem to be mathematically wrong
g2o::SE3Quat Converter::interpTcw(const g2o::SE3Quat &Tcw0, const g2o::SE3Quat &Tcw1, const double dTs0, const double dTs) {

    g2o::SE3Quat Tcwi;
    const double s = dTs0 / dTs;

    Tcwi.setRotation(Tcw0.rotation().slerp(s, Tcw1.rotation()));
    Tcwi.setTranslation(Tcw0.translation() * (1-s) + Tcw1.translation() * s);

    return Tcwi;
}

/* ================================================================================================================== */
// PoseSE3 to string

std::string Converter::toString(const cv::Mat& pose, const std::string& prefix) {

    std::ostringstream oss;

    oss << prefix << "[";
    for (int i = 0; i < pose.rows; i++) {
        for (int j = 0; j < pose.cols; j++) {
            oss << pose.at<float>(i,j) << ((j == pose.cols-1) ? "" : " ");
        }
        oss << ((i == pose.rows-1) ? "" : "\n"+prefix);
    }
    oss << "]\n";
    return oss.str();
}

std::string Converter::toString(const g2o::SE3Quat& pose, const std::string& prefix) {

    return toString(toCvMat(pose), prefix);
}

std::string Converter::toString(const g2o::Sim3& pose, const std::string& prefix) {

    std::ostringstream oss;
    oss << toString(g2o::SE3Quat(pose.rotation(), pose.translation()), prefix);
    oss << prefix << ", s = " << pose.scale() << endl;
    return oss.str();
}

std::string Converter::toString(const Eigen::MatrixXd& pose, const std::string& prefix) {

    return toString(toCvMat(pose), prefix);
}

std::string Converter::toStringQuatRaw(const cv::Mat &pose, const int prec, const std::string& delim) {

    stringstream oss;

    vector<float> Qcur = Converter::toQuaternion(pose.rowRange(0,3).colRange(0,3));
    cv::Mat trans = pose.rowRange(0,3).col(3);

    oss << setprecision(prec) << trans.at<float>(0,0) << delim << trans.at<float>(1,0) << delim <<
        trans.at<float>(2,0) << delim;

    for (size_t i = 0; i < Qcur.size(); i++) {
        oss << Qcur[i] << ((i == Qcur.size()-1) ? "" : delim);
    }

    return oss.str();
}

std::string Converter::toStringQuatRaw(const cv::Mat &t, const cv::Mat& R, const int prec, const std::string& delim) {

    stringstream oss;

    vector<float> Qcur = Converter::toQuaternion(R);

    oss << setprecision(prec) << t.at<float>(0,0) << delim << t.at<float>(1,0) << delim <<
        t.at<float>(2,0) << delim;

    for (size_t i = 0; i < Qcur.size(); i++) {
        oss << Qcur[i] << ((i == Qcur.size()-1) ? "" : delim);
    }

    return oss.str();
}

std::string Converter::toStringQuat(const cv::Mat &pose, const std::string& prefix) {

    stringstream oss;

    vector<float> Qcur = Converter::toQuaternion(pose.rowRange(0,3).colRange(0,3));
    cv::Mat trans = pose.rowRange(0,3).col(3);

    oss << prefix << "[" << trans.at<float>(0,0) << ", " << trans.at<float>(1,0) << ", " <<
                  trans.at<float>(2,0) << ", ";

    for (size_t i = 0; i < Qcur.size(); i++) {
        oss << Qcur[i] << ((i == Qcur.size()-1) ? "" : ", ");
    }
    oss << "]\n";

    return oss.str();
}

std::string Converter::toStringQuat(const g2o::SE3Quat& pose, const std::string& prefix) {

    ostringstream oss;
    oss << prefix << "[" << pose.translation().x() << ", " << pose.translation().y() << ", "
        << pose.translation().z() << ", ";
    oss << pose.rotation().x() << ", " << pose.rotation().y() << ", " <<
           pose.rotation().z() << ", " << pose.rotation().w() << "]\n";
    return oss.str();
}

std::string Converter::toStringQuat(const g2o::Sim3& pose, const std::string& prefix) {

    ostringstream oss;
    oss << toStringQuat(g2o::SE3Quat(pose.rotation(), pose.translation()), prefix);
    oss << prefix << ", scale = " << pose.scale() << endl;
    return oss.str();
}

    std::string Converter::toString(const std::vector<double> &vData, const std::string& prefix) {

        ostringstream oss{};
        const string& pref = prefix;

        oss << pref << "[";
        for (int i = 0; i < vData.size(); i++) {
            oss << vData[i];
            string sep = (i == vData.size() - 1) ? "" : ", ";
            oss << sep;
        }
        oss << "]";
        return oss.str();
    }

    Eigen::Matrix3d Converter::rotVecToRotMat(const cv::Mat &rvec) {

        float angle_in_radian = (float) cv::norm(rvec);
        cv::Mat axis_cv = rvec / angle_in_radian;
        Eigen::Vector3d axis;
        axis << (float) axis_cv.at<double>(0, 0),
                (float) axis_cv.at<double>(1, 0),
                (float) axis_cv.at<double>(2, 0);
        Eigen::Quaternion<double> q;
        q = Eigen::AngleAxis<double>(angle_in_radian, axis);
        return q.toRotationMatrix();
    }


    Eigen::Vector4d Converter::euler2homo(const Eigen::Vector3d &P_t_euler) {

        Eigen::Vector4d P_t_homo = Eigen::Vector4d::Ones();
        P_t_homo.block<3, 1>(0, 0) = P_t_euler;
        return P_t_homo;
    }

    Eigen::Vector3d Converter::homo2euler(const Eigen::Vector4d &P_t_homo) {

        Eigen::Vector3d P_t_euler = P_t_homo.block<3, 1>(0, 0) / P_t_homo[3];
        return P_t_euler;
    }

} //namespace NAV24
