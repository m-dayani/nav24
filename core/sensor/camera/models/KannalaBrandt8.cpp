/**
* This file is originally part of ORB-SLAM3: <https://github.com/UZ-SLAMLab/ORB_SLAM3>.
*
* For licencing information, consult the original project and/or
*   <http://www.gnu.org/licenses/>.
*/

#include "KannalaBrandt8.hpp"

#include <boost/serialization/export.hpp>
#include <opencv2/calib3d.hpp>

//BOOST_CLASS_EXPORT_IMPLEMENT(ORB_SLAM3::KannalaBrandt8)

namespace NAV24 {
//BOOST_CLASS_EXPORT_GUID(KannalaBrandt8, "KannalaBrandt8")

    KannalaBrandt8::KannalaBrandt8() : GeometricCamera(), precision(KB8_DEF_PRECISION) {

        mvParameters.resize(8);
        mnType = CAM_FISHEYE;
    }

    KannalaBrandt8::KannalaBrandt8(const std::vector<float>& _vParameters) : GeometricCamera(_vParameters),
            precision(KB8_DEF_PRECISION), mvLappingArea(2, 0) {

        assert(mvParameters.size() == 8);
        mnType = CAM_FISHEYE;

        mD_cv = (cv::Mat_<float>(4,1) << mvParameters[4], mvParameters[5], mvParameters[6], mvParameters[7]);
    }

    KannalaBrandt8::KannalaBrandt8(const std::vector<float>& _vParameters, const float _precision) :
            GeometricCamera(_vParameters), mvLappingArea(2,0), precision(_precision) {

        assert(mvParameters.size() == 8);
        mnType = CAM_FISHEYE;

        mD_cv = (cv::Mat_<float>(4,1) << mvParameters[4], mvParameters[5], mvParameters[6], mvParameters[7]);
    }

    KannalaBrandt8::KannalaBrandt8(KannalaBrandt8* pKannala) : GeometricCamera(pKannala->mvParameters),//, pKannala->tvr),
            precision(pKannala->precision), mvLappingArea(2,0) {

        assert(mvParameters.size() == 8);
        mnId=nNextId++;
        mnType = CAM_FISHEYE;

        mK_cv = pKannala->mK_cv.clone();
        mD_cv = pKannala->mD_cv.clone();
        mR = pKannala->mR.clone();
        mP = pKannala->mP.clone();
    }

    cv::Point2f KannalaBrandt8::project(const cv::Point3f &p3D) {

        const float x2_plus_y2 = p3D.x * p3D.x + p3D.y * p3D.y;
        const float theta = atan2f(sqrtf(x2_plus_y2), p3D.z);
        const float psi = atan2f(p3D.y, p3D.x);

        const float theta2 = theta * theta;
        const float theta3 = theta * theta2;
        const float theta5 = theta3 * theta2;
        const float theta7 = theta5 * theta2;
        const float theta9 = theta7 * theta2;
        const float r = theta + mvParameters[4] * theta3 + mvParameters[5] * theta5
                        + mvParameters[6] * theta7 + mvParameters[7] * theta9;

        return {static_cast<float>(mvParameters[0] * r * (float)cos(psi) + mvParameters[2]),
                static_cast<float>(mvParameters[1] * r * (float)sin(psi) + mvParameters[3])};

    }

    cv::Point2f KannalaBrandt8::project(const cv::Mat &m3D) {
        const auto* p3D = m3D.ptr<float>();

        return this->project(cv::Point3f(p3D[0],p3D[1],p3D[2]));
    }

    Eigen::Vector2d KannalaBrandt8::project(const Eigen::Vector3d &v3D) {

        const double x2_plus_y2 = v3D[0] * v3D[0] + v3D[1] * v3D[1];
        const double theta = atan2f(sqrtf((float)x2_plus_y2), (float)v3D[2]);
        const double psi = atan2f((float)v3D[1], (float)v3D[0]);

        const double theta2 = theta * theta;
        const double theta3 = theta * theta2;
        const double theta5 = theta3 * theta2;
        const double theta7 = theta5 * theta2;
        const double theta9 = theta7 * theta2;
        const double r = theta + mvParameters[4] * theta3 + mvParameters[5] * theta5
                        + mvParameters[6] * theta7 + mvParameters[7] * theta9;

        Eigen::Vector2d res;
        res[0] = mvParameters[0] * r * cos(psi) + mvParameters[2];
        res[1] = mvParameters[1] * r * sin(psi) + mvParameters[3];

        return res;

        /*cv::Point2f cvres = this->project(cv::Point3f(v3D[0],v3D[1],v3D[2]));

        Eigen::Vector2d res;
        res[0] = cvres.x;
        res[1] = cvres.y;

        return res;*/
    }

    cv::Mat KannalaBrandt8::projectMat(const cv::Point3f &p3D) {
        cv::Point2f point = this->project(p3D);
        cv::Mat ret = (cv::Mat_<float>(2,1) << point.x, point.y);
        return ret.clone();
    }

    float KannalaBrandt8::uncertainty2(const Eigen::Matrix<double,2,1> &p2D)
    {
        /*Eigen::Matrix<double,2,1> c;
        c << mvParameters[2], mvParameters[3];
        if ((p2D-c).squaredNorm()>57600) // 240*240 (256)
            return 100.f;
        else
            return 1.0f;*/
        return 1.f;
    }

    cv::Mat KannalaBrandt8::unprojectMat(const cv::Point2f &p2D){
        cv::Point3f ray = this->unproject(p2D);
        cv::Mat ret = (cv::Mat_<float>(3,1) << ray.x, ray.y, ray.z);
        return ret.clone();
    }

    cv::Point3f KannalaBrandt8::unproject(const cv::Point2f &p2D) {
        //Use Newthon method to solve for theta with good precision (err ~ e-6)
        cv::Point2f pw((p2D.x - mvParameters[2]) / mvParameters[0], (p2D.y - mvParameters[3]) / mvParameters[1]);
        float scale = 1.f;
        float theta_d = sqrtf(pw.x * pw.x + pw.y * pw.y);
        theta_d = fminf(fmaxf(-CV_PI / 2.f, theta_d), CV_PI / 2.f);

        if (theta_d > 1e-8) {
            //Compensate distortion iteratively
            float theta = theta_d;

            for (int j = 0; j < 10; j++) {
                float theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2, theta8 =
                        theta4 * theta4;
                float k0_theta2 = mvParameters[4] * theta2, k1_theta4 = mvParameters[5] * theta4;
                float k2_theta6 = mvParameters[6] * theta6, k3_theta8 = mvParameters[7] * theta8;
                float theta_fix = (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - theta_d) /
                                  (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8);
                theta = theta - theta_fix;
                if (fabsf(theta_fix) < precision)
                    break;
            }
            //scale = theta - theta_d;
            scale = std::tan(theta) / theta_d;
        }

        return {pw.x * scale, pw.y * scale, 1.f};
    }

    cv::Mat KannalaBrandt8::projectJac(const cv::Point3f &p3D) {
        float x2 = p3D.x * p3D.x, y2 = p3D.y * p3D.y, z2 = p3D.z * p3D.z;
        float r2 = x2 + y2;
        auto r = (float)sqrt(r2);
        float r3 = r2 * r;
        auto theta = (float)atan2(r, p3D.z);

        float theta2 = theta * theta, theta3 = theta2 * theta;
        float theta4 = theta2 * theta2, theta5 = theta4 * theta;
        float theta6 = theta2 * theta4, theta7 = theta6 * theta;
        float theta8 = theta4 * theta4, theta9 = theta8 * theta;

        float f = theta + theta3 * mvParameters[4] + theta5 * mvParameters[5] + theta7 * mvParameters[6] +
                  theta9 * mvParameters[7];
        float fd = 1 + 3 * mvParameters[4] * theta2 + 5 * mvParameters[5] * theta4 + 7 * mvParameters[6] * theta6 +
                   9 * mvParameters[7] * theta8;

        cv::Mat Jac(2, 3, CV_32F);
        Jac.at<float>(0, 0) = mvParameters[0] * (fd * p3D.z * x2 / (r2 * (r2 + z2)) + f * y2 / r3);
        Jac.at<float>(1, 0) =
                mvParameters[1] * (fd * p3D.z * p3D.y * p3D.x / (r2 * (r2 + z2)) - f * p3D.y * p3D.x / r3);

        Jac.at<float>(0, 1) =
                mvParameters[0] * (fd * p3D.z * p3D.y * p3D.x / (r2 * (r2 + z2)) - f * p3D.y * p3D.x / r3);
        Jac.at<float>(1, 1) = mvParameters[1] * (fd * p3D.z * y2 / (r2 * (r2 + z2)) + f * x2 / r3);

        Jac.at<float>(0, 2) = -mvParameters[0] * fd * p3D.x / (r2 + z2);
        Jac.at<float>(1, 2) = -mvParameters[1] * fd * p3D.y / (r2 + z2);

        //std::cout << "CV JAC: " << Jac << std::endl;

        return Jac.clone();
    }

    Eigen::Matrix<double, 2, 3> KannalaBrandt8::projectJac(const Eigen::Vector3d &v3D) {
        double x2 = v3D[0] * v3D[0], y2 = v3D[1] * v3D[1], z2 = v3D[2] * v3D[2];
        double r2 = x2 + y2;
        double r = sqrt(r2);
        double r3 = r2 * r;
        double theta = atan2(r, v3D[2]);

        double theta2 = theta * theta, theta3 = theta2 * theta;
        double theta4 = theta2 * theta2, theta5 = theta4 * theta;
        double theta6 = theta2 * theta4, theta7 = theta6 * theta;
        double theta8 = theta4 * theta4, theta9 = theta8 * theta;

        double f = theta + theta3 * mvParameters[4] + theta5 * mvParameters[5] + theta7 * mvParameters[6] +
                  theta9 * mvParameters[7];
        double fd = 1 + 3 * mvParameters[4] * theta2 + 5 * mvParameters[5] * theta4 + 7 * mvParameters[6] * theta6 +
                   9 * mvParameters[7] * theta8;

        Eigen::Matrix<double, 2, 3> JacGood;
        JacGood(0, 0) = mvParameters[0] * (fd * v3D[2] * x2 / (r2 * (r2 + z2)) + f * y2 / r3);
        JacGood(1, 0) =
                mvParameters[1] * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) - f * v3D[1] * v3D[0] / r3);

        JacGood(0, 1) =
                mvParameters[0] * (fd * v3D[2] * v3D[1] * v3D[0] / (r2 * (r2 + z2)) - f * v3D[1] * v3D[0] / r3);
        JacGood(1, 1) = mvParameters[1] * (fd * v3D[2] * y2 / (r2 * (r2 + z2)) + f * x2 / r3);

        JacGood(0, 2) = -mvParameters[0] * fd * v3D[0] / (r2 + z2);
        JacGood(1, 2) = -mvParameters[1] * fd * v3D[1] / (r2 + z2);

        return JacGood;
    }

    cv::Mat KannalaBrandt8::unprojectJac(const cv::Point2f &p2D) {
        return {};
    }

    std::vector<cv::KeyPoint> KannalaBrandt8::UndistortKeyPoints(const std::vector<cv::KeyPoint> &vKPts) {

        std::vector<cv::KeyPoint> vKeysUn = vKPts;
        std::vector<cv::Point2f> vPts(vKPts.size());

        for(size_t i = 0; i < vKPts.size(); i++) vPts[i] = vKPts[i].pt;

        cv::fisheye::undistortPoints(vPts, vPts, mK_cv, mD_cv, mR, mP);

        for(size_t i = 0; i < vKPts.size(); i++) vKeysUn[i].pt = vPts[i];

        return vKeysUn;
    }

    cv::Mat KannalaBrandt8::toK() {
//        cv::Mat K = (cv::Mat_<float>(3, 3)
//                << mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f);
        return mK_cv.clone();
    }

    bool KannalaBrandt8::epipolarConstrain(GeometricCamera* pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
            const cv::Mat &R12, const cv::Mat &t12, const float sigmaLevel, const float unc) {

        cv::Mat p3D;
        //return this->TriangulateMatches(pCamera2,kp1,kp2,R12,t12,sigmaLevel,unc,p3D) > KB8_DEF_TH_EPC;
        return false;
    }

    /*
    float KannalaBrandt8::TriangulateMatches(GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
            const cv::Mat &R12, const cv::Mat &t12, const float sigmaLevel, const float unc, cv::Mat& p3D) {
        cv::Mat r1 = this->unprojectMat(kp1.pt);
        cv::Mat r2 = pCamera2->unprojectMat(kp2.pt);

        //Check parallax
        cv::Mat r21 = R12*r2;

        const float cosParallaxRays = r1.dot(r21)/(cv::norm(r1)*cv::norm(r21));

        if(cosParallaxRays > KB8_DEF_MIN_PLX){
            return -1;
        }

        //Parallax is good, so we try to triangulate
        cv::Point2f p11,p22;
        const float* pr1 = r1.ptr<float>();
        const float* pr2 = r2.ptr<float>();

        p11.x = pr1[0];
        p11.y = pr1[1];

        p22.x = pr2[0];
        p22.y = pr2[1];

        cv::Mat x3D;
        cv::Mat Tcw1 = (cv::Mat_<float>(3,4) << 1.f,0.f,0.f,0.f,
                                                           0.f,1.f,0.f,0.f,
                                                           0.f,0.f,1.f,0.f);
        cv::Mat Tcw2;
        cv::Mat R21 = R12.t();
        cv::Mat t21 = -R21*t12;
        cv::hconcat(R21,t21,Tcw2);

        Triangulate(p11,p22,Tcw1,Tcw2,x3D);
        cv::Mat x3Dt = x3D.t();

        float z1 = x3D.at<float>(2);
        if(z1 <= 0){
            return -1;
        }

        float z2 = R21.row(2).dot(x3Dt)+t21.at<float>(2);
        if(z2<=0){
            return -1;
        }

        //Check reprojection error
        cv::Point2f uv1 = this->project(x3D);

        float errX1 = uv1.x - kp1.pt.x;
        float errY1 = uv1.y - kp1.pt.y;

        if((errX1*errX1+errY1*errY1)>KB8_DEF_CHISQ_COEF * sigmaLevel){   //Reprojection error is high
            return -1;
        }

        cv::Mat x3D2 = R21 * x3D + t21;
        cv::Point2f uv2 = pCamera2->project(x3D2);

        float errX2 = uv2.x - kp2.pt.x;
        float errY2 = uv2.y - kp2.pt.y;

        if((errX2*errX2+errY2*errY2)>KB8_DEF_CHISQ_COEF * unc){   //Reprojection error is high
            return -1;
        }

        p3D = x3D.clone();

        return z1;
    }
*/
    std::ostream & operator<<(std::ostream &os, const KannalaBrandt8 &kb) {
        os << kb.mvParameters[0] << " " << kb.mvParameters[1] << " " << kb.mvParameters[2] << " " << kb.mvParameters[3] << " "
           << kb.mvParameters[4] << " " << kb.mvParameters[5] << " " << kb.mvParameters[6] << " " << kb.mvParameters[7];
        return os;
    }

    std::istream & operator>>(std::istream &is, KannalaBrandt8 &kb) {
        float nextParam;
        for(size_t i = 0; i < 8; i++){
            assert(is.good());  //Make sure the input stream is good
            is >> nextParam;
            kb.mvParameters[i] = nextParam;

        }
        return is;
    }
/*
    void KannalaBrandt8::Triangulate(const cv::Point2f &p1, const cv::Point2f &p2, const cv::Mat &Tcw1, const cv::Mat &Tcw2, cv::Mat &x3D)
    {
        cv::Mat A(4,4,CV_32F);

        A.row(0) = p1.x*Tcw1.row(2)-Tcw1.row(0);
        A.row(1) = p1.y*Tcw1.row(2)-Tcw1.row(1);
        A.row(2) = p2.x*Tcw2.row(2)-Tcw2.row(0);
        A.row(3) = p2.y*Tcw2.row(2)-Tcw2.row(1);

        cv::Mat u,w,vt;
        cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
        x3D = vt.row(3).t();
        x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
    }
    */
}
