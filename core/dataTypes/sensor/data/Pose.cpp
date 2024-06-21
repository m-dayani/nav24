//
// Created by root on 5/15/21.
//

#include <utility>
#include <iostream>

#include "Pose.hpp"
#include "DataConversion.hpp"
#include "Point3D.hpp"


using namespace std;

namespace NAV24::TF {

#define DEF_SEP ':'

    Transformation::Transformation(std::string _ref, std::string _target, double _ts, const double &offset) :
        ref(std::move(_ref)), target(std::move(_target)), ts(_ts), offset(offset) {

        key = ref + DEF_SEP + target;
    }

    /* ============================================================================================================== */

    Trans2D::Trans2D(const string &ref_, const string &target_, double ts_, Eigen::Matrix3d T_rt_,
                     const double &offset) : Transformation(ref_, target_, ts_, offset), T_rt(std::move(T_rt_)) {

        T_tr = T_rt.inverse();
    }

    WO::woPtr Trans2D::transform(const WO::woPtr &pWo) {

        WO::woPtr pWoOut = nullptr;
        if (static_pointer_cast<WO::Point3D>(pWo)) {
            auto pt3d = static_pointer_cast<WO::Point3D>(pWo);
            auto Pc = Converter::toVector3d(pt3d->getPoint());
            auto Pw = this->transform(Pc);
            Pw /= Pw[2];
            pWoOut = make_shared<WO::Point3D>(Pw.x(), Pw.y(), Pw.z());
        }
        return pWoOut;
    }

    WO::woPtr Trans2D::transform(const OB::obsPtr &pObs) {
        return NAV24::WO::woPtr();
    }

    OB::obsPtr Trans2D::transformObs(const OB::obsPtr &pObs) {
        return NAV24::OB::obsPtr();
    }

    /* ============================================================================================================== */

    PoseSE3::PoseSE3(const string& _ref, const string& _target, double _ts, Eigen::Matrix4d T_rt_, const double& offset_) :
            Transformation(_ref, _target, _ts, offset_), T_rt(std::move(T_rt_)) {

        T_tr = T_rt.inverse();
    }

    PoseSE3::PoseSE3(const string& _ref, const string& _target, double _ts, const Eigen::Matrix3d &R_rt,
                     const Eigen::Vector3d &t_rt, const double& offset_)  :
            Transformation(_ref, _target, _ts, offset_) {

        T_rt = Eigen::Matrix4d::Identity();
        T_rt.block<3, 3>(0, 0) = R_rt;
        T_rt.block<3, 1>(0, 3) = t_rt;

        T_tr = T_rt.inverse();
    }

    ParamPtr
    PoseSE3::getTransParam(const std::string &ref, const std::string &tar, double t_rt,
                           const PosePtr &pPose, vector<ParamPtr>& vpParamHolder) {

        ParamPtr pRoot = make_shared<Parameter>("0", nullptr, Parameter::NodeType::MAP_NODE);

        ParamPtr pRef = make_shared<ParamType<string>>("ref", pRoot, ref);
        pRef->setType(Parameter::NodeType::STRING);
        ParamPtr pTar = make_shared<ParamType<string>>("target", pRoot, tar);
        pTar->setType(Parameter::NodeType::STRING);
        ParamPtr p_t_rt = make_shared<ParamType<double>>("t_rt", pRoot, t_rt);
        p_t_rt->setType(Parameter::NodeType::REAL);

        cv::Mat T_rt = Converter::toCvMat(pPose->getPose());
        ParamPtr p_T_rt = make_shared<ParamType<cv::Mat>>("T_rt", pRoot, T_rt);
        p_T_rt->setType(Parameter::NodeType::CV_MAT);

        pRoot->insertChild("ref", pRef);
        pRoot->insertChild("target", pTar);
        pRoot->insertChild("t_rt", p_t_rt);
        pRoot->insertChild("T_rt", p_T_rt);

        vpParamHolder.push_back(pRoot);
        vpParamHolder.push_back(pRef);
        vpParamHolder.push_back(pTar);
        vpParamHolder.push_back(p_t_rt);
        vpParamHolder.push_back(p_T_rt);

        return pRoot;
    }

    PosePtr PoseSE3::getTrans(const ParamPtr &pRelParam) {

        auto pRef = find_param<ParamType<string>>("ref", pRelParam);
        string ref = (pRef) ? pRef->getValue() : "";
        auto pTar = find_param<ParamType<string>>("target", pRelParam);
        string target = (pTar) ? pTar->getValue() : "";
        auto p_t_rt = find_param<ParamType<double>>("t_rt", pRelParam);
        double t_rt = (p_t_rt) ? p_t_rt->getValue() : 0.0;
        auto p_T_rt = find_param<ParamType<cv::Mat>>("T_rt", pRelParam);
        cv::Mat T_rt = (p_T_rt) ? p_T_rt->getValue() : cv::Mat();

        PosePtr pTrans = nullptr;
        if (!ref.empty()) {
            auto T_rt_eig = Converter::toMatrix4d(T_rt);
            pTrans = make_shared<PoseSE3>(ref, target, -1, T_rt_eig, t_rt);
        }

        return pTrans;
    }

    std::shared_ptr<PoseSE3> PoseSE3::inverse() {
        return make_shared<PoseSE3>(this->getTarget(), this->getRef(), ts, T_tr);

    }

    WO::woPtr PoseSE3::transform(const WO::woPtr &worldObject) {

        WO::woPtr pWo = nullptr;
        if (static_pointer_cast<WO::Point3D>(worldObject)) {
            auto pt3d = static_pointer_cast<WO::Point3D>(worldObject);
            auto Pc = Converter::toVector3d(pt3d->getPoint());
            Eigen::Vector4d Pcc(Pc.x(), Pc.y(), Pc.z(), 1.0);
            auto Pw = this->transform(Pcc);
            Pw /= Pw[3];
            pWo = make_shared<WO::Point3D>(Pw.x(), Pw.y(), Pw.z());
        }
        return pWo;
    }

    WO::woPtr PoseSE3::transform(const OB::obsPtr &pObs) {
        return NAV24::WO::woPtr();
    }

    OB::obsPtr PoseSE3::transformObs(const OB::obsPtr &pObs) {
        return NAV24::OB::obsPtr();
    }


} // NAV24::TF