//
// Created by root on 5/15/21.
//

#include <utility>
#include <iostream>

#include "Pose.hpp"
#include "DataConversion.hpp"


using namespace std;

namespace NAV24 {

#define DEF_SEP ':'

    PoseSE3::PoseSE3(string _ref, string _target, double _ts, Eigen::Matrix4d T_rt_, const double& offset_) :
            ref(std::move(_ref)), target(std::move(_target)), ts(_ts), T_rt(std::move(T_rt_)), offset(offset_) {

        key = ref + DEF_SEP + target;

        T_tr = T_rt.inverse();
    }

    PoseSE3::PoseSE3(string _ref, string _target, double _ts, const Eigen::Matrix3d &R_rt,
                     const Eigen::Vector3d &t_rt, const double& offset_)  :
            ref(std::move(_ref)), target(std::move(_target)), ts(_ts), offset(offset_) {

        key = ref + DEF_SEP + target;

        T_rt = Eigen::Matrix4d::Identity();
        T_rt.block<3, 3>(0, 0) = R_rt;
        T_rt.block<3, 1>(0, 3) = t_rt;

        T_tr = T_rt.inverse();
    }

    //inline Eigen::Vector4d PoseSE3::transform(const Eigen::Vector4d &P_t)

    //inline Eigen::Vector4d PoseSE3::invTransform(const Eigen::Vector4d &P_r)

    Eigen::Vector4d PoseSE3::euler2homo(const Eigen::Vector3d &P_t_euler) {

        Eigen::Vector4d P_t_homo = Eigen::Vector4d::Ones();
        P_t_homo.block<3, 1>(0, 0) = P_t_euler;
        return P_t_homo;
    }

    Eigen::Vector3d PoseSE3::homo2euler(const Eigen::Vector4d &P_t_homo) {

        Eigen::Vector3d P_t_euler = P_t_homo.block<3, 1>(0, 0) / P_t_homo[3];
        return P_t_euler;
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
} // NAV24