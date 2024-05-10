//
// Created by root on 5/15/21.
//

#include "Pose.hpp"

#include <utility>
#include <iostream>
#include <Eigen/Eigen>

#include "DataConversion.hpp"


using namespace std;

namespace NAV24 {

    Transformation::Transformation(std::string ref, std::string tar, PosePtr T_rt,
                                   double _ts_rt) : mRef(std::move(ref)), mTarget(std::move(tar)),
                                   T_rt(std::move(T_rt)), ts_rt(_ts_rt), mTransKey() {

        mTransKey.append(mRef).append(":").append(mTarget);
    }

    ParamPtr
    Transformation::getTransParam(const std::string &ref, const std::string &tar, double t_rt,
                                  const PosePtr &pPose, vector<ParamPtr>& vpParamHolder) {

        ParamPtr pRoot = make_shared<Parameter>("0", nullptr, Parameter::NodeType::MAP_NODE);

        ParamPtr pRef = make_shared<ParamType<string>>("ref", pRoot, ref);
        pRef->setType(Parameter::NodeType::STRING);
        ParamPtr pTar = make_shared<ParamType<string>>("target", pRoot, tar);
        pTar->setType(Parameter::NodeType::STRING);
        ParamPtr p_t_rt = make_shared<ParamType<double>>("t_rt", pRoot, t_rt);
        p_t_rt->setType(Parameter::NodeType::REAL);

        // TODO: better handle these transformations
        cv::Mat T_rt = cv::Mat::eye(4, 4, CV_32FC1);
        T_rt.at<float>(0, 3) = pPose->p[0];
        T_rt.at<float>(1, 3) = pPose->p[1];
        T_rt.at<float>(2, 3) = pPose->p[2];
        Eigen::Quaterniond q(pPose->q[0], pPose->q[1], pPose->q[2], pPose->q[3]);
        Eigen::Matrix3d R;
        R = q.toRotationMatrix();
        Converter::toCvMat(R).copyTo(T_rt.rowRange(0, 3).colRange(0, 3));
        //cout << T_rt << endl;
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

    std::shared_ptr<Transformation> Transformation::getTrans(const ParamPtr &pRelParam) {

        auto pRef = find_param<ParamType<string>>("ref", pRelParam);
        string ref = (pRef) ? pRef->getValue() : "";
        auto pTar = find_param<ParamType<string>>("target", pRelParam);
        string target = (pTar) ? pTar->getValue() : "";
        auto p_t_rt = find_param<ParamType<double>>("t_rt", pRelParam);
        double t_rt = (p_t_rt) ? p_t_rt->getValue() : 0.0;
        auto p_T_rt = find_param<ParamType<cv::Mat>>("T_rt", pRelParam);
        cv::Mat T_rt = (p_T_rt) ? p_T_rt->getValue() : cv::Mat();

        TransPtr pTrans = nullptr;

        if (!ref.empty()) {
            // TODO: work more on pose class
            PosePtr pT_rt = make_shared<Pose>();
            vector<float> q = Converter::toQuaternion(T_rt.rowRange(0, 3).colRange(0, 3));
            for (char i = 0; i < 4; i++) pT_rt->q[i] = q[i];
            pT_rt->p[0] = T_rt.at<float>(0, 3);
            pT_rt->p[1] = T_rt.at<float>(1, 3);
            pT_rt->p[2] = T_rt.at<float>(2, 3);

            pTrans = make_shared<Transformation>(ref, target, pT_rt, t_rt);
        }

        return pTrans;
    }
} // NAV24