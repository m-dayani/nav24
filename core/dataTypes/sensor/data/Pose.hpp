//
// Created by root on 5/15/21.
//

#ifndef NAV24_POSE_H
#define NAV24_POSE_H

#include <memory>
#include <eigen3/Eigen/Dense>

#include "SensorData.hpp"
#include "Parameter.hpp"


namespace NAV24 {

    class PoseSE3 : public SensorData {
    public:
        PoseSE3(std::string  _ref, std::string  _target, double _ts, Eigen::Matrix4d  T_rt_, const double& offset = 0);
        PoseSE3(std::string  _ref, std::string  _target, double _ts, const Eigen::Matrix3d& R_rt,
                const Eigen::Vector3d& t_rt, const double& offset = 0);

        Eigen::Vector4d transform(const Eigen::Vector4d& P_t) { return T_rt * P_t; }
        Eigen::Vector4d invTransform(const Eigen::Vector4d& P_r) { return T_tr * P_r; }

        std::shared_ptr<PoseSE3> inverse();

        [[nodiscard]] std::string getKey() const { return key; }
        [[nodiscard]] std::string getRef() const { return ref; }
        [[nodiscard]] std::string getTarget() const { return target; }
        [[nodiscard]] Eigen::Matrix4d getPose() const { return T_rt; }

        static Eigen::Vector4d euler2homo(const Eigen::Vector3d& P_t_euler);
        static Eigen::Vector3d homo2euler(const Eigen::Vector4d& P_t_homo);

        static ParamPtr getTransParam(const std::string& ref, const std::string& tar, double t_rt,
                                      const std::shared_ptr<PoseSE3>& pPose, std::vector<ParamPtr>& vpParamHolder);
        static std::shared_ptr<PoseSE3> getTrans(const ParamPtr& pParam);

    protected:
        std::string ref;
        std::string target;
        std::string key;

        double ts;
        double offset;

        Eigen::Matrix4d T_rt;
        Eigen::Matrix4d T_tr;
    };
    typedef std::shared_ptr<PoseSE3> PosePtr;

    class PoseSim3 : public PoseSE3 {
    public:

    protected:
        double scale{};
    };

} // NAV24


#endif //NAV24_POSE_H
