//
// Created by root on 5/15/21.
//

#ifndef NAV24_POSE_H
#define NAV24_POSE_H

#include <memory>
#include <eigen3/Eigen/Dense>

#include "SensorData.hpp"
#include "Parameter.hpp"
#include "WorldObject.hpp"


namespace NAV24 {
    namespace TF {

        class Transformation : public SensorData {
        public:
            Transformation(std::string _ref, std::string _target, double _ts, const double &offset = 0);

            virtual WO::WoPtr transform(const WO::WoPtr &pWo) = 0;

            virtual WO::WoPtr transform(const OB::ObsPtr &pObs) = 0;

            virtual OB::ObsPtr transformObs(const OB::ObsPtr &pObs) = 0;

            [[nodiscard]] std::string getKey() const { return key; }

            [[nodiscard]] std::string getRef() const { return ref; }

            [[nodiscard]] std::string getTarget() const { return target; }

            [[nodiscard]] double getTimestamp() const { return ts; }

            [[nodiscard]] double getOffset() const { return offset; }

        protected:
            std::string ref;
            std::string target;
            std::string key;

            double ts;
            double offset;
        };

        class Trans2D : public Transformation {
        public:
            Trans2D(const std::string &ref_, const std::string &target_, double ts_, Eigen::Matrix3d T_rt_,
                    const double &offset = 0);

            WO::WoPtr transform(const WO::WoPtr &pWo) override;

            WO::WoPtr transform(const OB::ObsPtr &pObs) override;

            OB::ObsPtr transformObs(const OB::ObsPtr &pObs) override;

            Eigen::Vector3d transform(const Eigen::Vector3d &P_t) { return T_rt * P_t; }

        private:
            Eigen::Matrix3d T_rt;
            Eigen::Matrix3d T_tr;
        };

        class PoseSE3 : public Transformation {
        public:
            PoseSE3(const std::string &ref_, const std::string &target_, double ts_, Eigen::Matrix4d T_rt_,
                    const double &offset = 0);

            PoseSE3(const std::string &ref_, const std::string &target_, double ts_, const Eigen::Matrix3d &R_rt,
                    const Eigen::Vector3d &t_rt, const double &offset = 0);

            WO::WoPtr transform(const WO::WoPtr &worldObject) override;

            WO::WoPtr transform(const OB::ObsPtr &pObs) override;

            OB::ObsPtr transformObs(const OB::ObsPtr &pObs) override;

            Eigen::Vector4d transform(const Eigen::Vector4d &P_t) { return T_rt * P_t; }
            //Eigen::Vector4d invTransform(const Eigen::Vector4d& P_r) { return T_tr * P_r; }

            std::shared_ptr<PoseSE3> inverse();

            [[nodiscard]] Eigen::Matrix4d getPose() const { return T_rt; }


            static ParamPtr getTransParam(const std::string &ref, const std::string &tar, double t_rt,
                                          const std::shared_ptr<PoseSE3> &pPose, std::vector<ParamPtr> &vpParamHolder);

            static std::shared_ptr<PoseSE3> getTrans(const ParamPtr &pParam);

        protected:
            Eigen::Matrix4d T_rt;
            Eigen::Matrix4d T_tr;
        };

        class PoseSim3 : public PoseSE3 {
        public:

        protected:
            double scale{};
        };
    } // TF

    typedef std::shared_ptr<TF::Transformation> TransPtr;
    typedef std::shared_ptr<TF::PoseSE3> PosePtr;
    typedef std::shared_ptr<TF::Trans2D> Tf2dPtr;
} // NAV24


#endif //NAV24_POSE_H
