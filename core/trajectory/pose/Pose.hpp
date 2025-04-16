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

    class Frame;
    typedef std::shared_ptr<Frame> FramePtr;
    typedef std::weak_ptr<Frame> FramePtrW;

    namespace TF {

        class Transformation : public SensorData {
        public:
            explicit Transformation(double ts_, std::string name_ = "trans");

            virtual WO::WoPtr transform(const WO::WoPtr &pWo) = 0;

            virtual WO::WoPtr transform(const OB::ObsPtr &pObs) = 0;

            virtual OB::ObsPtr transformObs(const OB::ObsPtr &pObs) = 0;

            [[nodiscard]] std::string getName() const { return name; }
            void setName(const std::string& name_) { name = name_; }

//            [[nodiscard]] std::string getRef() const { return ref; }

//            [[nodiscard]] std::string getTarget() const { return target; }

//            [[nodiscard]] ulong getId() const { return id; }

            [[nodiscard]] double getTimestamp() const { return ts; }

//            [[nodiscard]] double getOffset() const { return offset; }

        protected:
//            static ulong idCounter;

            // runtime id? -> you can use ts as the id
//            const ulong id;
//            double ts; -> defined in SensorData
            // find poses by name
            std::string name;
            // offset and ref/target names are defined for trajectories (groups of poses)
//            double offset;
        };

        class Trans2D : public Transformation {
        public:
            Trans2D(double ts_, Eigen::Matrix3d T_rt_, const std::string &name = "trans");

            WO::WoPtr transform(const WO::WoPtr &pWo) override;

            WO::WoPtr transform(const OB::ObsPtr &pObs) override;

            OB::ObsPtr transformObs(const OB::ObsPtr &pObs) override;

            Eigen::Vector3d transform(const Eigen::Vector3d &P_t) { return T_rt * P_t; }

        private:
            Eigen::Matrix3d T_rt;
            Eigen::Matrix3d T_tr;
        };

        class PoseSE3 : public Transformation, public SmartObject {
        public:
            PoseSE3(double ts_, Eigen::Matrix4d T_rt_, const std::string &name_ = "trans");

            PoseSE3(double ts_, const Eigen::Matrix3d &R_rt, const Eigen::Vector3d &t_rt,
                    const std::string &name_ = "trans");

            explicit PoseSE3(const std::shared_ptr<PoseSE3>& pPose) : PoseSE3(pPose->ts, pPose->T_rt, pPose->name) {}


            WO::WoPtr transform(const WO::WoPtr &worldObject) override;

            WO::WoPtr transform(const OB::ObsPtr &pObs) override;

            OB::ObsPtr transformObs(const OB::ObsPtr &pObs) override;

            Eigen::Vector4d transform(const Eigen::Vector4d &P_t) { return T_rt * P_t; }
            //Eigen::Vector4d invTransform(const Eigen::Vector4d& P_r) { return T_tr * P_r; }

            std::shared_ptr<PoseSE3> inverse();

            [[nodiscard]] Eigen::Matrix4d getPose() const { return T_rt; }


            static ParamPtr getTransParam(double t_rt,
                                          const std::shared_ptr<PoseSE3> &pPose,
                                          std::vector<ParamPtr> &vpParamHolder);

            static std::shared_ptr<PoseSE3> getTrans(const ParamPtr &pParam);

            FramePtr getFrame() { return mpFrame.lock(); }
            void setFrame(const FramePtr& pFrame) { mpFrame = pFrame; }

            void incLevel() { mLevel++; }
            [[nodiscard]] uint getLevel() const { return mLevel; }

//            void setGlobalScale(const std::shared_ptr<float>& pSc) { mpGlobSc; }

        protected:
            // Absolute pose of target (current pose) wrt reference pose (e.g. initial pose)
            Eigen::Matrix4d T_rt;
            Eigen::Matrix4d T_tr;

            // doubly linked structure
            std::weak_ptr<Transformation> mpPosePrev;
            std::weak_ptr<Transformation> mpPoseNext;

            FramePtrW mpFrame;

            // Pose level (0, 1, ...) -> frame, keyframe, ...
            uint mLevel;

//            std::shared_ptr<float> mpGlobSc;
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
