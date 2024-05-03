//
// Created by root on 5/15/21.
//

#ifndef NAV24_POSE_H
#define NAV24_POSE_H

#include <memory>

#include "SensorData.hpp"
#include "Parameter.hpp"


namespace NAV24 {

    struct Pose : public SensorData {

        Pose() : ts(0.0), q{}, p{} {}

        Pose(double ts, float px, float py, float pz, float qw, float qx, float qy, float qz) :
                ts(ts), q{qw, qx, qy, qz}, p{px, py, pz} {}

        Pose(double ts, const float _p[3], const float _q[4]) : ts(ts), q{}, p{} {

            for (unsigned char i = 0; i < 3; i++) {
                q[i] = _q[i];
                p[i] = _p[i];
            }
            q[3] = _q[3];
        }

        /*void print() const {
            std::cout << "[ts, px, py, pz, qw, qx, qy, qz]: [" << ts;
            for (unsigned char i; i < 3; i++)
                std::cout << ", " << p[i];
            for (unsigned char i; i < 4; i++)
                std::cout << ", " << q[i];
            std::cout << "]\n";
        }*/

        double ts;
        float q[4];    //q_w, q_x, q_y, q_z
        float p[3];    //px, py, pz
    };
    typedef std::shared_ptr<Pose> PosePtr;

    class Transformation {
    public:
        Transformation(std::string  ref, std::string  tar, PosePtr  T_rt, double _ts_rt);

        const PosePtr& getPose() { return T_rt; }

        static ParamPtr getTransParam(const std::string& ref, const std::string& tar, double t_rt,
                                      const PosePtr& pPose, std::vector<ParamPtr>& vpParamHolder);
    protected:
        PosePtr T_rt;
        std::string mRef;
        std::string mTarget;
        double ts_rt;
    };
    typedef std::shared_ptr<Transformation> TransPtr;

} // NAV24


#endif //NAV24_POSE_H
