//
// Created by root on 5/15/21.
//

#ifndef NAV24_POSE_H
#define NAV24_POSE_H

#include <memory>

#include "SharedQueue.hpp"


namespace NAV24 {

    struct Pose {

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

    typedef SharedQueue<PosePtr> PoseQueue;
    typedef std::shared_ptr<PoseQueue> PoseQueuePtr;

} // NAV24


#endif //NAV24_POSE_H
