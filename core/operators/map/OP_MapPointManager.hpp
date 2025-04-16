//
// Created by masoud on 2/24/25.
//
// Manage Map Points: Create/Delete/Invalidate/...

#ifndef NAV24_OP_MAPPOINTMANAGER_HPP
#define NAV24_OP_MAPPOINTMANAGER_HPP

#include <Eigen/Eigen>

#include "Operator.hpp"


namespace NAV24::OP {

    class MapPointManager : public Operator {
    public:
        explicit MapPointManager(const ChannelPtr& pChannel) : Operator(pChannel) {}

    private:
        static bool triangulate(Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2, Eigen::Matrix<float,3,4> &Tc1w,
                                Eigen::Matrix<float,3,4> &Tc2w, Eigen::Vector3f &x3D);
        void createMapPoints();
        void mapPointCulling();
//        void updateCovisibilityGraph();

    };

} // NAV24::OP

#endif //NAV24_OP_MAPPOINTMANAGER_HPP
