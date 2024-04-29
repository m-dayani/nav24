//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_POINT3D_HPP
#define NAV24_POINT3D_HPP

#include <opencv2/core.hpp>

#include "WorldObject.hpp"


namespace NAV24::WO {

    class Point3D : public WorldObject {
    public:
        Point3D(double x, double y, double z) : mPoint(x, y, z) {}

        [[nodiscard]] const cv::Point3d &getPoint() const {
            return mPoint;
        }

        void setPoint(const cv::Point3d &point) {
            Point3D::mPoint = point;
        }

    protected:
        cv::Point3d mPoint;
    };

} // NAV24::WO

#endif //NAV24_POINT3D_HPP
