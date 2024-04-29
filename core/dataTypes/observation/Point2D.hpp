//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_POINT2D_HPP
#define NAV24_POINT2D_HPP

#include "Observation.hpp"

namespace NAV24::OB {

    class Point2D : public Observation {
    public:
        Point2D(double _x, double _y) : x(_x), y(_y) {}

        double x, y;
    };

} // NAV24::OB

#endif //NAV24_POINT2D_HPP
