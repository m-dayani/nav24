//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_POINT2D_HPP
#define NAV24_POINT2D_HPP

#include <string>
#include <vector>
#include <opencv2/core.hpp>

#include "Observation.hpp"

namespace NAV24::OB {

    class Point2D : public Observation {
    public:
        Point2D(double _x, double _y) : x(_x), y(_y) {}

        double x, y;
    };

    class BBox : public Observation {
    public:
        BBox() = default;
        BBox(float x_, float y_);
        BBox(float x, float y, float w, float h);

        void updateBboxAndLastPoint(const std::string& coords);
        void updateBboxAndLastPoint(const cv::Rect2f& bbox);
        void updateBboxAndLastPoint(const cv::Point2f& pt2d);

        cv::Point2f getCenter() { return pt2d; }
        cv::Rect2f getBbox() { return bbox; }

        static cv::Point2f find_center(const cv::Rect2f& rect);

    protected:
        cv::Point2f pt2d;
        cv::Rect2f bbox;
    };

} // NAV24::OB

#endif //NAV24_POINT2D_HPP
