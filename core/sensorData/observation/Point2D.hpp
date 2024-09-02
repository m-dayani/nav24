//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_POINT2D_HPP
#define NAV24_POINT2D_HPP

#include <string>
#include <utility>
#include <vector>
#include <opencv2/core.hpp>

#include "Observation.hpp"

namespace NAV24::OB {

    class Point2D : public Observation {
    public:
        Point2D(float _x, float _y) : mbIsDistorted(false), mPoint(_x, _y) {}

        cv::Point2f getPoint() { return mPoint; }
        void setPoint(const cv::Point2f& point) { mPoint = point; }
        cv::Point2f getPointUd() { return mPointUd; }
        void setPointUd(const cv::Point2f& pointUd) { mPointUd = pointUd; }

        bool isDistorted() { return mbIsDistorted; }
        void updateDistorted(bool state) { mbIsDistorted = state; }

    protected:
        bool mbIsDistorted;
        cv::Point2f mPoint;
        cv::Point2f mPointUd;
    };

    class KeyPoint2D : public Point2D {
    public:
        KeyPoint2D(cv::KeyPoint kpt, const cv::Mat& desc) : Point2D(kpt.pt.x, kpt.pt.y),
            mKPt(std::move(kpt)), mDesc(desc.clone()) {}

        [[nodiscard]] const cv::KeyPoint &getKeyPoint() const {
            return mKPt;
        }

        void setKeyPoint(const cv::KeyPoint &kpt) {
            KeyPoint2D::mKPt = kpt;
        }

        [[nodiscard]] const cv::Mat &getDescriptor() const {
            return mDesc;
        }

        void setDescriptor(const cv::Mat &desc) {
            KeyPoint2D::mDesc = desc;
        }

    protected:
        cv::KeyPoint mKPt;
        cv::Mat mDesc;
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
