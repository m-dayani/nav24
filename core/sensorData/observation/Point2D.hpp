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

        [[nodiscard]] bool isDistorted() const { return mbIsDistorted; }
        void updateDistorted(bool state) { mbIsDistorted = state; }

        static std::vector<cv::Point2f> toCvPoint(const std::vector<ObsPtr>& vpObs);
        static std::vector<cv::Point2f> toCvPointUd(const std::vector<ObsPtr>& vpObs);

        void draw(cv::Mat &img) override;

    protected:
        bool mbIsDistorted;
        cv::Point2f mPoint;
        cv::Point2f mPointUd;
    };

    class KeyPoint2D : public Point2D {
    public:
        KeyPoint2D(cv::KeyPoint kpt, const cv::Mat& desc) : Point2D(kpt.pt.x, kpt.pt.y),
            angle(kpt.angle), octave(kpt.octave), mDesc(desc.clone()) {}

        [[nodiscard]] cv::KeyPoint getKeyPoint() const {
            cv::KeyPoint kpt;
            kpt.pt = mPoint;
            kpt.angle = angle;
            kpt.octave = octave;
            return kpt;
        }

        void setKeyPoint(const cv::KeyPoint &kpt) {
//            KeyPoint2D::mKPt = kpt;
            mPoint = kpt.pt;
            octave = kpt.octave;
            angle = kpt.angle;
        }

        [[nodiscard]] const cv::Mat &getDescriptor() const {
            return mDesc;
        }

        void setDescriptor(const cv::Mat &desc) {
            KeyPoint2D::mDesc = desc;
        }

        [[nodiscard]] cv::KeyPoint getKeyPointUd() const {
            cv::KeyPoint kpt;
            kpt.pt = mPointUd;
            kpt.angle = angle;
            kpt.octave = octave;
            return kpt;
        }

        [[nodiscard]] int getOctave() const { return octave; }
        [[nodiscard]] float getAngle() const { return angle; }

        static std::vector<cv::KeyPoint> toCvKeyPoint(const std::vector<ObsPtr>& vpObs);
        static std::vector<cv::KeyPoint> toCvKeyPointUd(const std::vector<ObsPtr>& vpObs);
    protected:
//        cv::KeyPoint mKPt;
        float angle;
        int octave;
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

        void draw(cv::Mat &img) override;

    protected:
        cv::Point2f pt2d;
        cv::Rect2f bbox;
    };

} // NAV24::OB

#endif //NAV24_POINT2D_HPP
