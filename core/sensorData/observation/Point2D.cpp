//
// Created by masoud on 2/11/24.
//

#include "Point2D.hpp"

using namespace std;

namespace NAV24::OB {
    BBox::BBox(float x_, float y_) {
        this->updateBboxAndLastPoint(cv::Point2f(x_, y_));
    }

    BBox::BBox(float x, float y, float w, float h) {
        this->updateBboxAndLastPoint(cv::Rect2f(x, y, w, h));
    }

    void BBox::updateBboxAndLastPoint(const cv::Rect2f &bbox_) {
        //mMtxYoloDet.lock();
        bbox = bbox_;
        //mMtxYoloDet.unlock();
        //mMtxLastPt.lock();
        pt2d = find_center(bbox_);
        //mMtxLastPt.unlock();
        //mbYoloUpdated = true;
    }

    void BBox::updateBboxAndLastPoint(const string &coords) {

        //cout << coords << endl;
        double val = 0;
        vector<float> vVals;
        vVals.reserve(4);
        istringstream iss{coords};
        while (iss.good()) {
            iss >> val;
            vVals.push_back(static_cast<float>(val));
            //cout << val << ", ";
        }
        //cout << endl;
        if (vVals.size() >= 4) {
            cv::Rect2f bbox(vVals[0], vVals[1], vVals[2], vVals[3]);
            this->updateBboxAndLastPoint(bbox);
        }
        else if (vVals.size() >= 2){
            cv::Point2f point(vVals[0], vVals[1]);
            this->updateBboxAndLastPoint(point);
        }
    }

    void BBox::updateBboxAndLastPoint(const cv::Point2f &pt2d_) {
        //mMtxLastPt.lock();
        pt2d = pt2d_;
        //mMtxLastPt.unlock();
        //mMtxYoloDet.lock();
        auto w = bbox.width;
        auto h = bbox.height;
        bbox = cv::Rect2f(pt2d.x - 0.5f * w, pt2d.y - 0.5f * h, w, h);
        //mMtxYoloDet.unlock();
        //mbYoloUpdated = true;
    }

    cv::Point2f BBox::find_center(const cv::Rect2f &rect) {

        return {rect.x + rect.width * 0.5f, rect.y + rect.height * 0.5f};
    }

    std::vector<cv::Point2f> Point2D::toCvPoint(const vector <ObsPtr> &vpObs) {

        vector<cv::Point2f> vPt2d;
        vPt2d.reserve(vpObs.size());
        for (const auto& pObs : vpObs) {
            if (pObs && dynamic_pointer_cast<Point2D>(pObs)) {
                vPt2d.push_back(dynamic_pointer_cast<Point2D>(pObs)->getPoint());
            }
        }
        return vPt2d;
    }

    std::vector<cv::Point2f> Point2D::toCvPointUd(const vector <ObsPtr> &vpObs) {

        vector<cv::Point2f> vPt2d;
        vPt2d.reserve(vpObs.size());
        for (const auto& pObs : vpObs) {
            if (pObs && dynamic_pointer_cast<Point2D>(pObs)) {
                vPt2d.push_back(dynamic_pointer_cast<Point2D>(pObs)->getPointUd());
            }
        }
        return vPt2d;
    }

    std::vector<cv::KeyPoint> KeyPoint2D::toCvKeyPoint(const vector <ObsPtr> &vpObs) {

        vector<cv::KeyPoint> vPt2d;
        vPt2d.reserve(vpObs.size());
        for (const auto& pObs : vpObs) {
            if (pObs && dynamic_pointer_cast<KeyPoint2D>(pObs)) {
                vPt2d.push_back(dynamic_pointer_cast<KeyPoint2D>(pObs)->getKeyPoint());
            }
        }
        return vPt2d;
    }

    std::vector<cv::KeyPoint> KeyPoint2D::toCvKeyPointUd(const vector <ObsPtr> &vpObs) {

        vector<cv::KeyPoint> vPt2d;
        vPt2d.reserve(vpObs.size());
        for (const auto& pObs : vpObs) {
            if (pObs && dynamic_pointer_cast<KeyPoint2D>(pObs)) {
                vPt2d.push_back(dynamic_pointer_cast<KeyPoint2D>(pObs)->getKeyPointUd());
            }
        }
        return vPt2d;
    }
} // NAV24::OB