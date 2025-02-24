//
// Created by masoud on 2/23/25.
//

#include <sstream>
#include <opencv2/opencv.hpp>

#include "ObsMl.hpp"


using namespace std;

namespace NAV24::OB {

    ObsMl::ObsMl(const std::string &label, const float &conf, const cv::Rect &bbox) :
            Point2D(bbox.x, bbox.y), mName(label), mConf(conf), mBbox(bbox) {

        mPoint2 = cv::Point(mPoint.x+mBbox.width, mPoint.y+mBbox.height);
    }

    void ObsMl::draw(cv::Mat &img) {
        Point2D::draw(img);

        ostringstream oss;
        oss << mName << ", " << mConf;
        cv::rectangle(img, mPoint, mPoint2, cv::Scalar(255,255,255), 2);
        cv::putText(img, oss.str(), cv::Point(mPoint.x, mPoint.y-5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255), 1);
    }


} // NAV24::OB