//
// Created by root on 5/15/21.
//

#include <sstream>

#include "Image.hpp"


using namespace std;

namespace NAV24 {

    //Image::Image(const cv::Mat &image, std::string imPath)

    /* -------------------------------------------------------------------------------------------------------------- */

    /*ImageTs::ImageTs(const cv::Mat &image, double ts, const std::string &imPath) :
            Image(image, imPath), mTimeStamp(ts)
    {}*/

    std::string Image::printStr() const {

        ostringstream oss;
        oss << "Path: " << mPath << ", Image: " << mImage.rows << "x" << mImage.cols << "\n";

        return oss.str();
    }

    std::string ImageTs::printStr() const {

        ostringstream oss;
        oss << Image::printStr() << "\b, ts: " << mTimeStamp << "\n";

        return oss.str();
    }
} // NAV24