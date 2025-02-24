//
// Created by masoud on 2/23/25.
//

#ifndef NAV24_OBSML_HPP
#define NAV24_OBSML_HPP

#include "Point2D.hpp"


namespace NAV24::OB {

    class ObsMl : public Point2D {
    public:
        ObsMl(const std::string& label, const float& conf, const cv::Rect& bbox);
        void draw(cv::Mat &img) override;

    private:
        std::string mName;
        float mConf;
        cv::Rect mBbox;
        cv::Point mPoint2;
    };

} // NAV24::OB

#endif //NAV24_OBSML_HPP
