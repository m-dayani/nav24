//
// Created by masoud on 2/23/25.
//

#ifndef NAV24_OBSAPRILTAG_HPP
#define NAV24_OBSAPRILTAG_HPP

#ifdef LIB_APRILTAG_FOUND
#include <apriltag/apriltag.h>
#endif

#include "Point2D.hpp"


namespace NAV24::OB {

    class ObsApriltag : public Point2D {
    public:
#ifdef LIB_APRILTAG_FOUND
        explicit ObsApriltag(apriltag_detection_t*& pDetection);
        ~ObsApriltag();

        void draw(cv::Mat &img) override;
#endif

    private:
#ifdef LIB_APRILTAG_FOUND
        apriltag_detection_t* mpDetection{};
#endif
    };

} // NAV24::OB

#endif //NAV24_OBSAPRILTAG_HPP
