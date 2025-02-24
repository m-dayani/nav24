//
// Created by masoud on 2/22/25.
//

#include "OP_ObjDetApriltag.hpp"
#include "ObsApriltag.hpp"

using namespace std;

namespace NAV24::OP {

    ObjDetApriltag::ObjDetApriltag(const std::string &family) : mTagFamily(family) {

#ifdef LIB_APRILTAG_FOUND
        // Initialize tag detector with options
        if (family == "tag36h11") {
            mpFamily = tag36h11_create();
        }
        mpDetector = apriltag_detector_create();
        apriltag_detector_add_family(mpDetector, mpFamily);
#endif
    }

    ObjDetApriltag::~ObjDetApriltag() {
#ifdef LIB_APRILTAG_FOUND
        if (mpDetector) {
            apriltag_detector_destroy(mpDetector);
            mpDetector = nullptr;
        }
        if (mpFamily) {
            if (mTagFamily == "tag36h11") {
                tag36h11_destroy(mpFamily);
            }
            mpFamily = nullptr;
        }
#endif
    }

    void ObjDetApriltag::detect(const ImagePtr& pImage, std::vector<OB::ObsPtr> &vpObs) {

        cv::Mat gray = pImage->mImage.clone();
        if (gray.channels() != 1) {
            cv::cvtColor(gray, gray, cv::COLOR_BGR2GRAY);
        }

#ifdef LIB_APRILTAG_FOUND
        // Make an image_u8_t header for the Mat data
        image_u8_t img = {gray.cols, gray.rows, gray.cols, gray.data};

        zarray_t *detections = apriltag_detector_detect(mpDetector, &img);

        int nDetections = zarray_size(detections);
        vpObs.reserve(vpObs.size() + nDetections);
        for (int i = 0; i < nDetections; i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            auto pObs = make_shared<OB::ObsApriltag>(det);
            vpObs.push_back(pObs);
        }
#endif
    }


}


// NAV24::OP


