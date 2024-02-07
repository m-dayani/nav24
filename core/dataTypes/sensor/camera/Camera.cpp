//
// Created by masoud on 2/6/24.
//

#include "Camera.hpp"

#include <glog/logging.h>

using namespace std;

namespace NAV24 {

#define PARAM_KEY_IMG_SIZE "resolution"
#define PARAM_KEY_CAM_FPS "fps"
#define PARAM_KEY_CAM_CALIB "calib"

    void Camera::receive(const NAV24::MsgPtr &msg) {

        Sensor::receive(msg);
    }

    void Camera::loadParams(const NAV24::MsgPtr &msg) {
        Sensor::loadParams(msg);

        if (!msg || !dynamic_pointer_cast<MsgConfig>(msg)) {
            DLOG(WARNING) << "Camera::loadParams, bad config message, abort\n";
            return;
        }

        auto pConfig = dynamic_pointer_cast<MsgConfig>(msg);
        auto pParamCam = pConfig->getConfig();

        if (pParamCam) {
            // Image size
            auto pImgSize = find_param<ParamSeq<int>>(PARAM_KEY_IMG_SIZE, pParamCam);
            if (pImgSize) {
                vector<int> imgSize = pImgSize->getValue();
                if (imgSize.size() >= 2) {
                    imWidth = imgSize[0];
                    imHeight = imgSize[1];
                }
            }

            // Frame per seconds
            auto pFps = find_param<ParamType<double>>(PARAM_KEY_CAM_FPS, pParamCam);
            if (pFps) {
                this->fps = (float) pFps->getValue();
            }

            // Calib
            mpCalib = make_shared<Calibration>(pParamCam->read(PARAM_KEY_CAM_CALIB));
        }
    }

    string Camera::printStr(const string &prefix) const {

        ostringstream oss;

        oss << Sensor::printStr(prefix);
        oss << "Image Size: " << imWidth << " x " << imHeight << "\n";
        oss << "Fps: " << fps << "\n";
        if (mpCalib) {
            oss << mpCalib->printStr();
        }

        return oss.str();
    }

}   //NAV24