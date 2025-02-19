//
// Created by root on 5/15/21.
//

#ifndef NAV24_IMAGE_H
#define NAV24_IMAGE_H

#include <string>
#include <vector>
#include <memory>

#include <opencv2/core.hpp>

//#include "SharedQueue.hpp"
#include "SensorData.hpp"
#include "Calibration.hpp"
#include "Point2D.hpp"


namespace NAV24 {

    struct Image : public SensorData {

        Image(const cv::Mat& image, std::string  imPath) : mImage(image.clone()), mPath(std::move(imPath)) {}
        virtual ~Image() = default;

        [[nodiscard]] virtual std::string printStr() const;

        cv::Mat mImage;
        std::string mPath;
    };
    typedef std::shared_ptr<Image> ImagePtr;

    struct ImageTs : public Image {

        ImageTs(const cv::Mat& image, double ts, const std::string& imPath) :
                Image(image, imPath), mTimeStamp(ts) {}

        [[nodiscard]] std::string printStr() const override;

        double mTimeStamp;
    };

    struct ImageTsCalib : public ImageTs {

        ImageTsCalib(const cv::Mat& image, double ts, const std::string& imPath, const CalibPtr pCalib) :
                ImageTs(image, ts, imPath), mpCamera(pCalib) {}

        void hello() {
            auto bili = std::make_shared<OB::Point2D>(3, 4);
            mpCamera->undistort(bili);
        }
        // This must be protected against the content modification
        CalibPtrRO mpCamera;
    };

    //typedef std::shared_ptr<ImageTs> ImageTsPtr;

    //typedef SharedQueue<ImagePtr> ImageQueue;
    //typedef std::shared_ptr<ImageQueue> ImageQueuePtr;

} // NAV24


#endif //NAV24_IMAGE_H
