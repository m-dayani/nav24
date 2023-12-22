//
// Created by root on 5/17/21.
//

#ifndef NAV24_IMAGEFILTER_H
#define NAV24_IMAGEFILTER_H

#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "Image.hpp"


namespace NAV24 {

    class ImageFilter {
    public:
        virtual void filter(ImagePtr& pImage) {}
    };

    typedef std::shared_ptr<ImageFilter> ImageFilterPtr;

    class ImageFilterGray : public ImageFilter {
    public:
        explicit ImageFilterGray(const bool colorOrder) : mbRGB(colorOrder) {}

        void filter(ImagePtr& pImage) override;

    private:
        // Color Order: true: RGB, false: BGR
        const bool mbRGB;
    };

    // class ImageFilterRectify

} // NAV24


#endif //NAV24_IMAGEFILTER_H
