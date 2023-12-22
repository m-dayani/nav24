//
// Created by root on 5/17/21.
//

#ifndef NAV24_IMAGEHOOK_H
#define NAV24_IMAGEHOOK_H

#include <memory>
#include <utility>

#include "ImageFilter.hpp"


namespace NAV24 {

    class ImageHook {
    public:
        explicit ImageHook(ImageQueuePtr pImageQueue) : mpImageQueue(std::move(pImageQueue)) {}
        virtual ~ImageHook() = default;

        virtual void dispatch(ImagePtr& pImage) { mpImageQueue->push(pImage); }

    protected:
        ImageQueuePtr mpImageQueue;
    };

    typedef std::shared_ptr<ImageHook> ImageHookPtr;

    class ImageHookFiltered : public ImageHook {
    public:
        ImageHookFiltered(const ImageQueuePtr& pImageQueue, ImageFilterPtr pImFilter) :
                ImageHook(pImageQueue), mpImageFilter(std::move(pImFilter)) {}

        void dispatch(ImagePtr& pImage) override;

    private:
        ImageFilterPtr mpImageFilter;
    };

} // NAV24


#endif //NAV24_IMAGEHOOK_H
