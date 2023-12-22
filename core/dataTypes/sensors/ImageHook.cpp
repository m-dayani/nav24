//
// Created by root on 5/17/21.
//

#include "ImageHook.hpp"

namespace NAV24 {

    inline void ImageHookFiltered::dispatch(ImagePtr &pImage) {

        mpImageFilter->filter(pImage);
        mpImageQueue->push(pImage);
    }

} // NAV24