//
// Created by masoud on 5/3/24.
//

#ifndef NAV24_IMAGEVIEWER_HPP
#define NAV24_IMAGEVIEWER_HPP

#include <string>
#include <memory>
#include <deque>
#include <queue>
#include <map>

#include "Image.hpp"
#include "Output.hpp"
#include "Sensor.hpp"


namespace NAV24 {

    class ImageViewer : public Output {
    public:
        inline static const std::string TOPIC = "ImageViewer";

//        explicit ImageViewer(float fps, std::string  winName = "Window") :
//                mFps(fps), mbStop(false), mWindowName(std::move(winName)), mmqpImages() {}

        explicit ImageViewer(const ChannelPtr&  pChannel);

        void receive(const MsgPtr &msg) override;

    protected:
        void setup(const MsgPtr& msg) override;

        void run() override;
        void requestStop(const std::string& channel) override;

    protected:
        std::map<std::string, std::shared_ptr<std::queue<ImagePtr>>> mmqpImages;
        std::mutex mMtxImgQueue;
        std::mutex mMtxImage;
        float mFps;
    };

} // NAV24

#endif //NAV24_IMAGEVIEWER_HPP
