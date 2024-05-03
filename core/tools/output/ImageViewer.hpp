//
// Created by masoud on 5/3/24.
//

#ifndef NAV24_IMAGEVIEWER_HPP
#define NAV24_IMAGEVIEWER_HPP

#include <string>
#include <memory>
#include <deque>
#include <queue>

#include "Image.hpp"
#include "Output.hpp"
#include "Sensor.hpp"


namespace NAV24 {

    class ImageViewer : public Output {
    public:
        inline static const std::string TOPIC = "ImageViewer";

//        explicit ImageViewer(float fps, std::string  winName = "Window") :
//                mfps(fps), mbStop(false), mWindowName(std::move(winName)), mqpImages() {}

        explicit ImageViewer(ChannelPtr  pChannel);

        void receive(const MsgPtr &msg) override;

    protected:
        void initialize(const MsgPtr& msg);

        void run();
        void requestStop() { mbStop = true; }
        [[nodiscard]] bool isStopped() const { return mbStop; }


        ChannelPtr mpChannel;

        std::string mName;
        std::shared_ptr<SensorInterface> mpInterface;

        std::queue<ImagePtr> mqpImages;
        float mfps;
        bool mbStop;
        std::string mWindowName;
    };

} // NAV24

#endif //NAV24_IMAGEVIEWER_HPP
