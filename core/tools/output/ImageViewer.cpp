//
// Created by masoud on 5/3/24.
//

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <glog/logging.h>

#include "ImageViewer.hpp"
#include "System.hpp"


using namespace std;

namespace NAV24 {

    ImageViewer::ImageViewer(const ChannelPtr& pChannel) :
            Output(pChannel), mFps(30), mmqpImages(), mMtxImgQueue(), mMtxImage() {}

    void ImageViewer::requestStop(const std::string &channel) {

        mMtxImgQueue.lock();
        if (mmqpImages.count(channel) > 0) {
            auto imageQueue = mmqpImages[channel];
            // flush the queue
            while(!imageQueue->empty()) {
                //mMtxImage.lock();
                imageQueue->pop();
                //mMtxImage.unlock();
            }
            mmqpImages.erase(channel);
        }

        if (mmqpImages.empty()) {
            mMtxStop.lock();
            mbStop = true;
            mMtxStop.unlock();
        }
        mMtxImgQueue.unlock();
    }

    void ImageViewer::run()  {

        while (true) {
            mMtxStop.lock();
            bool bStop = mbStop;
            mMtxStop.unlock();

            mMtxImgQueue.lock();
            auto imageChannels = mmqpImages;
            mMtxImgQueue.unlock();

            for (auto& imageQueuePair : imageChannels) {

                string winName = imageQueuePair.first;

                mMtxImage.lock();
                ImagePtr pImage;
                auto imageQueue = imageQueuePair.second;
                if (imageQueue && !imageQueue->empty()) {

                    pImage = imageQueue->front();
                    imageQueue->pop();

                    if (!pImage || pImage->mImage.empty()) {
                        this->requestStop(winName);
                    }
                }
                mMtxImage.unlock();

                if (!pImage) {
                    continue;
                }

                double imTs = 0.0;
                if (dynamic_pointer_cast<ImageTs>(pImage)) {
                    auto pImageTs = dynamic_pointer_cast<ImageTs>(pImage);
                    imTs = pImageTs->mTimeStamp;
                }
                cv::Mat imageToShow = pImage->mImage.clone();

                //cv::cvtColor(imageToShow, imageToShow, cv::COLOR_GRAY2RGB);

                cv::putText(imageToShow, std::to_string(imTs), cv::Point2f(10, 10),
                            cv::FONT_HERSHEY_COMPLEX_SMALL, ((float) imageToShow.cols * 1.5f) / 720.f,
                            cv::Scalar(0, 180, 0), 1);

                cv::imshow(winName, imageToShow);
                int keyVal = cv::waitKey(1); //static_cast<int>(1000.f / mFps));

                if (keyVal == 'q') {
                    this->requestStop(winName);
                    bStop = true;
                    auto msgStop = make_shared<Message>(ID_CH_SENSORS, Sensor::TOPIC,
                                                        FCN_SEN_STOP_PLAY, "stop_play");
                    mpChannel->send(msgStop);
                }

                if (bStop) {
                    break;
                }
            }
            if (bStop) {
                break;
            }
        }

        cv::destroyAllWindows();
        //cv::destroyWindow(mWindowName);
    }

    void ImageViewer::receive(const MsgPtr &msg) {
        Output::receive(msg);

        if (msg) {
            string topic = msg->getTopic();
            int catId = msg->getChId();

//            if (catId == ID_CH_OUTPUT || topic == Output::TOPIC || topic == TOPIC) {

            auto msgImage = dynamic_pointer_cast<MsgSensorData>(msg);
            if (msgImage) {
                auto sensorData = msgImage->getData();
                if (sensorData && dynamic_pointer_cast<ImageTs>(sensorData)) {
                    auto pImage = dynamic_pointer_cast<ImageTs>(sensorData);

                    string winName = msg->getMessage();
                    if (winName.empty()) {
                        winName = mName;
                    }

                    if (pImage->mImage.empty()) {
                        DLOG(WARNING) << "ImageViewer::receive, image is empty\n";
                        return;
                    }

                    mMtxImgQueue.lock();
                    if (mmqpImages.count(winName) <= 0) {
                        mmqpImages[winName] = make_shared<queue<ImagePtr>>();
                    }
                    mMtxImage.lock();
                    mmqpImages[winName]->push(pImage);
                    mMtxImage.unlock();
                    mMtxImgQueue.unlock();
                }
            }
//            }
        }
    }

    void ImageViewer::setup(const MsgPtr &msg) {
        Output::setup(msg);
    }

} // NAV24