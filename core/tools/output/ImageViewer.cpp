//
// Created by masoud on 5/3/24.
//

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <glog/logging.h>

#include "ImageViewer.hpp"

using namespace std;

namespace NAV24 {

    ImageViewer::ImageViewer(ChannelPtr pChannel) :
        Output(), mpChannel(std::move(pChannel)), mfps(24), mbStop(false), mWindowName(), mqpImages() {}

    void ImageViewer::run()  {

        cv::namedWindow(mWindowName, cv::WINDOW_AUTOSIZE);

        while (!mbStop) {

            if (!mqpImages.empty()) {

                ImagePtr pImage = mqpImages.front();

                if (pImage->mImage.empty()) {
                    mqpImages.pop();
                    break;
                }

                double imTs = 0.0;
                if (dynamic_cast<ImageTs*>(pImage.get())) {
                    auto* pImageTs = dynamic_cast<ImageTs*>(pImage.get());
                    imTs = pImageTs->mTimeStamp;
                }
                cv::Mat imageToShow = pImage->mImage.clone();

                cv::cvtColor(imageToShow, imageToShow, cv::COLOR_GRAY2RGB);

                cv::putText(imageToShow, std::to_string(imTs), cv::Point2f(10,10),
                            cv::FONT_HERSHEY_COMPLEX_SMALL,((float)imageToShow.cols*1.5f)/720.f,
                            cv::Scalar(0, 180, 0), 1);

                cv::imshow(mWindowName, imageToShow);
                cv::waitKey(static_cast<int>(1000.f / mfps));
                mqpImages.pop();
            }
        }

        cv::destroyWindow(mWindowName);
    }

    void ImageViewer::receive(const MsgPtr &msg) {

        if (msg) {
            string topic = msg->getTopic();

            if (topic == Output::TOPIC || topic == TOPIC) {

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

                        cv::imshow(winName, pImage->mImage);
                        int keyVal = cv::waitKey(1);
                        if (keyVal == 'q') {
                            // todo: don't do this here (this must be done in the players routine)
                            auto msgStop = make_shared<Message>(Sensor::TOPIC,
                                                                "stop_play", FCN_SEN_STOP_PLAY);
                            mpChannel->publish(msgStop);
                        }
                    }
                }
            }
            if (dynamic_pointer_cast<MsgConfig>(msg)) {
                this->initialize(msg);
            }
        }
    }

    void ImageViewer::initialize(const MsgPtr &msg) {

        if (msg && dynamic_pointer_cast<MsgConfig>(msg)) {
            auto msgConfig = dynamic_pointer_cast<MsgConfig>(msg);
            auto pParam = msgConfig->getConfig();
            if (pParam) {
                auto pOutName = find_param<ParamType<string>>("name", pParam);
                mName = (pOutName) ? pOutName->getValue() : "OutputImage0";
                auto pIcType = find_param<ParamType<string>>("interface/type", pParam);
                string icType = (pIcType) ? pIcType->getValue() : "";
                auto pIcTarget = find_param<ParamType<string>>("interface/target", pParam);
                string icTarget = (pIcTarget) ? pIcTarget->getValue() : "";
                auto pIcPort = find_param<ParamType<int>>("interface/port", pParam);
                int icPort = (pIcPort) ? pIcPort->getValue() : 0;

                // todo: make interface type consistent
                mpInterface = make_shared<SensorInterface>(SensorInterface::InterfaceType::DEFAULT,
                                                           icTarget, icPort);
            }
        }
    }

} // NAV24