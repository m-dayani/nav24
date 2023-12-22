//
// Created by root on 5/21/21.
//

#include "Visualization.hpp"

using namespace std;


namespace NAV24 {

    void SimpleImageDisplay::run()  {

        cv::namedWindow(mWindowName, cv::WINDOW_AUTOSIZE);

        while (!mbStop) {

            if (!mqpImages->empty()) {

                ImagePtr pImage = mqpImages->front();

                if (pImage->mImage.empty()) {
                    mqpImages->pop();
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
                mqpImages->pop();
            }
        }

        cv::destroyWindow(mWindowName);
    }

} // NAV24
