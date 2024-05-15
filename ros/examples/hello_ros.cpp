/**
 * Testing Bag Data Loaders:
 * Load different data: Event, Image, IMU, ...
*/

#include <iostream>
#include <chrono>
#include <thread>
//#include <boost/thread.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/package.h>

#include "ParameterServer.hpp"
#include "MVSEC_Loader.h"

using namespace std;
using namespace cv;
using namespace NAV24;

/*class ImageDisplay {
public:

    explicit ImageDisplay(float fps) : mfps(fps), mStop(false) {

        mqpImages = make_shared<ImageQueue>();
    }

    void run() {

        cv::namedWindow("Sample Image", cv::WINDOW_AUTOSIZE);

        while (!mStop) {

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

                cv::cvtColor(imageToShow, imageToShow, CV_GRAY2BGR);

                cv::putText(imageToShow, to_string(imTs), cv::Point2f(10,10),
                            cv::FONT_HERSHEY_COMPLEX_SMALL,((float)imageToShow.cols*1.5f)/720.f,
                            cv::Scalar(0, 180, 0), 1);

                cv::imshow("Sample Image", imageToShow);
                cv::waitKey(static_cast<int>(1000.f / mfps));
                mqpImages->pop();
            }
        }
    }

    ImageQueuePtr mqpImages;
    float mfps;
    bool mStop;
};*/


int main(int argc, char* argv[]) {

    //ros::init(argc, argv, "OG_SLAM");

    if (argc < 2) {
        cout << "Usage: " << argv[0] << " settings.yaml\n";
        return 1;
    }

    string settingsFile = argv[1];
    cout << "Settings File: " << settingsFile << endl;

    // Load Settings
    /*shared_ptr<ParameterServer> pParamServer = make_shared<ParameterServer>(settingsFile);
    cout << pParamServer->getFullStat();

    // Init. Loader
    MVSEC_Loader mvsecLoader(pParamServer->getDS_Params());
    cout << "Output Base Name: " << mvsecLoader.getOutputBasePath() << endl;

    // Get required camera parameters
    CamParamsPtr pCamParams = pParamServer->getCamParams();
    ImageDisplay imageDisplay(pCamParams->fps);

    // Prepare & add image hook to get images
    ImageFilterPtr imFilterGray = make_shared<ImageFilterGray>(pCamParams->mbRGB);
    ImageHookPtr imageHook = make_shared<ImageHookFiltered>(imageDisplay.mqpImages, imFilterGray);

    mvsecLoader.addImageHook(imageHook);

    // Run image display thread
    new std::thread(&ImageDisplay::run, &imageDisplay);

    // Play data loader
    mvsecLoader.play();

    // Wait until all images are displayed
    while (!imageDisplay.mqpImages->empty())
        std::this_thread::sleep_for(chrono::milliseconds(20));

    imageDisplay.mStop = true;
    //mptImageDisplay->join();*/

    return 0;
}
