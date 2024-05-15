/**
 * Testing Bag Data Loaders:
 * Load different data: Event, Image, IMU, ...
*/

#include <iostream>
#include <chrono>
#include <thread>

#include <ros/package.h>
#include <ros/init.h>

//#include <glog/logging.h>

#include "ParameterServer.hpp"
#include "MVSEC_Loader.h"
//#include "Visualization.h"
//#include "TabularTextWriter.hpp"

using namespace std;
using namespace cv;
using namespace NAV24;


int main(int argc, char* argv[]) {

    ros::init(argc, argv, "OG_SLAM");
//    google::InitGoogleLogging(argv[0]);

    if (argc < 2) {
        cout << "Usage: " << argv[0] << " settings.yaml\n";
        return 1;
    }

    string settingsFile = argv[1];
    cout << "Settings File: " << settingsFile << endl;

    // Testing parameter server
    // Load Settings
    /*shared_ptr<ParameterServer> pParamServer = make_shared<ParameterServer>(settingsFile);
    cout << pParamServer->getFullStat();

    // Get required camera parameters
    CamParamsPtr pCamParams = pParamServer->getCamParams();
    SimpleImageDisplay imageDisplay(pCamParams->fps);

    // Testing different loaders
    // Init. Loader
    MVSEC_Loader mvsecLoader(pParamServer->getDS_Params());
    cout << mvsecLoader.printLoaderStateStr();

    // Prepare & add image hook to get images
    ImageFilterPtr imFilterGray = make_shared<ImageFilterGray>(pCamParams->mbRGB);
    ImageHookPtr imageHook = make_shared<ImageHookFiltered>(imageDisplay.mqpImages, imFilterGray);
    mvsecLoader.addImageHook(imageHook);

    string imuFile = "../../../data/dummy-imu.txt";
    IMU_Writer imuWriter(imuFile);
    IMU_HookPtr pImuHook = make_shared<IMU_Hook>(imuWriter.mpDataQueue);
    mvsecLoader.addIMU_Hook(pImuHook);

    string gtPoseFile = "../../../data/dummy-gt-pose.txt";
    GtPoseWriter gtPoseWriter(gtPoseFile);
    PoseHookPtr pPoseHook = make_shared<PoseHook>(gtPoseWriter.mpDataQueue);
    mvsecLoader.addGT_PoseHook(pPoseHook);

    // Run image display thread
    auto* mptImDisplay = new std::thread(&SimpleImageDisplay::run, &imageDisplay);
    auto* mptImuWriter = new std::thread(&IMU_Writer::run, &imuWriter);
    auto* mptGtPoseWriter = new std::thread(&GtPoseWriter::run, &gtPoseWriter);

    // Play data loader
    cout << "-- Play ROSBAG Data: It might take a long time to load bag data!\n";
    mvsecLoader.play();

    // Wait until all images are displayed
    while (!imageDisplay.mqpImages->empty())
        std::this_thread::sleep_for(chrono::milliseconds(20));

    imageDisplay.mbStop = true;
    imuWriter.mbStop = true;
    gtPoseWriter.mbStop = true;

    mptImDisplay->join();
    delete mptImDisplay;
    mptImuWriter->join();
    delete mptImuWriter;
    mptGtPoseWriter->join();
    delete mptGtPoseWriter;*/

    return 0;
}
