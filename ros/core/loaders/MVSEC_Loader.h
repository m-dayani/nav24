//
// Created by root on 5/18/21.
//

#ifndef NAV24_MVSEC_LOADER_H
#define NAV24_MVSEC_LOADER_H

#include <thread>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
//#include <dvs_msgs/EventArray.h>

//#include "BaseLoader.h"
//#include "EventHook.h"


namespace NAV24 {

    class MVSEC_Loader {// : public BaseLoader {
    public:
        /*explicit MVSEC_Loader(const DS_ParamsPtr& pDsParams);
        //~MVSEC_Loader() override;

        bool checkSequence(unsigned int seq) const override;

        void playData();
        void playGroundTruth();
        void play() override;

        unsigned int getNumImages() override;

        unsigned int getNumTotalImages() override;

        void getImage(size_t idx, cv::Mat &image, double &ts) override;

        void getImage(size_t idx, cv::Mat &image, double &ts, std::string &imPath) override;

        double getImageTime(size_t idx) override;

        std::string getImageFileName(size_t idx) override;

        void resetCurrSequence() override;

        unsigned int getNextImu(double ts, std::vector<IMU_DataPtr> &vpImuData) override;

        unsigned int getNextPoseGT(double ts, std::vector<PosePtr> &vpPose) override;

    protected:
        void loadSequence(const std::string &dsRoot, const std::string &seqPath, size_t idx) override;
        bool checkLoadState() override;

    private:
        std::vector<std::pair<std::string, std::string>> mvBagFiles;

        // This loader uses Bags instead of data stores to source data
        rosbag::Bag mBagData;
        rosbag::Bag mBagGT;

        // All relative data paths are Rosbag topics
        std::string mPathEvents;

        std::vector<EventHookPtr> mvpEventHooks;
        // TODO: CamInfo Types and Hooks

        std::vector<std::string> mTopics;*/
    };

} // NAV24


#endif //NAV24_MVSEC_LOADER_H
