//
// Created by root on 5/18/21.
//

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include "MVSEC_Loader.h"

using namespace std;


namespace NAV24 {


    /*MVSEC_Loader::MVSEC_Loader(const DS_ParamsPtr &pDsParams) : BaseLoader(pDsParams) {

        // Check dataset format
        if (mDsFormat != DS_Params::EV_MVSEC) {
            LOG(ERROR) << "** Initializing MVSEC loader with " << DS_Params::mapDsFormats(mDsFormat) << " data!\n";
            mLoadState = TabularTextDS::BAD_DATA;
            return;
        }

        mPathEvents = pDsParams->getEventsPath();

        this->loadData();
        this->updateLoadState();

        mTopics = {mPathImFile, mPathImBase, mPathImu, mPathEvents, mPathGT};
    }

    MVSEC_Loader::~MVSEC_Loader() = default;

    void MVSEC_Loader::playData() {

        rosbag::View view(mBagData, rosbag::TopicQuery(mTopics));

        ros::Time bag_begin_time = view.getBeginTime();
        ros::Time bag_end_time = view.getEndTime();

        std::cout << "ROS bag time: " << (bag_end_time-bag_begin_time).toSec() << "(s)" << std::endl;

        //int imCnt = 0, camInfoCnt = 0;
        // Load all messages into our stereo dataset
        foreach(rosbag::MessageInstance const m, view) {

            if ((m.getTopic() == mPathImBase || ("/" + m.getTopic() == mPathImBase)) && !mvpImageHooks.empty())
            {
                sensor_msgs::Image::ConstPtr l_img = m.instantiate<sensor_msgs::Image>();
                cv_bridge::CvImagePtr cv_ptr;
                try
                {
                    cv_ptr = cv_bridge::toCvCopy(l_img, sensor_msgs::image_encodings::BGR8);
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return;
                }

                if (cv_ptr && !cv_ptr->image.empty()) {

                    double ts = l_img->header.stamp.toSec();
                    string imFile = l_img->header.frame_id;
                    ImagePtr pImage = make_shared<ImageTs>(cv_ptr->image, ts, imFile);

                    for (ImageHookPtr& pImageHook : mvpImageHooks) {
                        pImageHook->dispatch(pImage);
                    }
                }
            }
//            if (m.getTopic() == mPathImFile || ("/" + m.getTopic() == mPathImFile))
//            {
//                sensor_msgs::CameraInfo::ConstPtr l_info = m.instantiate<sensor_msgs::CameraInfo>();
//                if (l_info != nullptr)
//                    cout << l_info->D.at(0) << endl;
//                //camInfoCnt++;
//            }
            if ((m.getTopic() == mPathEvents || ("/" + m.getTopic() == mPathEvents)) && !mvpEventHooks.empty())
            {
                dvs_msgs::EventArray::ConstPtr evData = m.instantiate<dvs_msgs::EventArray>();
                if (evData != nullptr) {

                    for (const dvs_msgs::Event& ev : evData->events) {

                        EvDataPtr pEvData = make_shared<EventData>(ev.ts.toSec(), ev.x, ev.y, ev.polarity);

                        for (EventHookPtr& pEvHook : mvpEventHooks) {

                            pEvHook->dispatch(pEvData);
                        }
                    }
                }
            }
            if ((m.getTopic() == mPathImu || ("/" + m.getTopic() == mPathImu)) && !mvpIMU_Hooks.empty())
            {
                sensor_msgs::Imu::ConstPtr imuData = m.instantiate<sensor_msgs::Imu>();
                if (imuData != nullptr) {

                    double ts = imuData->header.stamp.toSec();
                    IMU_DataPtr pImuData = make_shared<IMU_Data>(ts, imuData->angular_velocity.x,
                                 imuData->angular_velocity.y, imuData->angular_velocity.z, imuData->linear_acceleration.x,
                                 imuData->linear_acceleration.y, imuData->linear_acceleration.z);

                    for (IMU_HookPtr& pImuHook : mvpIMU_Hooks) {
                        pImuHook->dispatch(pImuData);
                    }
                }
            }
        }
    }

    void MVSEC_Loader::playGroundTruth() {

        rosbag::View gtView(mBagGT, rosbag::TopicQuery(mTopics));

        foreach(rosbag::MessageInstance const m, gtView) {

            if ((m.getTopic() == mPathGT || ("/" + m.getTopic() == mPathGT)) && !mvpGtPoseHooks.empty())
            {
                auto gtData = m.instantiate<geometry_msgs::PoseStamped>();
                if (gtData != nullptr) {

                    double ts = gtData->header.stamp.toSec();
                    PosePtr pGtPose = make_shared<Pose>(ts, gtData->pose.position.x, gtData->pose.position.y,
                                gtData->pose.position.z, gtData->pose.orientation.w, gtData->pose.orientation.x,
                                gtData->pose.orientation.y, gtData->pose.orientation.z);

                    for (PoseHookPtr& pPoseHook : mvpGtPoseHooks) {
                        pPoseHook->dispatch(pGtPose);
                    }
                }
            }
        }
    }

    void MVSEC_Loader::play() {

        // If sequence counter is ok, load bags
        if (!this->checkSequence(mSeqIdx)) {
            LOG(ERROR) << "** MVSEC_Loader::play: Problem loading sequence: " << mSeqIdx << endl;
            return;
        }

        mBagData.open(mvBagFiles[mSeqIdx].first, rosbag::bagmode::Read);
        mBagGT.open(mvBagFiles[mSeqIdx].second, rosbag::BagMode::Read);

        // Run both Data & GT players in 2 threads simultaneously
        auto* ptDataPlayer = new thread(&MVSEC_Loader::playData, this);
        auto* ptGTPlayer = new thread(&MVSEC_Loader::playGroundTruth, this);

        ptDataPlayer->join();
        ptGTPlayer->join();

        delete ptDataPlayer;
        delete ptGTPlayer;

        mBagData.close();
        mBagGT.close();
    }

    void MVSEC_Loader::loadSequence(const string &dsRoot, const string &seqPath, size_t idx) {

        if (mvBagFiles.empty()) {
            mvBagFiles.resize(mSeqCount);
        }
        string bagFileBase = dsRoot + "/" + seqPath + "/" + seqPath;
        mvBagFiles[idx] = make_pair(bagFileBase+"_data.bag", bagFileBase+"_gt.bag");
    }

    bool MVSEC_Loader::checkLoadState() {

        unsigned bagFilesCount = mvBagFiles.size();

        if (bagFilesCount > 0) {
            mSeqCount = bagFilesCount;
            mLoadState = TabularTextDS::GOOD;
            return true;
        }
        else {
            mSeqCount = 0;
            mLoadState = TabularTextDS::BAD_DATA;
            return false;
        }
    }

    bool MVSEC_Loader::checkSequence(unsigned int seq) const {
        return BaseLoader::checkSequence(seq);
    }

    unsigned int MVSEC_Loader::getNumImages() {
        return 0;
    }

    unsigned int MVSEC_Loader::getNumTotalImages() {
        return 0;
    }

    void MVSEC_Loader::getImage(size_t idx, cv::Mat &image, double &ts) {

    }

    void MVSEC_Loader::getImage(size_t idx, cv::Mat &image, double &ts, string &imPath) {

    }

    double MVSEC_Loader::getImageTime(size_t idx) {
        return 0;
    }

    string MVSEC_Loader::getImageFileName(size_t idx) {
        return std::__cxx11::string();
    }

    unsigned int MVSEC_Loader::getNextImu(double ts, vector<IMU_DataPtr> &vpImuData) {
        return 0;
    }

    unsigned int MVSEC_Loader::getNextPoseGT(double ts, vector<PosePtr> &vpPose) {
        return 0;
    }

    void MVSEC_Loader::resetCurrSequence() {

    }*/
}
