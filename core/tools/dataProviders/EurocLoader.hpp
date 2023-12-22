//
// Created by root on 5/16/21.
//

#ifndef NAV24_EUROCLOADER_H
#define NAV24_EUROCLOADER_H

#include <iomanip>

#include "BaseLoader.hpp"


namespace NAV24 {

    class EurocLoader : public BaseLoader {
    public:
        explicit EurocLoader(const DS_ParamsPtr& pDsParams);
        ~EurocLoader() override = default;

        void play() override;

        void resetCurrSequence() override;

        // Images
        unsigned int getNumImages() override;
        unsigned int getNumTotalImages() override;

        double getImageTime(size_t idx) override;
        std::string getImageFileName(size_t idx) override;

        void getImage(size_t idx, cv::Mat &image, double &ts) override;
        void getImage(size_t idx, cv::Mat &image, double &ts, std::string& imPath) override;

        // IMU
        unsigned int getNextImu(double ts, std::vector<IMU_DataPtr> &vpImuData) override;

        // Ground-truth
        unsigned int getNextPoseGT(double ts, std::vector<PosePtr>& vpPose) override;

    protected:

        void loadSequence(const std::string &dsRoot, const std::string &seqPath, size_t idx) override;

        bool checkLoadState() override;

    protected:
        // Data Stores
        std::vector<ImageDS_UPtr> mvpImDs;
        std::vector<IMU_DS_UPtr> mvpImuDs;
        std::vector<PoseDS_UPtr> mvpGtDs;

    private:

    };

} // NAV24


#endif //NAV24_EUROCLOADER_H
