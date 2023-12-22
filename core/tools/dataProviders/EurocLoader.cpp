//
// Created by root on 5/16/21.
//

#include "EurocLoader.hpp"

using namespace std;

namespace NAV24 {

    EurocLoader::EurocLoader(const DS_ParamsPtr& pDsParams) : BaseLoader(pDsParams) {

        // Check dataset format
        if (mDsFormat != DS_Params::EUROC) {
            LOG(ERROR) << "** Initializing EuRoC loader with " << DS_Params::mapDsFormats(mDsFormat) << " data!\n";
            mLoadState = TabularTextDS::BAD_DATA;
            return;
        }

        this->loadData();
        this->updateLoadState();
    }

    bool EurocLoader::checkLoadState() {

        unsigned imDsSize = mvpImDs.size();
        unsigned imuDsSize = mvpImuDs.size();
        unsigned gtDsSize = mvpGtDs.size();

        bool cond = imDsSize == imuDsSize && imDsSize == gtDsSize && imDsSize > 0;
        if (cond) {
            mSeqCount = imDsSize;
            mLoadState = TabularTextDS::GOOD;
        }
        else {
            mSeqCount = 0;
            mLoadState = TabularTextDS::BAD_DATA;
        }
        return cond;
    }

    // Deal with timestamp units from the beginning
    void EurocLoader::loadSequence(const string &dsRoot, const string &sqPath, const size_t idx) {

        if (mvpImDs.empty() || mvpImuDs.empty() || mvpGtDs.empty()) {
            mvpImDs.resize(mSeqCount);
            mvpImuDs.resize(mSeqCount);
            mvpGtDs.resize(mSeqCount);
        }

        string seqPath = dsRoot + '/' + sqPath + '/';
        this->mvpImDs[idx] = std::make_unique<ImageDS>(seqPath + mPathImFile, seqPath + mPathImBase, mTsFactor);
        this->mvpImuDs[idx] = std::make_unique<IMU_DS>(seqPath + mPathImu, mbImuGyroFirst, mTsFactor);
        this->mvpGtDs[idx] = std::make_unique<PoseDS>(seqPath + mPathGT, mbGtQwFirst, mbGtPosFirst, mTsFactor);
    }

    void EurocLoader::resetCurrSequence() {

        if (!checkSequence(mSeqIdx))
            return;
        this->mvpImDs[mSeqIdx]->reset();
        this->mvpImuDs[mSeqIdx]->reset();
        this->mvpGtDs[mSeqIdx]->reset();
    }

    unsigned int EurocLoader::getNumImages() {
        if (!checkSequence(mSeqIdx))
            return 0;
        if (this->mvpImDs.empty() || !this->mvpImDs[mSeqIdx])
            return 0;
        return this->mvpImDs[mSeqIdx]->getNumFiles();
    }

    void EurocLoader::getImage(const size_t idx, cv::Mat &image, double &ts) {
        if (!checkSequence(mSeqIdx))
            return;
        this->mvpImDs[mSeqIdx]->getImage(idx, image, ts);
    }

    void EurocLoader::getImage(const size_t idx, cv::Mat &image, double &ts, string& imPath) {
        if (!checkSequence(mSeqIdx))
            return;
        this->mvpImDs[mSeqIdx]->getImage(idx, image, ts, imPath);
    }

    string EurocLoader::getImageFileName(const size_t idx) {
        if (!checkSequence(mSeqIdx))
            return string();
        return this->mvpImDs[mSeqIdx]->getFileName(idx, true);
    }

    double EurocLoader::getImageTime(size_t idx) {
        if (!checkSequence(mSeqIdx))
            return 0.0;
        return this->mvpImDs[mSeqIdx]->getTimeStamp(idx);
    }

    unsigned int EurocLoader::getNumTotalImages() {

        unsigned int numIm = 0;
        if (mSeqTarget < 0) {
            for (auto & mvpImD : mvpImDs) {
                numIm += mvpImD->getNumFiles();
            }
        }
        else {
            numIm = this->getNumImages();
        }
        return numIm;
    }

    unsigned int EurocLoader::getNextImu(const double ts, vector<IMU_DataPtr> &vpImuData) {

        if (!checkSequence(mSeqIdx))
            return 0;
        return this->mvpImuDs[mSeqIdx]->getNextChunk(ts, vpImuData);
    }

    unsigned int EurocLoader::getNextPoseGT(const double ts, vector<PosePtr> &vpPoseGT) {

        if (!checkSequence(mSeqIdx))
            return 0;
        return this->mvpGtDs[mSeqIdx]->getNextChunk(ts, vpPoseGT);
    }

    void EurocLoader::play() {

        if (mLoadState != TabularTextDS::GOOD) {
            LOG(ERROR) << "** Loader::play: Load State is not good, abort\n";
            return;
        }

        uint nImages = this->getNumImages();

        // Loop through images
        for (uint i = 0; i < nImages; i++) {

            // Get Data
            // Image
            double ts = 0.0;
            cv::Mat image;
            string imPath;
            this->getImage(i, image, ts, imPath);
            ImagePtr imData = make_shared<ImageTs>(image, ts, imPath);

            // IMU
            vector<IMU_DataPtr> vpImuData;
            this->getNextImu(ts, vpImuData);

            // GT
            vector<PosePtr> vpPose;
            this->getNextPoseGT(ts, vpPose);

            // Dispatch Data
            for (ImageHookPtr& imHook : mvpImageHooks) {
                imHook->dispatch(imData);
            }

            if (!vpImuData.empty() && vpImuData.back()->ts > ts) {
                DLOG(WARNING) << "EurocLoader::play: Last IMU Ts: " << std::fixed << std::setprecision(6)
                              << vpImuData.back()->ts << " > Image Ts: " << ts << endl;
            }
            else if (vpImuData.empty()) {
                DLOG(WARNING) << "EurocLoader::play: Empty IMU at Image Ts: " << std::fixed << std::setprecision(6)
                                                                              << ts << endl;
            }
            for (const IMU_DataPtr& pImuData : vpImuData) {
                for (IMU_HookPtr& pImuHook : mvpIMU_Hooks) {
                    pImuHook->dispatch(pImuData);
                }
            }

            if (!vpPose.empty() && vpPose.back()->ts > ts) {
                DLOG(WARNING) << "EurocLoader::play: Last GtPose Ts: " << std::fixed << std::setprecision(6)
                              << vpPose.back()->ts << " > Image Ts: " << ts << endl;
            }
            else if (vpPose.empty()) {
                DLOG(WARNING) << "EurocLoader::play: Empty GtPose at Image Ts: " << std::fixed << std::setprecision(6)
                              << ts << endl;
            }
            for (const PosePtr& pPose : vpPose) {
                for (PoseHookPtr& pGtPoseHook : mvpGtPoseHooks) {
                    pGtPoseHook->dispatch(pPose);
                }
            }
        }
    }

} // NAV24
