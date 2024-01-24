//
// Created by root on 5/16/21.
//

#include "BaseLoader.hpp"

#include <memory>

using namespace std;
using namespace boost::filesystem;

namespace NAV24 {

    BaseLoader::BaseLoader(const DS_ParamsPtr &pDsParams) :
            mLoadState(TabularTextDS::LoadState::BAD_PATH), /*mDsFormat(pDsParams->getFormat()),*/ mDsName(pDsParams->getName()),
            mSeqCount(0), mSeqTarget(0), mSeqIdx(0), mnMaxIter(0), mTsFactor(1.0/*pDsParams->getTsFactor()*/),
            mbGtQwFirst(pDsParams->isGtQwFirst()), mbGtPosFirst(pDsParams->isGtPosFirst()),
            mbImuGyroFirst(pDsParams->isImuGyroFirst()){

        // Check important paths
        if (checkDatasetPaths(pDsParams)) {

            // If everything is good, load Data Stores
            this->loadData();
        }
        // Maybe data paths are wrong so check loader integrity
        this->updateLoadState();
    }

    bool BaseLoader::checkDatasetPaths(const DS_ParamsPtr &pDsParams) {

        // Check DS root path
        mPathDsRoot = pDsParams->getDatasetRoot();
        if (!TabularTextDS::checkDirectory(mPathDsRoot)) {
            mLoadState = TabularTextDS::BAD_PATH;
            return false;
        }

        // Assign relative sequence data paths
        mPathImFile = pDsParams->getImageFilePath();
        mPathImBase = pDsParams->getImageBasePath();
        mPathImu = pDsParams->getIMU_Path();
        mPathGT = pDsParams->getGroundTruthPosePath();

        // Resolve and check sequence paths
        if (!this->resolveSeqInfo(pDsParams)) {
            mLoadState = TabularTextDS::BAD_PATH;
            return false;
        }

        // Resolve output trajectory path
        this->resolveOutputBasePath(pDsParams);

        mLoadState = TabularTextDS::READY;
        return true;
    }

    /**
     * This will set 3 internal class variables:
     *  mSeqTarget, mSeqCount, mSeqNames
     * @param pDsParams
     * @return
     */
    bool BaseLoader::resolveSeqInfo(const DS_ParamsPtr &pDsParams) {

        mSeqTarget = pDsParams->getSeqTarget();

        vector<string> seqNames = pDsParams->getSeqNames();
        unsigned int seqCount = seqNames.size();

        if (seqCount) {
            // Check target sequence path(s) exist.
            if (mSeqTarget >= 0) {
                if (mSeqTarget < seqCount) {

                    string seqPath = mPathDsRoot + '/' + seqNames[mSeqTarget];
                    if (!TabularTextDS::checkDirectory(seqPath)) {
                        LOG(ERROR) << "** Failed to find sequence: " << seqPath << endl;
                        return false;
                    }
                    else {
                        mSeqCount = 1;
                        mSeqNames.resize(mSeqCount);
                        mSeqNames[0] = seqNames[mSeqTarget];
                        mSeqTarget = 0;
                        mSeqIdx = 0;
                    }
                }
                else {
                    LOG(ERROR) << "** Target sequence number is outside range.\n";
                    return false;
                }
            }
            else {
                // If some sequences does not exist, we still want to
                // be able to work with existing sequences
                for (size_t seq = 0; seq < seqCount; seq++) {
                    string seqPath = mPathDsRoot + '/' + seqNames[seq];
                    if (!TabularTextDS::checkDirectory(seqPath)) {
                        LOG(ERROR) << "** Failed to find sequence: " << seqPath << endl;
                    }
                    else {
                        mSeqNames.push_back(seqNames[seq]);
                    }
                }
                mSeqCount = mSeqNames.size();
                if (!mSeqCount)
                    return false;
                mSeqIdx = 0;
            }
        }
        else {
            LOG(ERROR) << "** Empty sequence names.\n";
            return false;
        }
        return true;
    }

    void BaseLoader::resolveOutputBasePath(const DS_ParamsPtr &pDsParams) {

        mPathOutBase = DS_Params::mapDsFormats(mDsFormat) + '_' + "sensor_config"; //pDsParams->getSensorConfig()->toDsStr();

        if (!mSeqNames.empty() && mSeqTarget >= 0 && mSeqTarget < mSeqNames.size()) {
            mPathOutBase += '_' + mSeqNames[mSeqTarget];
        }
    }

    bool BaseLoader::checkLoadState() {}

    bool BaseLoader::updateLoadState() {

        return this->checkLoadState();
    }

    void BaseLoader::loadData() {

        if (mSeqTarget < 0) {
            mSeqCount = mSeqNames.size();
        }
        else {
            mSeqCount = 1;
        }

        if (mSeqCount) {

            if (mSeqTarget < 0) {
                for (size_t seq = 0; seq < mSeqCount; seq++) {
                    loadSequence(mPathDsRoot, mSeqNames[seq], seq);
                }
            }
            else if (mSeqTarget < mSeqNames.size()) {
                loadSequence(mPathDsRoot, mSeqNames[mSeqTarget], 0);
            }
        }
    }

    // Do not implement this in BaseLoader (gives error for other datasets)
    void BaseLoader::loadSequence(const string &dsRoot, const string &sqPath, const size_t idx) {}

    string BaseLoader::getSequencePath() {

        if (mSeqNames.empty() || mSeqTarget < 0 || mSeqTarget >= mSeqNames.size())
            return string();
        return mPathDsRoot + '/' + mSeqNames[mSeqTarget];
    }

    void BaseLoader::resetSequences() {
        if (this->mSeqTarget < 0)
            this->mSeqIdx = 0;
    }

    void BaseLoader::incSequence() {
        if (this->mSeqTarget < 0 && this->mSeqIdx < mSeqCount-1)
            this->mSeqIdx++;
    }

    void BaseLoader::decSequence() {
        if (this->mSeqTarget < 0 && this->mSeqIdx > 1)
            this->mSeqIdx--;
    }

    bool BaseLoader::checkSequence(const unsigned int seq) const {

        if (mLoadState == TabularTextDS::GOOD) {
            if (mSeqTarget < 0) {
                return seq >= 0 && seq < mSeqCount;
            }
            else {
                return seq == 0;
            }
        }
        return false;
    }

    unsigned int BaseLoader::getNumTargetSequences() const {

        if (mSeqTarget < 0)
            return this->getNumSequences();
        else {
            if (checkSequence(mSeqIdx))
                return 1;
            else
                return 0;
        }
    }

    string BaseLoader::getSequenceName() const {

        if (!checkSequence(mSeqIdx))
            return string();
        return mSeqNames[mSeqIdx];
    }

    bool BaseLoader::isGood() const {

        return mLoadState == TabularTextDS::GOOD;
    }

    void BaseLoader::addImageHook(ImageHookPtr &pImageHook) {

        mvpImageHooks.push_back(pImageHook);
    }

    void BaseLoader::addIMU_Hook(IMU_HookPtr &pImuHook) {

        mvpIMU_Hooks.push_back(pImuHook);
    }

    void BaseLoader::addGT_PoseHook(PoseHookPtr &pPoseHook) {

        mvpGtPoseHooks.push_back(pPoseHook);
    }

    std::string BaseLoader::printLoaderStateStr() {

        ostringstream oss;

        oss << "# Loader Info.:\n";
        oss << "\tDataset State is: " << ((this->isGood()) ? "good" : "bad") << endl;
        oss << "\tDataset Name: " << this->getDatasetName() << endl;
        oss << "\tNum. Sequences: " << this->getNumSequences() << endl;
        oss << "\tNum. Target Sequences: " << this->getNumTargetSequences() << endl;
        oss << "\tNum. Max Iterations: " << this->getMaxNumIter() << endl;
        oss << "\tCurrent Sequence Name: " << this->getSequenceName() << endl;
        oss << "\tOutput Base Name: " << this->getOutputBasePath() << endl;
        oss << "\tImages:\n";
        oss << "\t\tNum. Total Images: " << this->getNumTotalImages() << endl;
        oss << "\t\tCurrent Sequence Num. Images: " << this->getNumImages() << endl;

        return oss.str();
    }

} // NAV24
