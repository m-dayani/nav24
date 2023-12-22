//
// Created by root on 5/16/21.
//

#ifndef NAV24_BASELOADER_H
#define NAV24_BASELOADER_H

#include <vector>
#include <string>
#include <memory>

#include <opencv2/core.hpp>

#include "DS_Params.hpp"
#include "ImageDS.hpp"
#include "ImageHook.hpp"
#include "IMU_DS.hpp"
#include "IMU_Hook.hpp"
#include "PoseDS.hpp"
#include "PoseHook.hpp"


namespace NAV24 {

    class BaseLoader {
    public:
        explicit BaseLoader(const DS_ParamsPtr& pDsParams);
        virtual ~BaseLoader() = default;

        virtual void addImageHook(ImageHookPtr& pImageHook);
        virtual void addIMU_Hook(IMU_HookPtr& pImuHook);
        virtual void addGT_PoseHook(PoseHookPtr& pPoseHook);

        // Default play: Image-based!
        virtual void play() = 0;

        //Getters/Setters
        std::string getDatasetName() const { return mDsName; }
        unsigned int getNumSequences() const { return mSeqCount; }
        unsigned int getNumTargetSequences() const;
        unsigned int getMaxNumIter() const { return mnMaxIter; }
        std::string getSequenceName() const;
        std::string getOutputBasePath() const { return mPathOutBase; };

        //Utils
        virtual bool isGood() const;
        virtual void resetSequences();
        virtual void resetCurrSequence() = 0;

        virtual std::string printLoaderStateStr();

        void incSequence();
        void decSequence();

        virtual bool checkSequence(unsigned int seq) const;

        //Image
        virtual unsigned int getNumImages() = 0;
        virtual unsigned int getNumTotalImages() = 0;

        virtual double getImageTime(size_t idx) = 0;
        virtual std::string getImageFileName(size_t idx) = 0;

        virtual void getImage(size_t idx, cv::Mat &image, double &ts) = 0;
        virtual void getImage(size_t idx, cv::Mat &image, double &ts, std::string& imPath) = 0;

        //IMU
        // initTs is in (sec)
        //void initImu(double initTs, int idx = -1);
        virtual unsigned int getNextImu(double ts, std::vector<IMU_DataPtr> &vpImuData) = 0;

        //GT
        virtual unsigned int getNextPoseGT(double ts, std::vector<PosePtr>& vpPose) = 0;

    protected:
        bool checkDatasetPaths(const DS_ParamsPtr& pDsParams);
        virtual bool resolveSeqInfo(const DS_ParamsPtr& pDsParams);
        virtual void resolveOutputBasePath(const DS_ParamsPtr& pDsParams);

        std::string getSequencePath();

        void loadData();
        virtual void loadSequence(const std::string &dsRoot, const std::string &seqPath, size_t idx);

        bool updateLoadState();
        virtual bool checkLoadState();

    protected:
        // Hooks
        std::vector<ImageHookPtr> mvpImageHooks;
        std::vector<IMU_HookPtr> mvpIMU_Hooks;
        std::vector<PoseHookPtr> mvpGtPoseHooks;

        TabularTextDS::LoadState mLoadState;
        DS_Params::DS_Formats mDsFormat;

        std::string mDsName;

        //std::string mPathSettings;
        std::string mPathDsRoot;
        std::string mPathOutBase;
        std::string mPathImFile;
        std::string mPathImBase;
        std::string mPathImu;
        std::string mPathGT;

        // Sequence count counts the size of active dataStores (like mImDs)
        unsigned int mSeqCount;
        int mSeqTarget;
        std::vector<std::string> mSeqNames;
        // If mSeqCount == 1, mSeqIdx must always be zero
        unsigned int mSeqIdx;

        unsigned int mnMaxIter;

        double mTsFactor;

        bool mbGtQwFirst;
        bool mbGtPosFirst;
        bool mbImuGyroFirst;

        //std::shared_ptr<MyCalibrator> mpCalib;

    private:

    };

} // NAV24


#endif //NAV24_BASELOADER_H
