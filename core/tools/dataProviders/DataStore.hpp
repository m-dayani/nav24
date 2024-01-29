//
// Created by root on 5/16/21.
//

#ifndef NAV24_BASELOADER_H
#define NAV24_BASELOADER_H

#include <vector>
#include <string>
#include <memory>

#include <opencv2/core.hpp>

//#include "ImageDS.hpp"
//#include "IMU_DS.hpp"
//#include "PoseDS.hpp"
#include "TabularTextDS.hpp"
#include "Message.hpp"
#include "Channel.hpp"


namespace NAV24 {

#define FCN_DS_LOAD 1
#define FCN_DS_REQ 3
#define FCN_DS_REQ_CHANGE 4
//#define FCN_DS_PRINT 5

#define TAG_DS_GET_STAT "DataProvider/GetStat"
#define TAG_DS_GET_PATH_IMG "DataProvider/GetImagePath"
#define TAG_DS_GET_PATH_IMU "DataProvider/GetImuPath"
#define TAG_DS_GET_PATH_GT "DataProvider/GetGtPath"

#define TAG_DS_CH_SEQ_RST "DataProvider/ResetSequences"
#define TAG_DS_CH_SEG_INC "DataProvider/IncrementSequences"
#define TAG_DS_CH_SEQ_DEC "DataProvider/DecrementSequences"

    class DataStore : public MsgCallback {
    public:
        //inline static const std::string TAG{"DataStore"};
        inline static const std::string TOPIC{"DataStore"};

        explicit DataStore(const ChannelPtr& server);
        DataStore(const ChannelPtr& server, const MsgPtr& configMsg);

        void receive(const MsgPtr& msg) override;

        void notifyChange();

        // Default play: Image-based!
        //virtual void play() = 0;

    protected:
        //Getters/Setters
        std::string getDatasetName() const { return mDsName; }
        unsigned int getNumSequences() const { return mSeqCount; }
        unsigned int getNumTargetSequences() const;
        unsigned int getMaxNumIter() const { return mnMaxIter; }
        std::string getSequenceName() const;
        std::string getOutputBasePath() const { return mPathResults; };

        //Utils
        virtual bool isGood() const;
        virtual bool checkSequence(unsigned int seq) const;

        virtual void resetSequences();
        //virtual void resetCurrSequence() = 0;
        void incSequence();
        void decSequence();

        virtual std::string printLoaderStateStr();

        //Image
        //virtual unsigned int getNumImages() = 0;
        //virtual unsigned int getNumTotalImages() = 0;

    protected:
        bool checkDatasetPaths(const ParamPtr& pPathParams);
        virtual bool resolveSeqInfo(const ParamPtr& pSeqParams);
        //virtual void resolveOutputPath(const ParamPtr& pParams);
        virtual void loadOtherInfo(const ParamPtr& pParams);

        std::string getSequencePath();

        //void loadData();
        //virtual void loadSequence(const std::string &dsRoot, const std::string &seqPath, size_t idx);

        //bool updateLoadState();
        //virtual bool checkLoadState();

        void handleRequest(const MsgPtr& msg);
        void handleChangeRequest(const MsgPtr& msg);
        void load(const MsgPtr& configMsg);

    protected:
        ChannelPtr mpChannel;

        ParamPtr mpDsParams;

        TabularTextDS::LoadState mLoadState;

        std::string mDsName;
        std::string mDsFormat;

        std::string mPathDsRoot;
        std::string mPathResults;
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

    private:

    };

} // NAV24


#endif //NAV24_BASELOADER_H
