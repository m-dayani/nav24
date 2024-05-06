//
// Created by root on 5/16/21.
//

#ifndef NAV24_BASELOADER_H
#define NAV24_BASELOADER_H

#include <vector>
#include <string>
#include <memory>

#include <opencv2/core.hpp>

#include "TabularTextDS.hpp"
#include "Message.hpp"
#include "Interface.hpp"


namespace NAV24 {

#define FCN_DS_LOAD 1
#define FCN_DS_REQ 3
#define FCN_DS_REQ_CHANGE 4
#define FCN_DS_REQ_CH_NS 22
//#define FCN_DS_PRINT 5

#define TAG_DS_GET_STAT "DataProvider/GetStat"
#define TAG_DS_GET_PATH_IMG "DataProvider/GetImagePath"
#define TAG_DS_GET_PATH_IMU "DataProvider/GetImuPath"
#define TAG_DS_GET_PATH_GT "DataProvider/GetGtPath"
#define TAG_DS_GET_PATH_MODEL "DataProvider/GetModelPath"

#define TAG_DS_CH_SEQ_RST "DataProvider/ResetSequences"
#define TAG_DS_CH_SEQ_INC "DataProvider/IncrementSequences"
#define TAG_DS_CH_SEQ_DEC "DataProvider/DecrementSequences"

    class DataStore : public MsgCallback {
    public:
        //inline static const std::string TAG{"DataStore"};
        inline static const std::string TOPIC{"DataStore"};

        explicit DataStore(ChannelPtr  server);
        //DataStore(const ChannelPtr& server, const MsgPtr& configMsg);

        void receive(const MsgPtr& msg) override;

    protected:
        void notifyChange();

        //Getters/Setters
        [[nodiscard]] std::string getDatasetName() const { return mDsName; }
        [[nodiscard]] unsigned int getNumSequences() const { return mSeqCount; }
        [[nodiscard]] unsigned int getNumTargetSequences() const;
        [[nodiscard]] unsigned int getMaxNumIter() const { return mnMaxIter; }
        [[nodiscard]] std::string getSequenceName() const;
        [[nodiscard]] std::string getOutputBasePath() const { return mPathResults; };

        //Utils
        [[nodiscard]] virtual bool isGood() const;
        [[nodiscard]] virtual bool checkSequence(unsigned int seq) const;

        virtual void resetSequences();
        void incSequence();
        void decSequence();
        void changeSeqByName(const std::string& seqName);

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

        //virtual void loadSequence(const std::string &dsRoot, const std::string &seqPath, size_t idx);
        //bool updateLoadState();
        //virtual bool checkLoadState();

        void handleRequest(const MsgPtr& msg) override;
        void handleChangeRequest(const MsgPtr& msg);
        void setup(const MsgPtr& configMsg) override;

        void run() override;

    protected:
        ChannelPtr mpChannel;

        // Never hold a pointer to params, it's ParamServer's property and
        // changing params can cause problems especially when working with threads
        //ParamPtr mpDsParams;

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

        std::vector<ParamPtr> mvpParams;

    private:

    };

} // NAV24


#endif //NAV24_BASELOADER_H
