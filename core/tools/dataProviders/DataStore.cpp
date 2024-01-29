//
// Created by root on 5/16/21.
//

#include "DataStore.hpp"
#include "ParameterServer.hpp"

#include <memory>

using namespace std;
using namespace boost::filesystem;

namespace NAV24 {

#define TAG_DS_NAME "name"
#define TAG_DS_FORMAT "format"
#define TAG_DS_PATH "paths"
#define TAG_DS_SEQ "sequence"


    DataStore::DataStore(const ChannelPtr& server) :
            mpChannel(server), mpDsParams(), mLoadState(TabularTextDS::LoadState::BAD_PATH),
            mDsFormat(), mDsName(), mSeqCount(0), mSeqTarget(0), mSeqIdx(0), mnMaxIter(0), mTsFactor(1.0),
            mbGtQwFirst(false), mbGtPosFirst(false), mbImuGyroFirst(false) {}

    DataStore::DataStore(const ChannelPtr &server, const MsgPtr &configMsg) : DataStore(server) {
        this->receive(configMsg);
    }

    void DataStore::notifyChange() {

        // todo: generalize
        MsgPtr msg = make_shared<Message>("Topic/Notify/Sensors", "DS Changed");
        mpChannel->publish(msg);
    }

    void DataStore::receive(const MsgPtr &msg) {

        // check the topic
        string topic = msg->getTopic();

        // check for param server response
        if (topic == ParameterServer::TOPIC) {
            this->load(msg);
            return;
        }

        if (topic != DataStore::TOPIC) {
            // todo: log warning
            return;
        }

        // check function
        int action = msg->getTargetId();
        switch (action) {
            case FCN_DS_LOAD:
                this->load(msg);
                break;
            case FCN_DS_REQ:
                this->handleRequest(msg);
                break;
            case FCN_DS_REQ_CHANGE:
                this->handleChangeRequest(msg);
                break;
            default:
                //todo: log warning
                break;
        }
    }

    void DataStore::handleRequest(const MsgPtr &msg) {

        // All these messages have a sender and require a response

        MsgReqPtr request = static_pointer_cast<MsgRequest>(msg);
        if (!request) {
            // todo: log warning
            return;
        }

        MsgCbPtr sender = request->getCallback();

        // check message
        string tag = request->getMessage();

        if (tag == TAG_DS_GET_STAT) {
            msg->setMessage(this->printLoaderStateStr());
            sender->receive(msg);
            return;
        }

        ParamPtr pParam = nullptr;
        if (tag == TAG_DS_GET_PATH_IMG) {
            string imBase = this->getSequencePath() + "/" + mPathImBase;
            if (mPathImBase.empty()) {
                imBase = mPathImBase;
            }
            string imFile = this->getSequencePath() + "/" + mPathImFile;
            if (mPathImFile.empty()) {
                imFile = mPathImFile;
            }
            vector<string> vImgPath{imBase, imFile};
            pParam = make_shared<ParamSeq<string>>("imagePaths", nullptr, vImgPath);
            //pParam->insertChild("imagePath", make_shared<ParamType<string>>("imagePath", pParam, imBase));
            //pParam->insertChild("imageFile", make_shared<ParamType<string>>("imageFile", pParam, imBase));
        }
        else if (tag == TAG_DS_GET_PATH_IMU) {
            string imuFile = this->getSequencePath() + "/" + mPathImu;
            pParam = make_shared<ParamType<string>>("imuPath", nullptr, imuFile);
        }
        else if (tag == TAG_DS_GET_PATH_GT) {
            string gtFile = this->getSequencePath() + "/" + mPathGT;
            pParam = make_shared<ParamType<string>>("imuPath", nullptr, gtFile);
        }

        // create a response message
        MsgPtr response = make_shared<MsgConfig>(msg->getTopic(), pParam);

        // send back to the caller
        sender->receive(response);
    }

    void DataStore::handleChangeRequest(const MsgPtr &msg) {

        const string tag = msg->getMessage();
        if (tag == TAG_DS_CH_SEQ_RST) {
            this->resetSequences();
        }
        else if (tag == TAG_DS_CH_SEG_INC) {
            this->incSequence();
        }
        else if (tag == TAG_DS_CH_SEQ_DEC) {
            this->decSequence();
        }
        else {
            // todo: log error
        }
    }

    void DataStore::load(const MsgPtr &configMsg) {

        auto pMsg = static_pointer_cast<MsgConfig>(configMsg);
        if (pMsg) {
            auto pParam = pMsg->getConfig();
            if (pParam) {
                mpDsParams = pParam;
                bool resPath = false, resSeq = false;
                // Dataset name
                auto pParamNew = find_param<ParamType<string>>(TAG_DS_NAME, pParam);
                if (pParamNew) {
                    mDsName = pParamNew->getValue();
                }
                // Dataset format
                if (pParamNew = find_param<ParamType<string>>(TAG_DS_FORMAT, pParam)) {
                    mDsFormat = pParamNew->getValue();
                }
                // Paths
                auto pParamPath = pParam->read(TAG_DS_PATH);
                if (pParamPath) {
                    resPath = checkDatasetPaths(pParamPath);
                }
                // Sequence
                auto pParamSeq = pParam->read(TAG_DS_SEQ);
                if (pParamSeq) {
                    resSeq = resolveSeqInfo(pParamSeq);
                }
                // Other info
                loadOtherInfo(pParam);
                //resolveOutputPath(pParam);

                mLoadState = (resPath && resSeq) ? TabularTextDS::GOOD : TabularTextDS::BAD_PATH;
            }
        }
    }

    void DataStore::loadOtherInfo(const ParamPtr &pParam) {

        auto pTsFactor = find_param<ParamType<double>>("tsFactor", pParam);
        if (pTsFactor) {
            mTsFactor = pTsFactor->getValue();
        }
        auto pParamNew = find_param<ParamType<int>>("numMaxIter", pParam);
        if (pParamNew) {
            mnMaxIter = pParamNew->getValue();
        }
        if (pParamNew = find_param<ParamType<int>>("gtQwFirst", pParam)) {
            mbGtQwFirst = pParamNew->getValue() > 0;
        }
        if (pParamNew = find_param<ParamType<int>>("imuGyroFirst", pParam)) {
            mbImuGyroFirst = pParamNew->getValue() > 0;
        }
        if (pParamNew = find_param<ParamType<int>>("gtPosFirst", pParam)) {
            mbGtPosFirst = pParamNew->getValue() > 0;
        }
    }

    bool DataStore::checkDatasetPaths(const ParamPtr& pPathParams) {

        // Check DS root path
        auto pParam = find_param<ParamType<string>>("root", pPathParams);
        if (pParam) {
            mPathDsRoot = pParam->getValue();
        }
        if (!TabularTextDS::checkDirectory(mPathDsRoot)) {
            return false;
        }

        // Assign relative sequence data paths
        if (pParam = find_param<ParamType<string>>("imageBase", pPathParams)) {
            mPathImBase = pParam->getValue();
        }
        if (pParam = find_param<ParamType<string>>("imageFile", pPathParams)) {
            mPathImFile = pParam->getValue();
        }
        if (pParam = find_param<ParamType<string>>("imu", pPathParams)) {
            mPathImu = pParam->getValue();
        }
        if (pParam = find_param<ParamType<string>>("gt", pPathParams)) {
            mPathGT = pParam->getValue();
        }
        if (pParam = find_param<ParamType<string>>("results", pPathParams)) {
            mPathResults = pParam->getValue();
        }

        return true;
    }

    bool DataStore::resolveSeqInfo(const ParamPtr& pParamSeq) {

        auto pParam = find_param<ParamType<int>>("target", pParamSeq);
        if (pParam) {
            mSeqTarget = pParam->getValue();
        }

        vector<string> seqNames{};
        auto pParamSeqStr = find_param<ParamSeq<string>>("names", pParamSeq);
        if (pParamSeqStr) {
            seqNames = pParamSeqStr->getValue();
        }
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

    /*void DataStore::resolveOutputPath(const ParamPtr& pDsParams) {

        mPathResults = mDsFormat + '_' + "sensor_config"; //pDsParams->getSensorConfig()->toDsStr();

        if (!mSeqNames.empty() && mSeqTarget >= 0 && mSeqTarget < mSeqNames.size()) {
            mPathResults += '_' + mSeqNames[mSeqTarget];
        }
    }

    bool DataStore::checkLoadState() {
        return false;
    }

    bool DataStore::updateLoadState() {

        return this->checkLoadState();
    }

    void DataStore::loadData() {

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

    // Do not implement this in DataStore (gives error for other dataset)
    void DataStore::loadSequence(const string &dsRoot, const string &sqPath, const size_t idx) {}*/

    string DataStore::getSequencePath() {

        if (mSeqNames.empty() || mSeqTarget < 0 || mSeqTarget >= mSeqNames.size())
            return string();
        return mPathDsRoot + '/' + mSeqNames[mSeqTarget];
    }

    void DataStore::resetSequences() {
        if (this->mSeqTarget < 0) {
            this->mSeqIdx = 0;
            this->notifyChange();
        }
    }

    void DataStore::incSequence() {
        if (this->mSeqTarget < 0 && this->mSeqIdx < mSeqCount-1) {
            this->mSeqIdx++;
            this->notifyChange();
        }
    }

    void DataStore::decSequence() {
        if (this->mSeqTarget < 0 && this->mSeqIdx > 1) {
            this->mSeqIdx--;
            this->notifyChange();
        }
    }

    bool DataStore::checkSequence(const unsigned int seq) const {

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

    unsigned int DataStore::getNumTargetSequences() const {

        if (mSeqTarget < 0)
            return this->getNumSequences();
        else {
            if (checkSequence(mSeqIdx))
                return 1;
            else
                return 0;
        }
    }

    string DataStore::getSequenceName() const {

        if (!checkSequence(mSeqIdx))
            return string();
        return mSeqNames[mSeqIdx];
    }

    bool DataStore::isGood() const {

        return mLoadState == TabularTextDS::GOOD;
    }

    std::string DataStore::printLoaderStateStr() {

        ostringstream oss;

        oss << "# Loader Info.:\n";
        oss << "\tDataset State is: " << ((this->isGood()) ? "good" : "bad") << endl;
        oss << "\tDataset Name: " << this->getDatasetName() << endl;
        oss << "\tDataset Format: " << this->mDsFormat << endl;
        oss << "\tNum. Sequences: " << this->getNumSequences() << endl;
        oss << "\tNum. Target Sequences: " << this->getNumTargetSequences() << endl;
        oss << "\tNum. Max Iterations: " << this->getMaxNumIter() << endl;
        oss << "\tCurrent Sequence Name: " << this->getSequenceName() << endl;
        oss << "\tOutput Base Name: " << this->getOutputBasePath() << endl;
        oss << "\tImages:\n";
        //oss << "\t\tNum. Total Images: " << this->getNumTotalImages() << endl;
        //oss << "\t\tCurrent Sequence Num. Images: " << this->getNumImages() << endl;

        return oss.str();
    }

} // NAV24