//
// Created by masoud on 2/24/25.
//

#include <utility>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>
#include <glog/logging.h>

#include "OP_VPR_DBoW2.hpp"


using namespace std;


namespace NAV24::OP {


    VPR_DBoW2::VPR_DBoW2(string pathVocab, string pathDb, string pathTsMap) : Operator(),
            mPathVocab(std::move(pathVocab)), mPathDb(std::move(pathDb)), mPathTsMap(std::move(pathTsMap)), mVocLock() {

        loadVocabulary();
        loadDatabase();
        loadTsMap();
    }

    VPR_DBoW2::VPR_DBoW2(const ChannelPtr &pChannel) : Operator(pChannel), mVocLock() {}

    void VPR_DBoW2::add(const FramePtr &pKF) {

        vector<cv::Mat> vDescriptors;
        getDescriptors(pKF->getObservations(), vDescriptors);

        mpOrbDatabase->add(vDescriptors);
    }

    void VPR_DBoW2::getBestMatches(const FramePtr &pKF, const int nMatches,
                                   vector<unsigned long> &vResult) {

        vector<cv::Mat> vDescriptors;
        getDescriptors(pKF->getObservations(), vDescriptors);

        DBoW2::QueryResults ret;
        mpOrbDatabase->query(vDescriptors, ret, 4);

        vResult.reserve(nMatches);
        int cnt = 0;
        for (const auto& res : ret) {
            if (mTsMap.contains(res.Id)) {
                vResult.push_back(mTsMap[res.Id]);
            }
            cnt++;
            if (cnt >= nMatches) {
                break;
            }
        }
    }

    void VPR_DBoW2::loadVocabulary() {

        auto pathVoc = boost::filesystem::path(mPathVocab);
        if (mPathVocab.empty() || !exists(pathVoc)) {
            DLOG(WARNING) << "VPR_DBoW2::loadVocabulary, Could not find ORB vocabulary: " << mPathVocab << "\n";
            return;
        }

        string ext = pathVoc.extension().string();
        if (ext != ".txt" && ext != ".gz") {
            DLOG(WARNING) << "VPR_DBoW2::loadVocabulary, Text files are not supported\n";
            return;
        }

        bool loaded = false;

        // This might take a while!
        cout << "Loading ORB Vocab (this might take a while)...\n";
        mVocLock.lock();
        if (ext == ".txt") {
            mpOrbVocabulary = make_shared<ORBVocabulary>();
            loaded = mpOrbVocabulary->loadFromTextFile(mPathVocab);
        }
        else if (ext == ".gz") {
            mpOrbVocabulary = make_shared<ORBVocabulary>(mPathVocab);
            loaded = true;
        }
        mVocLock.unlock();
        if (loaded) cout << "Finished loading ORB Vocab.\n";
    }

    void VPR_DBoW2::loadTsMap() {

        // TS Map maps frames' timestamps to frame id's present in an ORB Database;
        if (mPathTsMap.empty() || !exists(boost::filesystem::path(mPathTsMap))) {
            DLOG(WARNING) << "Could not find timestamp map: " << mPathTsMap << "\n";
            return;
        }

        ifstream ifs(mPathTsMap);

        string line;
        unsigned int idx = 0;
        while (getline(ifs, line, '\n')) {

            istringstream iss{line};

            unsigned long ts;

            iss >> ts;

            mTsMap.insert(make_pair(idx, ts));
            idx++;
        }
    }

    void VPR_DBoW2::loadDatabase() {

        if (mPathDb.empty() || !exists(boost::filesystem::path(mPathDb))) {

            DLOG(INFO) << "Could not find ORB database: " << mPathDb << "\n";
            mVocLock.lock();
            if (mpOrbVocabulary) {
                mpOrbDatabase = make_shared<OrbDatabase>(*mpOrbVocabulary, false, 0);
            }
            mVocLock.unlock();
        }
        else {
            mpOrbDatabase = make_shared<OrbDatabase>(mPathDb);
        }
    }

    void VPR_DBoW2::saveDatabase() {

        if (mpOrbDatabase && !mPathDb.empty()) {
            mpOrbDatabase->save(mPathDb);
        }
    }

    void VPR_DBoW2::saveVocabulary() {

        mVocLock.lock();
        if (mpOrbVocabulary && !mPathVocab.empty()) {
            mpOrbVocabulary->save(mPathVocab);
        }
        mVocLock.unlock();
    }

    void VPR_DBoW2::getDescriptors(const vector <OB::ObsPtr> &vpObs, vector <cv::Mat> &vDescriptors) {

        vDescriptors.reserve(vpObs.size());

        for (const auto& pObs : vpObs) {
            if (dynamic_pointer_cast<OB::KeyPoint2D>(pObs)) {
                auto pKP = dynamic_pointer_cast<OB::KeyPoint2D>(pObs);
                vDescriptors.push_back(pKP->getDescriptor());
            }
        }
    }

#ifdef LIB_DBOW2_FOUND
    void VPR_DBoW2::createVocab(const vector <FramePtr> &vpFrames, std::shared_ptr<OrbVocabulary>& pVoc) {

        vector<vector<cv::Mat>> features;
        features.reserve(vpFrames.size());
        for (const auto& pFrame : vpFrames) {
            features.emplace_back();
            auto vpObs = pFrame->getObservations();
            getDescriptors(vpObs, features.back());
        }

        // branching factor and depth levels
        const int k = 9;
        const int L = 3;
        const DBoW2::WeightingType weight = DBoW2::TF_IDF;
        const DBoW2::ScoringType scoring = DBoW2::L1_NORM;

        pVoc = make_shared<OrbVocabulary>(k, L, weight, scoring);

        DLOG(INFO) << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
        pVoc->create(features);
        DLOG(INFO) << "... done!" << endl;

        DLOG(INFO) << "Vocabulary information: " << endl << *pVoc << endl << endl;
    }

    void VPR_DBoW2::computeBoW(const vector <OB::ObsPtr> &vpObs, DBoW2::BowVector& v1) {

        vector<cv::Mat> vDescriptors;
        getDescriptors(vpObs, vDescriptors);
        mVocLock.lock();
        mpOrbVocabulary->transform(vDescriptors, v1);
        mVocLock.unlock();
    }

    double VPR_DBoW2::computeScore(const DBoW2::BowVector &v1, const DBoW2::BowVector &v2) {

        mVocLock.lock();
        return mpOrbVocabulary->score(v1, v2);
        mVocLock.unlock();
    }

    void VPR_DBoW2::setTsMap(vector<unsigned long> &vTsMap) {

        for (size_t i = 0; i < vTsMap.size(); i++) {
            mTsMap.insert(make_pair(i, vTsMap[i]));
        }
    }

    void VPR_DBoW2::reloadDbWithVocab(const shared_ptr<ORBVocabulary> &pVoc) {

        mVocLock.lock();
        mpOrbVocabulary = pVoc;
        mpOrbDatabase = make_shared<OrbDatabase>(*mpOrbVocabulary, false, 0);
        mVocLock.unlock();
    }

    void VPR_DBoW2::setup(const MsgPtr &msg) {

        if (msg && dynamic_pointer_cast<MsgConfig>(msg)) {

            auto pParams = dynamic_pointer_cast<MsgConfig>(msg)->getConfig();
            if (pParams) {
                // verify it's VPR DBoW2 params
                auto pParamName = find_param<ParamType<string>>("name", pParams);
                if (!pParamName || pParamName->getValue() != "vpr_dbow2") {
                    return;
                }

                // path params: voc, db, and ts_map
                // voc can be the old ORB-SLAM.txt file or the newer voc.yml.gz file
                auto pParamPathVoc = find_param<ParamType<string>>("path_voc", pParams);
                mPathVocab = (pParamPathVoc) ? pParamPathVoc->getValue() : "";

                auto pParamPathDb = find_param<ParamType<string>>("path_db", pParams);
                mPathDb = (pParamPathDb) ? pParamPathDb->getValue() : "";

                auto pParamPathTsMap = find_param<ParamType<string>>("path_ts_map", pParams);
                mPathTsMap = (pParamPathTsMap) ? pParamPathTsMap->getValue() : "";

                loadVocabulary();
//                if (!mThLoadVoc) {
//                    mThLoadVoc = make_shared<thread>(&VPR_DBoW2::loadVocabulary, this);
//                }
                loadDatabase();
                loadTsMap();
            }
        }
    }

    void VPR_DBoW2::computeBowInfo(const FramePtr &pKF) {

        if (!pKF || !dynamic_pointer_cast<FrameMonoOS>(pKF) || !mpOrbVocabulary) {
            return;
        }

        auto pOrbFrame = dynamic_pointer_cast<FrameMonoOS>(pKF);

        auto mvpObservations = pOrbFrame->getObservations();
        vector<cv::Mat> vDesc;
        getDescriptors(mvpObservations, vDesc);

        DBoW2::BowVector v1;
        DBoW2::FeatureVector fv1;
        int levelsup = 0;
        mVocLock.lock();
        mpOrbVocabulary->transform(vDesc, v1, fv1, levelsup);
        mVocLock.unlock();

        pOrbFrame->setBowVecDBoW2(v1);
        pOrbFrame->setFtVecDBoW2(fv1);
        pOrbFrame->updateInitDBoW2(true);
    }

#endif

    // todo: implement MsgCallback's methods

    bool ORBVocabulary::loadFromTextFile(const string &filename) {
        std::ifstream f;
        f.open(filename.c_str());

        if(f.eof())
            return false;

        m_words.clear();
        m_nodes.clear();

        std::string s;
        getline(f,s);
        std::stringstream ss;
        ss << s;
        ss >> m_k;
        ss >> m_L;
        int n1, n2;
        ss >> n1;
        ss >> n2;

        if(m_k<0 || m_k>20 || m_L<1 || m_L>10 || n1<0 || n1>5 || n2<0 || n2>3)
        {
            std::cerr << "Vocabulary loading failure: This is not a correct text file!" << endl;
            return false;
        }

        m_scoring = (DBoW2::ScoringType)n1;
        m_weighting = (DBoW2::WeightingType)n2;
        createScoringObject();

        // nodes
        int expected_nodes =
                (int)((pow((double)m_k, (double)m_L + 1) - 1)/(m_k - 1));
        m_nodes.reserve(expected_nodes);

        m_words.reserve(pow((double)m_k, (double)m_L + 1));

        m_nodes.resize(1);
        m_nodes[0].id = 0;
        while(!f.eof())
        {
            std::string snode;
            getline(f,snode);
            std::stringstream ssnode;
            ssnode << snode;

            int nid = m_nodes.size();
            m_nodes.resize(m_nodes.size()+1);
            m_nodes[nid].id = nid;

            int pid ;
            ssnode >> pid;
            m_nodes[nid].parent = pid;
            m_nodes[pid].children.push_back(nid);

            int nIsLeaf;
            ssnode >> nIsLeaf;

            std::stringstream ssd;
            for(int iD=0;iD<DBoW2::FORB::L;iD++)
            {
                std::string sElement;
                ssnode >> sElement;
                ssd << sElement << " ";
            }
            DBoW2::FORB::fromString(m_nodes[nid].descriptor, ssd.str());

            ssnode >> m_nodes[nid].weight;

            if(nIsLeaf>0)
            {
                int wid = m_words.size();
                m_words.resize(wid+1);

                m_nodes[nid].word_id = wid;
                m_words[wid] = &m_nodes[nid];
            }
            else
            {
                m_nodes[nid].children.reserve(m_k);
            }
        }

        return true;
    }
} // NAV24::OP
//