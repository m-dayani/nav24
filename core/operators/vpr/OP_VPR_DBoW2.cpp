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


    VPR_DBoW2::VPR_DBoW2(string pathVocab, string pathDb, string pathTsMap) :
            mPathVocab(std::move(pathVocab)), mPathDb(std::move(pathDb)), mPathTsMap(std::move(pathTsMap)) {

        loadVocabulary();
        loadDatabase();
        loadTsMap();
    }

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

        if (mPathVocab.empty() || !exists(boost::filesystem::path(mPathVocab))) {
            DLOG(WARNING) << "Could not find ORB vocabulary: " << mPathVocab << "\n";
            return;
        }
        mpOrbVocabulary = make_shared<OrbVocabulary>(mPathVocab);
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
            mpOrbDatabase = make_shared<OrbDatabase>(*mpOrbVocabulary, false, 0);
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

        if (mpOrbVocabulary && !mPathVocab.empty()) {
            mpOrbVocabulary->save(mPathVocab);
        }
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
        mpOrbVocabulary->transform(vDescriptors, v1);
    }

    double VPR_DBoW2::computeScore(const DBoW2::BowVector &v1, const DBoW2::BowVector &v2) {

        return mpOrbVocabulary->score(v1, v2);
    }

    void VPR_DBoW2::setTsMap(vector<unsigned long> &vTsMap) {

        for (size_t i = 0; i < vTsMap.size(); i++) {
            mTsMap.insert(make_pair(i, vTsMap[i]));
        }
    }

    void VPR_DBoW2::reloadDbWithVocab(const shared_ptr <OrbVocabulary> &pVoc) {

        mpOrbVocabulary = pVoc;
        mpOrbDatabase = make_shared<OrbDatabase>(*mpOrbVocabulary, false, 0);
    }

#endif

} // NAV24::OP
//