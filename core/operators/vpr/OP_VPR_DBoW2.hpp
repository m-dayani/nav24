//
// Created by masoud on 2/24/25.
//
// Visual Place Recognition using DBoW2
// Link world objects using feature-matching (ORB key points and descriptors)
// Requires DBoW2 Package

#ifndef NAV24_OP_VPR_DBOW2_HPP
#define NAV24_OP_VPR_DBOW2_HPP

#include <vector>
#include <list>
#include <set>
#include<mutex>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>

#include "Frame.hpp"
#ifdef LIB_DBOW2_FOUND
#include "DBoW2.h"
#endif


namespace NAV24::OP {

    class VPR_DBoW2 {
    public:
        VPR_DBoW2(std::string   pathVocab, std::string  pathDb, std::string  pathTsMap);

        void add(const FramePtr& pKF);
        void getBestMatches(const FramePtr &pKF, int nMatches,
                            std::vector<unsigned long>& vResult);

        static void createVocab(const std::vector<FramePtr>& vpFrames, std::shared_ptr<OrbVocabulary>& pVoc);

    private:
        void loadVocabulary();
        void saveVocabulary();
        void loadDatabase();
        void saveDatabase();
        void loadTsMap();

        static void getDescriptors(const std::vector<OB::ObsPtr>& vpObs, std::vector<cv::Mat>& vDescriptors);
        void computeBoW(const std::vector<OB::ObsPtr>& vpObs, DBoW2::BowVector& v1);
        double computeScore(const DBoW2::BowVector& v1, const DBoW2::BowVector& v2);

    private:
        std::string mPathVocab;
        std::string mPathDb;
        std::string mPathTsMap;

        std::map<unsigned int, unsigned long> mTsMap;

#ifdef LIB_DBOW2_FOUND
        std::shared_ptr<OrbVocabulary> mpOrbVocabulary;
        std::shared_ptr<OrbDatabase> mpOrbDatabase;
#endif
    };

} // NAV24::OP
//

#endif //NAV24_OP_VPR_DBOW2_HPP
