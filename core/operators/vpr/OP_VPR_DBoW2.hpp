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
#include "Message.hpp"
#include "Operator.hpp"
#ifdef LIB_DBOW2_FOUND
#include "DBoW2/DBoW2.h"
#endif


namespace NAV24::OP {

    class VPR_DBoW2 : public Operator {
    public:
        VPR_DBoW2(std::string pathVocab, std::string pathDb, std::string pathTsMap);
        explicit VPR_DBoW2(const ChannelPtr& pChannel);

        void add(const FramePtr& pKF);
        void getBestMatches(const FramePtr &pKF, int nMatches,
                            std::vector<unsigned long>& vResult);
        void setTsMap(std::vector<unsigned long>& vTsMap);

#ifdef LIB_DBOW2_FOUND
        static void createVocab(const std::vector<FramePtr>& vpFrames, std::shared_ptr<OrbVocabulary>& pVoc);
        void reloadDbWithVocab(const std::shared_ptr<OrbVocabulary>& pVoc);
#endif

        // todo: unify these data conversion methods
        static void getDescriptors(const std::vector<OB::ObsPtr>& vpObs, std::vector<cv::Mat>& vDescriptors);

        void computeBowInfo(const FramePtr& pKF);

    protected:
        void setup(const MsgPtr &configMsg) override;

    private:
        void loadVocabulary();
        void saveVocabulary();
        void loadDatabase();
        void saveDatabase();
        void loadTsMap();

#ifdef LIB_DBOW2_FOUND
        void computeBoW(const std::vector<OB::ObsPtr>& vpObs, DBoW2::BowVector& v1);
        double computeScore(const DBoW2::BowVector& v1, const DBoW2::BowVector& v2);
#endif

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
