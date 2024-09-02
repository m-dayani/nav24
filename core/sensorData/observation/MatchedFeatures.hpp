//
// Created by masoud on 8/30/24.
//

#ifndef NAV24_MATCHEDFEATURES_HPP
#define NAV24_MATCHEDFEATURES_HPP

#include <vector>
#include <set>
#include <map>

#include "Observation.hpp"
#include "Calibration.hpp"

namespace NAV24::OB {

    // Data structures for efficient observation manipulation

    class MatchedFeatures {
    public:
        MatchedFeatures() : mId(idCounter++), mspMatches() {}

        void addMatch(const ObsPtr& pObs);

        [[nodiscard]] long getId() const { return mId; }

        std::vector<ObsPtr> getAllFeatures() { return mvpMatches; }

    protected:
        long mId;
        static long idCounter;
        std::set<ObsPtr> mspMatches;
        std::vector<ObsPtr> mvpMatches;
    };
    typedef std::shared_ptr<MatchedFeatures> MatchedFtPtr;


    class FeatureTracks {
    public:
        FeatureTracks() = default;

        void addMatch(const ObsPtr& pObs, const ObsPtr& pMatch);

        std::size_t getNumTracks() { return mmTracks.size(); }

        std::map<long, MatchedFtPtr> getAllTracks() { return mmTracks; }

    protected:
        void addFeatureToFrames(const ObsPtr& pObs);

    protected:
        std::map<long, MatchedFtPtr> mmTracks;
        std::map<ObsPtr, long> mmInvIndex;
        std::map<long, std::set<ObsPtr>> mmFrameMatches;
    };
    typedef std::shared_ptr<FeatureTracks> FtTracksPtr;

} // NAV24::OB

#endif //NAV24_MATCHEDFEATURES_HPP
