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
    // The first two classes are two unstable and inefficient

    class MatchedFeatures {
    public:
        MatchedFeatures() : mId(idCounter++), mspMatches() {}

        void addMatch(const ObsPtr& pObs);

        [[nodiscard]] long getId() const { return mId; }

        std::vector<ObsPtr> getAllFeatures() { return mvpMatches; }

        std::size_t size() { return mspMatches.size(); }
        long minFrame() { return *(msFrameIds.begin()); }
        long maxFrame() { return *(msFrameIds.rbegin()); }

        ObsPtr findObsByFrameId(const long& frameId);

    protected:
        long mId;
        static long idCounter;
        std::set<ObsPtr> mspMatches;
        std::vector<ObsPtr> mvpMatches;
        std::set<long> msFrameIds;
    };
    typedef std::shared_ptr<MatchedFeatures> MatchedFtPtr;


    class FeatureTracks {
    public:
        FeatureTracks() = default;

        void addMatch(const ObsPtr& pObs, const ObsPtr& pMatch);

//        std::size_t getNumTracks() { return mmTracks.size(); }

        std::map<long, MatchedFtPtr> getAllTracks() { return mmTracks; }

        bool findGoodFramesMapInit(int thNumTracks, int thNumMch, long& firstFrame, long& secondFrame);

        std::vector<std::pair<ObsPtr, ObsPtr>> getMatches(long frame1, long frame2);

        void cleanTracks(int lastFrameDist=3);
        void refreshAll();

    protected:
        void addFeatureToFrames(const ObsPtr& pObs);
        void removeTrack(const long& trackId);

    protected:
        std::map<long, MatchedFtPtr> mmTracks;
        std::map<ObsPtr, long> mmInvIndex;
        std::map<long, std::set<ObsPtr>> mmFrameMatches;
    };
    typedef std::shared_ptr<FeatureTracks> FtTracksPtr;


    class MatchedObs {
    public:
        MatchedObs() : mnMatches(0), mvMatches12() {}

        int mnMatches;
        FramePtrW mpMatchedFrame;
        std::vector<int> mvMatches12;
    };

} // NAV24::OB

#endif //NAV24_MATCHEDFEATURES_HPP
