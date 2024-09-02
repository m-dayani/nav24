//
// Created by masoud on 8/30/24.
//

#include "MatchedFeatures.hpp"

using namespace std;

namespace NAV24::OB {

    long MatchedFeatures::idCounter;

    void MatchedFeatures::addMatch(const ObsPtr &pObs) {

        if (mspMatches.contains(pObs)) {
            return;
        }
        mspMatches.insert(pObs);
        mvpMatches.push_back(pObs);
    }

    void FeatureTracks::addMatch(const ObsPtr &pObs, const ObsPtr &pMatch) {

        if (mmInvIndex.contains(pObs)) {
            const long trackIdx = mmInvIndex[pObs];
            if (!mmInvIndex.contains(pMatch)) {
                mmTracks[trackIdx]->addMatch(pMatch);
                this->addFeatureToFrames(pMatch);
                mmInvIndex.insert(make_pair(pMatch, trackIdx));
            }
        }
        else if (mmInvIndex.contains(pMatch)) {
            const long trackIdx = mmInvIndex[pMatch];
            if (!mmInvIndex.contains(pObs)) {
                mmTracks[trackIdx]->addMatch(pObs);
                this->addFeatureToFrames(pObs);
                mmInvIndex.insert(make_pair(pObs, trackIdx));
            }
        }
        else {
            auto pTrack = make_shared<MatchedFeatures>();
            const long trackIdx = pTrack->getId();
            pTrack->addMatch(pObs);
            pTrack->addMatch(pMatch);
            mmTracks.insert(make_pair(trackIdx, pTrack));
            this->addFeatureToFrames(pObs);
            mmInvIndex.insert(make_pair(pObs, trackIdx));
            this->addFeatureToFrames(pMatch);
            mmInvIndex.insert(make_pair(pMatch, trackIdx));
        }
    }

    void FeatureTracks::addFeatureToFrames(const ObsPtr &pObs) {
        const long frameId = pObs->getFrameId();
        if (!mmFrameMatches.contains(frameId)) {
            mmFrameMatches[frameId] = set<OB::ObsPtr>();
        }
        mmFrameMatches[frameId].insert(pObs);
    }
} // NAV24::OB
