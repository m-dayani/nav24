//
// Created by masoud on 8/30/24.
//

#include <glog/logging.h>

#include "MatchedFeatures.hpp"
#include "Frame.hpp"

using namespace std;

namespace NAV24::OB {

    long MatchedFeatures::idCounter;

    void MatchedFeatures::addMatch(const ObsPtr &pObs) {

        if (!pObs || !pObs->getFrame()) {
            DLOG(WARNING) << "MatchedFeatures::addMatch, received a null observation without a frame\n";
            return;
        }

        if (mspMatches.contains(pObs)) {
            return;
        }

        mspMatches.insert(pObs);
        mvpMatches.push_back(pObs);
        msFrameIds.insert(pObs->getFrame()->getId());
    }

    ObsPtr MatchedFeatures::findObsByFrameId(const long &frameId) {

        if (mspMatches.empty()) {
            return nullptr;
        }
        for (const auto& pObs : mspMatches) {
            if (pObs) {
                auto pFrame = pObs->getFrame();
                if (pFrame && pFrame->getId() == frameId) {
                    return pObs;
                }
            }
        }
        return nullptr;
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
        if (!pObs || !pObs->getFrame()) {
            DLOG(WARNING) << "FeatureTracks::addFeatureToFrames, received a null observation without a frame\n";
            return;
        }
        const long frameId = pObs->getFrame()->getId();
        if (!mmFrameMatches.contains(frameId)) {
            mmFrameMatches[frameId] = set<OB::ObsPtr>();
        }
        mmFrameMatches[frameId].insert(pObs);
    }

    bool FeatureTracks::findGoodFramesMapInit(int thNumTracks, int thNumMch, long &firstFrame, long &secondFrame) {

        int cntNumTracks = 0;
        map<long, int> cntMinFrames;
        map<long, int> cntMaxFrames;

        for (const auto& pTrack : mmTracks) {
            if (pTrack.second->size() >= thNumMch) {
                cntNumTracks++;
                long minFrame = pTrack.second->minFrame();
                if (!cntMinFrames.contains(minFrame)) {
                    cntMinFrames.insert(make_pair(minFrame, 0));
                }
                cntMinFrames[minFrame] += 1;
                long maxFrame = pTrack.second->maxFrame();
                if (!cntMaxFrames.contains(maxFrame)) {
                    cntMaxFrames.insert(make_pair(maxFrame, 0));
                }
                cntMaxFrames[maxFrame] += 1;
            }
        }

        if (cntNumTracks < thNumTracks) {
            return false;
        }

        int maxCntMinFrame = cntMinFrames.begin()->second;
        firstFrame = cntMinFrames.begin()->first;
        for (const auto& minFrame : cntMinFrames) {
            if (minFrame.second > maxCntMinFrame) {
                maxCntMinFrame = minFrame.second;
                firstFrame = minFrame.first;
            }
        }

        int maxCntMaxFrame = cntMaxFrames.begin()->second;
        secondFrame = cntMaxFrames.begin()->first;
        for (const auto& maxFrame : cntMaxFrames) {
            if (maxFrame.second > maxCntMaxFrame) {
                maxCntMaxFrame = maxFrame.second;
                secondFrame = maxFrame.first;
            }
        }

        return true;
    }

    std::vector<std::pair<ObsPtr, ObsPtr>> FeatureTracks::getMatches(long frame1, long frame2) {

        vector<pair<ObsPtr, ObsPtr>> vpMatches;
        vpMatches.reserve(mmFrameMatches[frame1].size());

        for (const auto& pObs1 : mmFrameMatches[frame1]) {
            auto trackId = mmInvIndex[pObs1];
            auto pTrack1 = mmTracks[trackId];
            auto pObs2 = pTrack1->findObsByFrameId(frame2);
            if (pObs2) {
                vpMatches.push_back(make_pair(pObs1, pObs2));
            }
        }

        return vpMatches;
    }

    void FeatureTracks::cleanTracks(int lastFrameDist) {

        const long currFrameId = mmFrameMatches.rbegin()->first;
        vector<long> tracksToDelete{};
        tracksToDelete.reserve(mmTracks.size());
        for (const auto& pTrackPair : mmTracks) {
            auto& pTrack = pTrackPair.second;
            if (pTrack && currFrameId - pTrack->maxFrame() > lastFrameDist) {
                tracksToDelete.push_back(pTrack->getId());
            }
        }

        for (const auto& trackId : tracksToDelete) {
            this->removeTrack(trackId);
        }
    }

    void FeatureTracks::removeTrack(const long &trackId) {

        if (!mmTracks.contains(trackId)) {
            return;
        }
        auto vpObs = mmTracks[trackId]->getAllFeatures();
        for (auto& pObs : vpObs) {
            if (pObs) {
                if (pObs->getFrame()) {
                    // clean frames
                    long frameId = pObs->getFrame()->getId();
                    if (mmFrameMatches.contains(frameId)) {
                        auto frameList = mmFrameMatches[frameId];
                        if (frameList.contains(pObs))
                            frameList.erase(pObs);
                        if (frameList.empty())
                            mmFrameMatches.erase(frameId);
                    }
                }
                if (mmInvIndex.contains(pObs)) {
                    // clean inverse index
                    mmInvIndex.erase(pObs);
                }
            }
        }
        mmTracks.erase(trackId);
    }

    void FeatureTracks::refreshAll() {

        mmInvIndex.clear();
        mmTracks.clear();
        mmFrameMatches.clear();
    }

    int MatchedObs::getNumMatches(const FramePtr &pFrame) {
        if (pFrame && dynamic_pointer_cast<FrameImgMono>(pFrame)) {
            auto pMatchedObs = dynamic_pointer_cast<FrameImgMono>(pFrame)->getMatches();
            if (pMatchedObs) {
                return pMatchedObs->mnMatches;
            }
        }
        return 0;
    }

    void MatchedObs::getMatches(const FramePtr &pFrame, vector<int> &vMatches12, int &nMatches) {

        if (!pFrame || !dynamic_pointer_cast<FrameImgMono>(pFrame)) {
            return;
        }
        auto pImgFrame = dynamic_pointer_cast<FrameImgMono>(pFrame);
        auto pMatches12 = pImgFrame->getMatches();
        if (!pMatches12) {
            return;
        }
        vMatches12 = pMatches12->mvMatches12;
        nMatches = pMatches12->mnMatches;
    }
} // NAV24::OB
