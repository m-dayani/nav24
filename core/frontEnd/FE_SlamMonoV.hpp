//
// Created by masoud on 8/29/24.
//

#ifndef NAV24_FE_SLAMMONOV_HPP
#define NAV24_FE_SLAMMONOV_HPP

#include "FrontEnd.hpp"
#include "Frame.hpp"
#include "OP_FtDtOrbSlam.hpp"
#include "MatchedFeatures.hpp"
#include "FeatureGrid.hpp"
#include "Calibration.hpp"
#include "OP_FtAssocOrbSlam.hpp"


namespace NAV24::FE {

    class SlamMonoV : public FrontEnd, public std::enable_shared_from_this<SlamMonoV> {
    public:
        explicit SlamMonoV(const ChannelPtr &pChannel);
        void receive(const MsgPtr &msg) override;

    protected:
        void setup(const MsgPtr &configMsg) override;

        void handleRequest(const MsgPtr &reqMsg) override;

        void run() override;

        virtual void handleImageMsg(const MsgPtr &msg);

        void initOperators();

    protected:
        bool mbInitialized;

        //std::string mTrType;

        std::string mMapName;
        std::vector<WO::WoPtr> mvpPts3D{};

        std::string mTrajectory;
        ParamPtr mpTempParam;

        FramePtr mpFirstFrame;
        FramePtr mpLastFrame;
        FramePtr mpCurrFrame;

        std::shared_ptr<OB::FeatureTracks> mpFtTracks;
//        std::shared_ptr<OB::FeatureGrid> mpDummyGrid;

        CalibPtr mpCalib;

        std::shared_ptr<OP::FtDt> mpOrbDetector;
        std::shared_ptr<OP::FtAssoc> mpOrbMatcher;
    };

}

#endif //NAV24_FE_SLAMMONOV_HPP
