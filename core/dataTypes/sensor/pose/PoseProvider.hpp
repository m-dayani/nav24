//
// Created by masoud on 6/19/24.
//

#ifndef NAV24_POSEPROVIDER_HPP
#define NAV24_POSEPROVIDER_HPP

#include "Sensor.hpp"
#include "TabularTextDS.hpp"


namespace NAV24 {
    class PoseProvider : public Sensor {
    public:
        explicit PoseProvider(const ChannelPtr& pChannel);
        ~PoseProvider();

        void receive(const MsgPtr &msg) override;

        static ParamPtr getPoseParams(const std::string& seqBase, const std::string& gtFile, double tsFact,
                                      bool posFirst, bool qwFirst, std::vector<ParamPtr>& vpParams);

    protected:
        void setup(const MsgPtr &msg) override;

        void getNext(MsgPtr pReq) override;

        void getNextBr(MsgPtr msg) override;

        void reset() override;

        [[nodiscard]] std::string printStr(const std::string &prefix) const override;

        void handleRequest(const MsgPtr &reqMsg) override;

        void run() override;

    protected:
        std::string mSeqBase;
        std::string mPoseFile;
        double tsFactor;
        bool mbPosFirst;
        bool mbQwFirst;

        std::shared_ptr<TabularTextDS> mpPoseDS;
    };
} // NAV24

#endif //NAV24_POSEPROVIDER_HPP
