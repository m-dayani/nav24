//
// Created by root on 12/21/23.
//

#ifndef NAV24_SYSTEM_HPP
#define NAV24_SYSTEM_HPP

#include <string>
#include <vector>
#include <map>

#include "Channel.hpp"
#include "ParameterServer.hpp"
#include "DataStore.hpp"
#include "Sensor.hpp"
#include "Atlas.hpp"
#include "TrajManager.hpp"
#include "Pose.hpp"


namespace NAV24 {

#define FCN_LD_PARAMS 1


class System : public Channel, public MsgCallback, public std::enable_shared_from_this<System> {
    public:
        inline static const std::string TOPIC{"System"};

        explicit System();

        void publish(const MsgPtr &message) override;

        void registerChannel(const MsgCbPtr& callback, const std::string &topic) override;

        void unregisterChannel(const MsgCbPtr& callback, const std::string &topic) override;

        void receive(const MsgPtr &msg) override;

    protected:
        void loadSettings(const std::string& settings);
        void handleConfigMsg(const MsgPtr &msg);
        void initComponents();

    protected:
        std::map<std::string, std::vector<MsgCbPtr>> mmChannels;

        std::shared_ptr<ParameterServer> mpParamServer;
        std::map<std::string, std::shared_ptr<DataStore>> mmpDataStores;
        std::map<std::string, std::shared_ptr<Sensor>> mmpSensors;
        std::map<std::string, TransPtr> mmpTrans;

        ParamPtr mpTempParam;

        AtlasPtr mpAtlas;
        TrajManagerPtr mpTrajManager;
    };

}

#endif //NAV24_SYSTEM_HPP
