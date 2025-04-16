//
// Created by root on 12/21/23.
//

#ifndef NAV24_SYSTEM_HPP
#define NAV24_SYSTEM_HPP

#include <string>
#include <vector>
#include <map>
#include <thread>

#include "Interface.hpp"
#include "ParameterServer.hpp"
#include "DataStore.hpp"
#include "Sensor.hpp"
#include "Atlas.hpp"
#include "TrajManager.hpp"
#include "trajectory/pose/Pose.hpp"
#include "Output.hpp"


namespace NAV24 {

#define FCN_LD_PARAMS 1
#define FCN_SYS_UPDATE 4
#define FCN_GET_TRANS 7
#define FCN_SYS_RUN 8
#define FCN_SYS_STOP 10
#define FCN_SYS_INIT_OP 21

class System : public Channel, public MsgCallback, public std::enable_shared_from_this<System> {
    public:
        inline static const std::string TOPIC{"System"};

        System();
//        explicit System(const std::string& settings);

        void send(const MsgPtr &message) override;
        void publish(const MsgPtr &message) override;

        void registerChannel(int catId, const MsgCbPtr& callback) override;
        void unregisterChannel(int catId, const MsgCbPtr& callback) override;
        void registerPublisher(int chId, const MsgCbPtr &callback) override;
        void registerSubscriber(int chId, const MsgCbPtr &callback) override;

        void receive(const MsgPtr &msg) override;

    protected:
        void loadSettings(const std::string& settings);
        void loadParameters(const std::string& settings);
        void loadDatasets();
        void loadOutputs();
        void loadSensors();
        void loadCameras();
        void loadPoseSensors();
        void loadOperators();

        void initComponents();

        void setup(const MsgPtr &configMsg) override;

        void run() override;

    void stop() override;

    void handleConfigMsg(const MsgPtr &msg);
        void handleRequest(const MsgPtr& msg) override;

    protected:
        // This is the preferred change
        std::map<int, std::set<MsgCbPtr>> mmChannels;
        std::map<int, std::set<MsgCbPtr>> mmPublishers;
        std::map<int, std::set<MsgCbPtr>> mmSubscribers;

        std::shared_ptr<ParameterServer> mpParamServer;
        std::map<std::string, std::shared_ptr<DataStore>> mmpDataStores;
        std::map<std::string, std::shared_ptr<Sensor>> mmpSensors;
        std::map<std::string, OutputPtr> mmpOutputs;

        std::set<std::shared_ptr<std::thread>> mpThreads;

        ParamPtr mpTempParam;

        AtlasPtr mpAtlas;
        TrajManagerPtr mpTrajManager;
    };

}

#endif //NAV24_SYSTEM_HPP
