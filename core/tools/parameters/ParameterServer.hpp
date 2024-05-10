//
// Created by root on 5/10/21.
//

#ifndef NAV24_PARAMETERSERVER_HPP
#define NAV24_PARAMETERSERVER_HPP

#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>

#include "Message.hpp"
#include "Interface.hpp"


namespace NAV24 {

#define FCN_PS_LOAD 1
#define FCN_PS_SAVE 2
#define FCN_PS_REQ 3
#define FCN_PS_CONF 4
#define FCN_PS_PRINT 5
#define TAG_PS_GET_STAT "ParamServer/GetStat"
#define TAG_PS_USE_LOAD_PATH "ParamServer/UseLoadPath"

    class ParameterServer : public MsgCallback {
    public:
        //inline static const std::string TAG{"ParameterServer"};
        inline static const std::string TOPIC{"ParamServer"};

        explicit ParameterServer(const ChannelPtr&  server);
        //ParameterServer(const ChannelPtr& server, const MsgPtr& configMsg);

        void receive(const MsgPtr& msg) override;

    protected:
        void load(const std::string& settings);
        void save(const std::string& pathParams);
        void handleRequest(const MsgPtr& msg) override;
        void changeParams(const MsgPtr& msg);

        void setup(const MsgPtr &configMsg) override;

        void run() override;

    private:
        std::string mConfigFile;

        ParamPtr mpParamRoot;
        std::vector<ParamPtr> mvpAllParams;
    };

} // NAV24

#endif //NAV24_PARAMETERSERVER_HPP
