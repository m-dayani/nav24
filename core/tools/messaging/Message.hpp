//
// Created by root on 12/21/23.
//

#ifndef NAV24_MESSAGE_HPP
#define NAV24_MESSAGE_HPP

#include <string>
#include <memory>


#include "Parameter.hpp"


namespace NAV24 {

    class Message {
    public:
        Message() : topic(), msg(), targetId(), chId() {}

        Message(const std::string& _topic, const std::string& _msg = "", const int _targetId = 0, const int _chId = 0) :
                topic(_topic), msg(_msg), targetId(_targetId), chId(_chId) {}

        virtual std::string getMessage() { return msg; }
        virtual void setMessage(const std::string& _msg) { this->msg = _msg; }
        std::string getTopic() { return topic; }
        void setTopic(const std::string& _topic) { this->topic = _topic; }
        int getTargetId() { return targetId; }
        void setTargetId(int id) { this->targetId = id; }
        int getChannelId() { return chId; }
        void setChannelId(int id) { this->chId = id; }

    protected:
        std::string topic;
        std::string msg;
        int targetId;
        int chId;
    };
    typedef std::shared_ptr<Message> MsgPtr;

    class MsgCallback {
    public:
        virtual void receive(const MsgPtr& msg) = 0;
    protected:
        //virtual void setup(const MsgPtr& configMsg) = 0;
        //virtual void handleRequest(const MsgPtr& reqMsg) = 0;
    };
    typedef std::shared_ptr<MsgCallback> MsgCbPtr;

    class MsgRequest : public Message {
    public:
        MsgRequest(const std::string& topic, const MsgCbPtr& callback) : Message(topic), mpCallback(callback) {}
        MsgRequest(const std::string& topic, const std::string& msg, const int targetId, const MsgCbPtr& callback) :
            Message(topic, msg, targetId), mpCallback(callback) {}
        MsgCbPtr getCallback() { return mpCallback; }
    protected:
        MsgCbPtr mpCallback;
    };
    typedef std::shared_ptr<MsgRequest> MsgReqPtr;

    class MsgConfig : public Message {
    public:
        MsgConfig(const std::string& topic, const ParamPtr& paramPtr) : Message(topic), mConfig(paramPtr) {}

        ParamPtr getConfig() { return mConfig; }

        std::string getMessage() override { return mConfig->printStr(); }
    protected:
        ParamPtr mConfig;
    };
    typedef std::shared_ptr<MsgConfig> MsgConfigPtr;

    /*class MsgSensorData : public Message {
    public:
        MsgSensorData(const std::string& topic, const SensorDataPtr& dataPtr) : Message(topic), mSensorData(dataPtr) {}

    protected:
        SensorDataPtr mSensorData;
    };*/

}

#endif //NAV24_MESSAGE_HPP
