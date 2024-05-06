//
// Created by root on 12/21/23.
//

#ifndef NAV24_MESSAGE_HPP
#define NAV24_MESSAGE_HPP

#include <string>
#include <memory>
#include <utility>


#include "Parameter.hpp"
#include "SensorData.hpp"


namespace NAV24 {

    class Message {
    public:
        Message() : topic(), msg(), targetId() {}

        explicit Message(std::string  _topic, std::string  _msg = "", const int _targetId = 0) :
                topic(std::move(_topic)), msg(std::move(_msg)), targetId(_targetId) {}

        std::string getMessage() { return msg; }
        void setMessage(const std::string& _msg) { this->msg = _msg; }
        std::string getTopic() { return topic; }
        void setTopic(const std::string& _topic) { this->topic = _topic; }
        [[nodiscard]] int getTargetId() const { return targetId; }
        void setTargetId(int id) { this->targetId = id; }
        //int getChannelId() { return chId; }
        //void setChannelId(int id) { this->chId = id; }

        virtual std::string toString() { return msg; }

    protected:
        std::string topic;
        std::string msg;
        int targetId;
        //int chId;
    };
    typedef std::shared_ptr<Message> MsgPtr;

    /* -------------------------------------------------------------------------------------------------------------- */

    class MsgCallback {
    public:
        MsgCallback() : mbStop(false), mMtxStop() {}
        virtual void receive(const MsgPtr& msg) = 0;
    protected:
        virtual void setup(const MsgPtr& configMsg) = 0;
        virtual void handleRequest(const MsgPtr& reqMsg) = 0;
        virtual void run() = 0;
        virtual void stop() { mbStop = true; };
    protected:
        bool mbStop;
        std::mutex mMtxStop;
    };
    typedef std::shared_ptr<MsgCallback> MsgCbPtr;

    /* -------------------------------------------------------------------------------------------------------------- */

    class Channel;
    typedef std::shared_ptr<Channel> ChannelPtr;
    class Channel {
    public:
        virtual void publish(const MsgPtr& message) = 0;
        //virtual void receive(const Message& message) = 0; -> this is implemented by the request class
        virtual void registerChannel(const MsgCbPtr& callback, const std::string& topic) = 0;
        virtual void unregisterChannel(const MsgCbPtr& callback, const std::string& topic) = 0;
    };

    /* -------------------------------------------------------------------------------------------------------------- */

    class MsgRequest : public Message {
    public:
        MsgRequest(const std::string& topic, MsgCbPtr callback) : Message(topic), mpCallback(std::move(callback)) {}
        MsgRequest(const std::string& topic, const std::string& msg, const int targetId, MsgCbPtr  callback) :
            Message(topic, msg, targetId), mpCallback(std::move(callback)) {}

        MsgCbPtr getCallback() { return mpCallback; }
        void setCallback(const MsgCbPtr& callback) { mpCallback = callback; }

    protected:
        MsgCbPtr mpCallback;
    };
    typedef std::shared_ptr<MsgRequest> MsgReqPtr;

    class MsgConfig : public Message {
    public:
        MsgConfig(const std::string& topic, ParamPtr paramPtr) :
            Message(topic), mConfig(std::move(paramPtr)) {}

        ParamPtr getConfig() { return mConfig; }
        void setConfig(const ParamPtr& config) { mConfig = config; }

        std::string toString() override;

    protected:
        ParamPtr mConfig;
    };
    typedef std::shared_ptr<MsgConfig> MsgConfigPtr;

    class MsgSensorData : public Message {
    public:
        MsgSensorData(const std::string& topic, SensorDataPtr  dataPtr) :
            Message(topic), mSensorData(std::move(dataPtr)) {}

        SensorDataPtr getData() { return mSensorData; }
        void setData(const SensorDataPtr& sData) { mSensorData = sData; }

    protected:
        SensorDataPtr mSensorData;
    };

    template<typename T>
    class MsgType : public Message {
    public:
        MsgType(const std::string& topic, const T& data) : Message(topic), mData(data) {}
        MsgType(const std::string& topic, const int target, const T& data) : Message(topic, "", target), mData(data) {}
        MsgType(const std::string& topic, const std::string& msgStr, const int target, const T& data) :
            Message(topic, msgStr, target), mData(data) {}

        T& getData() { return mData; }
        void setData(const T& data) { mData = data; }

        std::string toString() override {
            return Message::toString();
        }

    protected:
        T mData;
    };
}

#endif //NAV24_MESSAGE_HPP
