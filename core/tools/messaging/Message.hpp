//
// Created by root on 12/21/23.
//

#ifndef NAV24_MESSAGE_HPP
#define NAV24_MESSAGE_HPP

#include <string>
#include <memory>
#include <utility>
#include <mutex>

#include "Parameter.hpp"
#include "SensorData.hpp"


namespace NAV24 {

#define DEF_TOPIC ""
#define DEF_ACTION 0
#define DEF_MSG ""
#define DEF_CAT 0

// Category IDs for regular message passing
#define ID_CH_PARAMS 1
#define ID_CH_DS 2
#define ID_CH_OUTPUT 3
#define ID_CH_SENSORS 4
#define ID_CH_NOTIFY_CHANGE 5
#define ID_CH_ATLAS 6
#define ID_CH_TRAJECTORY 7
#define ID_CH_SYS 8
#define ID_CH_FE 9
#define ID_CH_OP 10

// Category IDs for publisher/listeners
#define ID_TP_SDATA 21
#define ID_TP_OP 22
#define ID_TP_FE 23
#define ID_TP_OUTPUT 24

    class Message {
    public:
        Message() : targetId(DEF_ACTION), chId(DEF_CAT) {}

        // This is the preferred constructor
        explicit Message(const int catId, std::string _topic = DEF_TOPIC,
                         const int action = DEF_ACTION, std::string _msg = DEF_MSG) :
            topic(std::move(_topic)), msg(std::move(_msg)), targetId(action), chId(catId) {}

        std::string getMessage() { return msg; }
        void setMessage(const std::string& _msg) { this->msg = _msg; }
        std::string getTopic() { return topic; }
        void setTopic(const std::string& _topic) { this->topic = _topic; }
        [[nodiscard]] int getTargetId() const { return targetId; }
        void setTargetId(int id) { this->targetId = id; }
        [[nodiscard]] int getChId() const { return chId; }
        void setChId(int id) { this->chId = id; }

        virtual std::string toString() { return msg; }

    protected:
        std::string topic;
        std::string msg;
        int targetId;
        int chId;
    };
    typedef std::shared_ptr<Message> MsgPtr;
    typedef std::weak_ptr<Message> MsgPtrW;

    class Channel;
    typedef std::shared_ptr<Channel> ChannelPtr;
    typedef std::weak_ptr<Channel> ChannelPtrW;

    /* -------------------------------------------------------------------------------------------------------------- */

    class MsgCallback {
    public:
        MsgCallback() : mbStop(false), mMtxStop(), mpChannel() {}
        explicit MsgCallback(const ChannelPtr& pChannel) : MsgCallback() { mpChannel = pChannel; }

        virtual void receive(const MsgPtr& msg) = 0;

        [[nodiscard]] std::string getName() const { return mName; }
    protected:
        virtual void setup(const MsgPtr& configMsg) = 0;
        virtual void handleRequest(const MsgPtr& reqMsg) = 0;

        virtual void run() = 0;
        virtual void stop() {
            mMtxStop.lock();
            mbStop = true;
            mMtxStop.unlock();
        };
        virtual bool isStopped() {
            mMtxStop.lock();
            bool isStopped = mbStop;
            mMtxStop.unlock();
            return isStopped;
        }

    protected:
        std::string mName;

        ChannelPtr mpChannel;

        bool mbStop;
        std::mutex mMtxStop;
    };
    typedef std::shared_ptr<MsgCallback> MsgCbPtr;

    /* -------------------------------------------------------------------------------------------------------------- */

    class Channel {
    public:
        virtual void send(const MsgPtr& message) = 0;
        virtual void publish(const MsgPtr& message) = 0;
        virtual void registerPublisher(int chId, const MsgCbPtr& callback) = 0;
        virtual void registerSubscriber(int chId, const MsgCbPtr& callback) = 0;
        virtual void registerChannel(int chId, const MsgCbPtr& callback) = 0;
        virtual void unregisterChannel(int chId, const MsgCbPtr& callback) = 0;
    };

    /* -------------------------------------------------------------------------------------------------------------- */

    class MsgRequest : public Message {
    public:
        MsgRequest(const int catId, MsgCbPtr  callback, const std::string& topic = DEF_TOPIC,
                   const int targetId = DEF_ACTION, const std::string& msg = DEF_MSG) :
            Message(catId, topic, targetId, msg), mpCallback(std::move(callback)) {}

        MsgCbPtr getCallback() { return mpCallback; }
        void setCallback(const MsgCbPtr& callback) { mpCallback = callback; }

    protected:
        MsgCbPtr mpCallback;
    };
    typedef std::shared_ptr<MsgRequest> MsgReqPtr;

    class MsgConfig : public Message {
    public:
        MsgConfig(const int catId, ParamPtr paramPtr, const std::string& topic = DEF_TOPIC,
                  const int targetId = DEF_ACTION, const std::string& msg = DEF_MSG) :
            Message(catId, topic, targetId, msg), mConfig(std::move(paramPtr)) {}

        ParamPtr getConfig() { return mConfig; }
        void setConfig(const ParamPtr& config) { mConfig = config; }

        std::string toString() override;

    protected:
        ParamPtr mConfig;
    };
    typedef std::shared_ptr<MsgConfig> MsgConfigPtr;

    class MsgSensorData : public Message {
    public:
        MsgSensorData(const int catId, SensorDataPtr  dataPtr, const std::string& topic = DEF_TOPIC,
                      const int targetId = DEF_ACTION, const std::string& msg = DEF_MSG) :
            Message(catId, topic, targetId, msg), mSensorData(std::move(dataPtr)) {}

        SensorDataPtr getData() { return mSensorData; }
        void setData(const SensorDataPtr& sData) { mSensorData = sData; }

    protected:
        SensorDataPtr mSensorData;
    };

    template<typename T>
    class MsgType : public Message {
    public:
        MsgType(const int catId, const T& data, const std::string& topic = DEF_TOPIC,
                const int targetId = DEF_ACTION, const std::string& msg = DEF_MSG) :
            Message(catId, topic, targetId, msg), mData(data) {}

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
