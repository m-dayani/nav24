//
// Created by masoud on 4/30/24.
//

#include "Output.hpp"
#include "Serial.hpp"
#include "ImageViewer.hpp"
#include "MapViewer.hpp"
#include "System.hpp"


using namespace std;

namespace NAV24 {

    Output::Output(const ChannelPtr& pChannel) : MsgCallback(pChannel), mpInterface() {}

    void Output::receive(const MsgPtr &msg) {

        if (msg) {
            if (dynamic_pointer_cast<MsgConfig>(msg)) {
                this->setup(msg);
            }
            if (dynamic_pointer_cast<MsgRequest>(msg)) {
                this->handleRequest(msg);
            }
            if (msg->getTargetId() == FCN_SYS_STOP) {
                this->stop();
            }
        }
    }

    void Output::handleRequest(const NAV24::MsgPtr &msg) {

        if (msg && dynamic_pointer_cast<MsgRequest>(msg)) {
            auto msgReq = dynamic_pointer_cast<MsgRequest>(msg);
            auto senderCb = msgReq->getCallbackFun();
            if (senderCb) {
                if (msg->getTargetId() == FCN_SYS_RUN) {
                    auto pThread = make_shared<thread>(&Output::run, this);
                    auto msgRes = make_shared<MsgType<shared_ptr<thread>>>(ID_CH_SYS, pThread,
                                                                           System::TOPIC);
                    senderCb(msgRes);
                }
            }
        }
    }

    std::shared_ptr<Output> Output::getNewInstance(const ChannelPtr& pChannel, const ParamPtr &pParam) {

        SensorInterface ifDummy(pParam);

        // Unlike the camera sensors, the creation of outputs requires fetching some params
        OutputPtr pOutput;
        if (ifDummy.typeStr == "screen") {
            if (ifDummy.target == "image") {
                pOutput = make_shared<ImageViewer>(pChannel);
            }
            else if (ifDummy.target == "3d-graphics") {
                pOutput = make_shared<MapViewer>(pChannel);
            }
        }
        else if (ifDummy.typeStr == "serial") {
            pOutput = make_shared<Serial>(pChannel);
        }

        // load output params
        auto msgConfig = make_shared<MsgConfig>(ID_CH_PARAMS, pParam);
        pOutput->receive(msgConfig);

        return pOutput;
    }

    void Output::setup(const MsgPtr &msg) {

        if (msg && dynamic_pointer_cast<MsgConfig>(msg)) {
            auto msgConfig = dynamic_pointer_cast<MsgConfig>(msg);
            auto pParam = msgConfig->getConfig();
            if (pParam) {
                auto pOutName = find_param<ParamType<string>>("name", pParam);
                mName = (pOutName) ? pOutName->getValue() : "Output";

                mpInterface = make_shared<SensorInterface>(pParam);
            }
        }
    }


} // NAV24
