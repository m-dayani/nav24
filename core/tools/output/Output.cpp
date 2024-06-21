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
        }
    }

    void Output::handleRequest(const NAV24::MsgPtr &msg) {

        if (msg && dynamic_pointer_cast<MsgRequest>(msg)) {
            auto msgReq = dynamic_pointer_cast<MsgRequest>(msg);
            auto sender = msgReq->getCallback();
            if (sender) {
                if (msg->getTargetId() == FCN_SYS_RUN) {
                    auto pThread = make_shared<thread>(&Output::run, this);
                    auto msgRes = make_shared<MsgType<shared_ptr<thread>>>(ID_CH_SYS, pThread,
                                                                           System::TOPIC);
                    sender->receive(msgRes);
                }
            }
        }
    }

    std::shared_ptr<Output> Output::getNewInstance(const ParamPtr &pParam, const ChannelPtr& pChannel) {

        // todo: optimize parameter retrieval
        auto pOutName = find_param<ParamType<string>>("name", pParam);
        string outName = (pOutName) ? pOutName->getValue() : "Output0";
        auto pIfType = find_param<ParamType<string>>("interface/type", pParam);
        string ifType = (pIfType) ? pIfType->getValue() : "";
        auto pIfTarget = find_param<ParamType<string>>("interface/target", pParam);
        string ifTarget = (pIfTarget) ? pIfTarget->getValue() : "";

        OutputPtr pOutput;
        if (ifType == "screen") {
            if (ifTarget == "image") {
                pOutput = make_shared<ImageViewer>(pChannel);
            }
            else if (ifTarget == "3d-graphics") {
                pOutput = make_shared<MapViewer>(pChannel);
            }
        }
        else if (ifType == "serial") {
            pOutput = make_shared<Serial>(pChannel);
        }

        return pOutput;
    }

    void Output::setup(const MsgPtr &msg) {

        if (msg && dynamic_pointer_cast<MsgConfig>(msg)) {
            auto msgConfig = dynamic_pointer_cast<MsgConfig>(msg);
            auto pParam = msgConfig->getConfig();
            if (pParam) {
                auto pOutName = find_param<ParamType<string>>("name", pParam);
                mName = (pOutName) ? pOutName->getValue() : "name";
                auto pIcType = find_param<ParamType<string>>("interface/type", pParam);
                string icType = (pIcType) ? pIcType->getValue() : "type";
                auto pIcTarget = find_param<ParamType<string>>("interface/target", pParam);
                string icTarget = (pIcTarget) ? pIcTarget->getValue() : "target";
                auto pIcPort = find_param<ParamType<int>>("interface/port", pParam);
                int icPort = (pIcPort) ? pIcPort->getValue() : 0;

                // todo: make interface type consistent
                mpInterface = make_shared<SensorInterface>(SensorInterface::InterfaceType::DEFAULT,
                                                           icTarget, icPort);
            }
        }
    }


} // NAV24
