//
// Created by masoud on 4/30/24.
//

#include "Serial.hpp"

#include <utility>
#include <glog/logging.h>


using namespace std;

namespace NAV24 {

    Serial::Serial(ChannelPtr pChannel) : mpChannel(std::move(pChannel)) {

//        mpSerial = make_shared<serial::Serial>("/dev/ttyUSB0", SER_DEF_BAUD,
//                                               serial::Timeout::simpleTimeout(1000));
    }

    void Serial::receive(const MsgPtr &msg) {

        if (msg) {
            string topic = msg->getTopic();
            if (topic == Output::TOPIC || topic == Serial::TOPIC) {

                switch (msg->getTargetId()) {
                    case FCN_SER_OPEN:
                        break;
                    case FCN_SER_WRITE: {
                        if (mpSerial) {
                            size_t bytes_wrote = mpSerial->write(msg->getMessage());
                        }
                    }
                        break;
                    case FCN_SER_READ: {
                        if (mpSerial) {
                            string buffer;
                            buffer.resize(128);
                            string result = mpSerial->read(buffer.length() + 1);
                        }
                    }
                        break;
                    default:
                        DLOG(WARNING) << "Serial::receive, unsupported action: " << msg->getTargetId() << "\n";
                        break;
                }
            }
            if (dynamic_pointer_cast<MsgConfig>(msg)) {
                this->initialize(msg);
            }
        }
    }

    void Serial::initialize(const MsgPtr &msg) {

        if (msg && dynamic_pointer_cast<MsgConfig>(msg)) {
            auto msgConfig = dynamic_pointer_cast<MsgConfig>(msg);
            auto pParam = msgConfig->getConfig();
            if (pParam) {
                auto pOutName = find_param<ParamType<string>>("name", pParam);
                mName = (pOutName) ? pOutName->getValue() : "OutputSerial0";
                auto pIcType = find_param<ParamType<string>>("interface/type", pParam);
                string icType = (pIcType) ? pIcType->getValue() : "serial";
                auto pIcTarget = find_param<ParamType<string>>("interface/target", pParam);
                string icTarget = (pIcTarget) ? pIcTarget->getValue() : "/dev/ttyUSB0";
                auto pIcPort = find_param<ParamType<int>>("interface/port", pParam);
                int icPort = (pIcPort) ? pIcPort->getValue() : SER_DEF_BAUD;

                // todo: make interface type consistent
                mpInterface = make_shared<SensorInterface>(SensorInterface::InterfaceType::DEFAULT,
                                                           icTarget, icPort);

                try {
                    mpSerial = make_shared<serial::Serial>(icTarget, icPort,
                                                           serial::Timeout::simpleTimeout(1000));
                }
                catch (const exception& e) {
                    DLOG(WARNING) << e.what();
                    mpSerial = nullptr;
                }
            }
        }
    }


} // NAV24

