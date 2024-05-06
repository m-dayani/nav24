//
// Created by masoud on 4/30/24.
//

#include <glog/logging.h>

#include "Serial.hpp"


using namespace std;

namespace NAV24 {

    Serial::Serial(const ChannelPtr& pChannel) : Output(pChannel),
        mWriteBuffer(), mReadBuffer(), mMtxReadBuff(), mMtxWriteBuff() {}

    void Serial::receive(const MsgPtr &msg) {
        Output::receive(msg);

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
                        DVLOG(2) << "Serial::receive, unsupported action: " << msg->getTargetId() << "\n";
                        break;
                }
            }
        }
    }

    void Serial::setup(const MsgPtr &msg) {
        Output::setup(msg);

        if (mpInterface) {
            try {
                mpSerial = make_shared<serial::Serial>(mpInterface->target, mpInterface->port,
                                                       serial::Timeout::simpleTimeout(1000));
            }
            catch (const exception &e) {
                DLOG(WARNING) << e.what();
                mpSerial = nullptr;
            }
        }
    }

    void Serial::run() {

        if (!mpSerial) {
            DLOG(ERROR) << "Serial::run, serial interface is null\n";
            return;
        }

        while (!mbStop) {
            if (!mReadBuffer.empty()) {

            }
            if (!mWriteBuffer.empty()) {

            }
        }
    }

    void Serial::requestStop(const string &channel) {

    }
} // NAV24

