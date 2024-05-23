//
// Created by masoud on 4/30/24.
//

#include <glog/logging.h>

#include "Serial.hpp"
#include "System.hpp"
#include "FrontEnd.hpp"


using namespace std;

namespace NAV24 {

    Serial::Serial(const ChannelPtr& pChannel) : Output(pChannel),
        mWriteBuffer(), mReadBuffer(), mMtxReadBuff(), mMtxWriteBuff() {}

    void Serial::receive(const MsgPtr &msg) {
        Output::receive(msg);

        if (msg) {
            string topic = msg->getTopic();
            int catId = msg->getChId();
            if (catId == ID_CH_OUTPUT || topic == Output::TOPIC || topic == Serial::TOPIC) {

                switch (msg->getTargetId()) {
                    case FCN_SER_OPEN:
                        // todo: implement open and close, considering mpSerial might already be open
                        break;
                    case FCN_SER_WRITE: {
//                        if (mpSerial) {
//                            size_t bytes_wrote = mpSerial->write(msg->getMessage());
//                        }
                        mMtxWriteBuff.lock();
                        mWriteBuffer.push(msg->getMessage());
                        mMtxWriteBuff.unlock();
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
            if (topic == System::TOPIC && msg->getTargetId() == FCN_SYS_STOP) {
                this->stop();
            }
        }
    }

    void Serial::setup(const MsgPtr &msg) {
        Output::setup(msg);

        if (mpInterface) {
            try {
                mpSerial = make_shared<serial::Serial>(mpInterface->target, mpInterface->port,
                                                       serial::Timeout::simpleTimeout(100));
                // when you instantiate serial class, it opens automatically
                if(mpSerial && mpSerial->isOpen()) {
                    DLOG(INFO) << "Serial::setup, serial: " << mpInterface->target << ", interface opened successfully\n";
                }
            }
            catch (const exception &e) {
                DLOG(WARNING) << e.what();
                if (mpSerial && mpSerial->isOpen()) {
                    mpSerial->close();
                }
                mpSerial = nullptr;
            }
        }
    }

    void Serial::run() {

        if (!mpSerial) {
            DLOG(ERROR) << "Serial::run, serial interface is null\n";
            return;
        }

        //int count = 0;
        while (!mbStop) {
            if (!mReadBuffer.empty()) {

            }
            std::string msg2write;
            mMtxWriteBuff.lock();
            if (!mWriteBuffer.empty()) {
                msg2write = mWriteBuffer.front();
                mWriteBuffer.pop();
                DVLOG(2) << "Serial::run, msg to write: " << msg2write << "\n";
            }
            mMtxWriteBuff.unlock();

            if (!msg2write.empty()) {

                size_t bytes_wrote = mpSerial->write(msg2write);
                DVLOG(2) << "Serial::run, wrote " << bytes_wrote << " bytes\n";

                string result = mpSerial->read(msg2write.length()+1);

                // send the message
                if (!result.empty()) {
                    auto msgRes = make_shared<Message>(ID_TP_SDATA, FE::FrontEnd::TOPIC,
                                                       FCN_SER_READ, result);
                    auto t0 = chrono::high_resolution_clock::now().time_since_epoch().count();
                    DLOG(INFO) << "Serial received: " << result << ", at " << t0 << "\n";
                    mpChannel->publish(msgRes);
                }

//                cout << "Iteration: " << count << ", Bytes written: ";
//                cout << bytes_wrote << ", Bytes read: ";
//                cout << result.length() << ", String read: " << result << endl;
            }
            //count++;
        }
    }

    void Serial::requestStop(const string &channel) {
        this->stop();
    }

    void Serial::stop() {
        MsgCallback::stop();
        mpSerial->close();
    }
} // NAV24

