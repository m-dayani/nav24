//
// Created by masoud on 5/13/24.
//

#include <string>
#include <glog/logging.h>

#include "System.hpp"
#include "FrontEnd.hpp"
#include "Serial.hpp"

#define PI 3.14

using namespace std;
using namespace NAV24;

namespace NAV24::FE {

    class FE_SerialTraj : public FrontEnd {
    public:
        inline static const std::string TOPIC = "ObjTracking";

        enum TrajType {
            DEF,
            QDES,
            CIRC
        } mTrajType;

        explicit FE_SerialTraj(const ChannelPtr& pChannel) : FrontEnd(pChannel), mTrajType(QDES) {}

        void receive(const MsgPtr &msg) override {

            if (msg) {
                if (msg->getTargetId() == FCN_SYS_RUN) {
                    this->run();
                }
            }
        }

    protected:

        void getNextTrajQDes(vector<float>& vPoints) {

            if (vPoints.size() < 2) vPoints.resize(2);
            auto lspi = static_cast<float>(spi);

            vPoints[0] = Dam[0]  * sin(Freq  * (lspi) * ST);
            //dqdes[0] = Dam[0] * Freq  * cos(Freq  * (spi) * ST);
            //ddqdes[0] = -Dam[0] * Freq   * Freq  * sin(Freq  * (spi) * ST);
            vPoints[1] = Dam[1]  * sin(2 * Freq  * (lspi) * ST);
            //dqdes[1] = 2 * Dam[1] * Freq  * cos(2 * Freq  * (spi) * ST);
            //ddqdes[1] = -4 * Dam[1] * Freq   * Freq  * sin(2 * Freq  * (spi) * ST);
        }

        void getNextTrajCircle(vector<float>& vPoints) {

            auto lspi = static_cast<float>(spi);

            float x = Dam[0]  * sin(Freq  * (lspi) * ST);
            float y = Dam[1]  * sin(2 * Freq  * (lspi) * ST);

            auto c2=static_cast<float>((x*x+y*y-(0.64*0.64)-(0.4*0.4))/(2*0.4*0.64));
            float s2=sqrt(1-(c2*c2));

            float tetad1 =atan2(s2,c2);
            auto tetad0 =static_cast<float>(atan2(y,x)-atan2((0.4*sin(tetad1)),(0.64+(0.4*cos(tetad1)))));

            tetad1=static_cast<float>(tetad1*180/PI);
            tetad0=static_cast<float>(tetad0*180/PI);

            tetad0=-(tetad0-90);
            tetad1=-(tetad1-162);

            vPoints[0] = tetad0;
            vPoints[1] = tetad1;

            // todo: limit the spi for this
        }

        void run() override {
            FrontEnd::run();

            double runTimeSec = 40.0;
            auto t0 = chrono::high_resolution_clock::now();
            vector<float> vPoints(2);

            while (true) {

                if (mTrajType == QDES) {
                    this->getNextTrajQDes(vPoints);
                }
                else if (mTrajType == CIRC) {
                    this->getNextTrajCircle(vPoints);
                }

                // Send a coord message to serial output
                stringstream locStr;
                locStr << " " << vPoints[0] << " " << vPoints[1];
                auto msgSerial = make_shared<Message>(ID_TP_OUTPUT, Output::TOPIC,
                                                      FCN_SER_WRITE, locStr.str());
                mpChannel->publish(msgSerial);

                //cout << locStr.str() << endl;

                auto t1 = chrono::high_resolution_clock::now();
                auto duration = duration_cast<chrono::microseconds>(t1 - t0);
                double loopTime = static_cast<double>(duration.count()) / 1000000.0;
                //cout << "Loop time: " << loopTime << " (s)" << endl;

                if (loopTime >= runTimeSec) {
                    mMtxStop.lock();
                    mbStop = true;
                    mMtxStop.unlock();
                }

                if (this->isStopped()) {
                    break;
                }

                spi++;
            }
        }

        void setup(const MsgPtr &configMsg) override {}

        float Freq = 1, Dam[2] = {20, 20};
        float ts[1048]{}, ST = 0.02;
        int s = 0;
        int spi = 0;
        //float qdes[2] = {0, 0}, dqdes[2] = {0, 0}, ddqdes[2] = {0, 0};
        //float tetad[2] = {0, 0};
    };
}

int main([[maybe_unused]] int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    string confFile = "../../config/AUN_ARM1.yaml";
//    string saveFile = "../../config/AUN_ARM1.yaml";
    //shared_ptr <ParamReceiver> pParamRec = make_shared<ParamReceiver>();

    // Create the system
    shared_ptr <System> mpSystem = make_shared<System>();
    mpSystem->registerChannel(ID_CH_SYS, mpSystem);

    // Load settings
    MsgPtr msgLoadSettings = make_shared<Message>(ID_CH_SYS, System::TOPIC, FCN_LD_PARAMS, confFile);
    mpSystem->receive(msgLoadSettings);

    // Create trajectory broadcaster
    auto mpFrontend = make_shared<FE::FE_SerialTraj>(mpSystem);
    mpSystem->registerChannel(ID_CH_FE, mpFrontend);

    // Run broadcaster
    auto msgReqRun = make_shared<Message>(ID_CH_FE,FE::FE_SerialTraj::TOPIC, FCN_SYS_RUN);
    mpFrontend->receive(msgReqRun);

    return 0;
}