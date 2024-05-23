//
// Created by masoud on 5/13/24.
//

#include <string>
#include <glog/logging.h>

#include "System.hpp"
#include "ParameterServer.hpp"
#include "FrontEnd.hpp"
#include "Serial.hpp"

#define PI 3.14

using namespace std;
using namespace NAV24;

namespace NAV24::FE {

    class FE_SerialTraj : public FrontEnd {
    public:
        inline static const std::string TOPIC = "ObjTracking";

        explicit FE_SerialTraj(const ChannelPtr& pChannel) : FrontEnd(pChannel) {}

        void receive(const MsgPtr &msg) override {

            if (msg) {
                if (msg->getTargetId() == FCN_SYS_RUN) {
                    this->run();
                }
            }
        }

    protected:

        void run() override {
            FrontEnd::run();

            double runTimeSec = 40.0;
            auto t0 = chrono::high_resolution_clock::now();
            while (true) {

                qdes[0] = Dam[0]  * sin(Freq  * (spi) * ST);
                dqdes[0] = Dam[0] * Freq  * cos(Freq  * (spi) * ST);
                ddqdes[0] = -Dam[0] * Freq   * Freq  * sin(Freq  * (spi) * ST);
                qdes[1] = Dam[1]  * sin(2 * Freq  * (spi) * ST);
                dqdes[1] = 2 * Dam[1] * Freq  * cos(2 * Freq  * (spi) * ST);
                ddqdes[1] = -4 * Dam[1] * Freq   * Freq  * sin(2 * Freq  * (spi) * ST);

                float x = Dam[0]  * sin(Freq  * (spi) * ST);
                float y = Dam[1]  * sin(2 * Freq  * (spi) * ST);

                float c2=(x*x+y*y-(0.64*0.64)-(0.4*0.4))/(2*0.4*0.64);
                float s2=sqrt(1-(c2*c2));

                tetad[1] =atan2(s2,c2);
                tetad[0] =atan2(y,x)-atan2((0.4*sin(tetad[1])),(0.64+(0.4*cos(tetad[1]))));

                tetad[1]=tetad[1]*180/PI;
                tetad[0]=tetad[0]*180/PI;

                tetad[0]=-(tetad[0]-90);
                tetad[1]=-(tetad[1]-162);


                // Send a coord message to serial output
                stringstream locStr;
                locStr << " " << qdes[0] << " " << qdes[1];
                auto msgSerial = make_shared<Message>(ID_TP_OUTPUT, Output::TOPIC,
                                                      FCN_SER_WRITE, locStr.str());
                mpChannel->publish(msgSerial);

                //cout << locStr.str() << endl;

                auto t1 = chrono::high_resolution_clock::now();
                auto duration = duration_cast<chrono::microseconds>(t1 - t0);
                double loopTime = static_cast<double>(duration.count()) / 1000000.0;
                //cout << "Loop time: " << loopTime << " (s)" << endl;

                if (this->isStopped() || loopTime >= runTimeSec) {
                    break;
                }
                spi += 1.f;
            }
        }

        void setup(const MsgPtr &configMsg) override {}

        float Freq = 1, Dam[2] = {20, 20};
        float ts[1048]{}, ST = 0.02;
        int s = 0;
        float spi = 0.f;
        float qdes[2] = {0, 0}, dqdes[2] = {0, 0}, ddqdes[2] = {0, 0};
        float tetad[2] = {0, 0};
    };
}

int main([[maybe_unused]] int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    string confFile = "../../config/AUN_ARM1.yaml";
    string saveFile = "../../config/AUN_ARM1.yaml";
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