//
// Created by masoud on 5/13/24.
//

#include <string>
#include <glog/logging.h>
#include <iomanip>

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
            CIRC,
            TIME
        } mTrajType;

        explicit FE_SerialTraj(const ChannelPtr& pChannel) : FrontEnd(pChannel), mTrajType(DEF) {}

        void receive(const MsgPtr &msg) override {

            if (msg) {
                int action = msg->getTargetId();
                if (action == FCN_SYS_RUN) {
                    this->run();
                }
                if (action == FCN_SER_READ) {
                    if (mTrajType == TIME) {
                        this->updateTimeStat(msg->getMessage());
                    }
                }
            }
        }

        [[nodiscard]] double getTsAvg() const { return tsAvg; }

        void setTrajType(TrajType trajType) { mTrajType = trajType; }

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

        void updateTimeStat(const string& tstr) {

            auto tt = chrono::time_point_cast<chrono::nanoseconds>(chrono::high_resolution_clock::now());
            long t1 = tt.time_since_epoch().count();
            int t0_idx = -1;
            istringstream iss{tstr};
            iss >> t0_idx;
            if (t0_idx >= 0 && mmTsMap.count(t0_idx) > 0) {

                long t0 = mmTsMap[t0_idx];
                if (mLastTs < 0) {
                    mLastTs = t1;
                    return;
                }
                else {
                    t0 = mLastTs;
                    mLastTs = t1;
                }
                long t_diff = t1 - t0;
                tsAvg = (static_cast<double>(t_diff) + static_cast<double>(cntTs) * tsAvg) / (static_cast<double>(cntTs) + 1);
                DLOG(INFO) << t0_idx << ": " << t1 << " (ns) - " << t0 << " (ns) = " << std::fixed << t_diff / 1e9 << " (s)\n";
                cntTs++;
                mmTsMap.erase(t0_idx);
            }
        }

        void run() override {
            FrontEnd::run();

            double runTimeSec = 40.0;
            auto t0 = chrono::high_resolution_clock::now();
            vector<float> vPoints(2);
            idxTs = 0;

            while (true) {

                stringstream locStr;

                if (mTrajType == QDES || mTrajType == CIRC) {
                    if (mTrajType == QDES) {
                        this->getNextTrajQDes(vPoints);
                    }
                    else if (mTrajType == CIRC) {
                        this->getNextTrajCircle(vPoints);
                    }
                    locStr << " " << vPoints[0] << " " << vPoints[1];
                }
                else if (mTrajType == TIME) {
                    auto tt = chrono::time_point_cast<chrono::nanoseconds>(chrono::high_resolution_clock::now());
                    long t0_send = tt.time_since_epoch().count();
                    mmTsMap.insert(make_pair(idxTs++, t0_send));
                    // can't just send long to Arduino
                    // so map these
                    locStr << " " << (idxTs - 1);
                }

                // Send a coord message to serial output
                auto msgSerial = make_shared<Message>(ID_TP_OUTPUT, Output::TOPIC,
                                                      FCN_SER_WRITE, locStr.str());
                mpChannel->publish(msgSerial);

                // Calculate loop time
                auto t1 = chrono::high_resolution_clock::now();
                auto duration = duration_cast<chrono::microseconds>(t1 - t0);
                double loopTime = static_cast<double>(duration.count()) / 1000000.0;
                //cout << "Loop time: " << loopTime << " (s)" << endl;

                if (loopTime >= runTimeSec) {
                    this->stop();
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
        long mLastTs = -1;
        long cntTs = 0;
        double tsAvg = 0.0;
        map<int, long> mmTsMap;
        int idxTs = 0;
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
    mpSystem->registerSubscriber(ID_TP_SDATA, mpFrontend);

    // Run broadcaster
    auto msgReqRun = make_shared<Message>(ID_CH_FE,FE::FE_SerialTraj::TOPIC, FCN_SYS_RUN);

    mpFrontend->setTrajType(NAV24::FE::FE_SerialTraj::TIME);
    mpFrontend->receive(msgReqRun);

    cout << "Average TR time: " << std::fixed << std::setprecision(9) << mpFrontend->getTsAvg() / 1e9 << " (s)\n";

    return 0;
}