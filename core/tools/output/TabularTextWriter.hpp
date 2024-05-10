//
// Created by root on 5/28/21.
//

#ifndef NAV24_TABULARTEXTWRITER_H
#define NAV24_TABULARTEXTWRITER_H

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

#include "SharedQueue.hpp"
#include "IMU.hpp"
#include "Pose.hpp"

namespace NAV24 {

    template<typename T>
    class TabularTextWriter {
    public:

        explicit TabularTextWriter(const std::string& outFileName) :
                mFileStream(outFileName, std::ofstream::out), mbStop(false) {

            mpDataQueue = std::make_shared<SharedQueue<T>>();
        }
        virtual ~TabularTextWriter() {
            if (mFileStream.is_open())
                mFileStream.close();
        }

        void run() {

            if (!mFileStream.is_open()) {
                return;
            }
            while (!mbStop && mFileStream) {

                if (!mpDataQueue->empty()) {

                    T pData = mpDataQueue->front();

                    this->write(mFileStream, pData);

                    mpDataQueue->pop();
                }
            }
        }

        virtual void write(std::ofstream& ofs, const T& data) = 0;

        std::ofstream mFileStream;
        std::shared_ptr<SharedQueue<T>> mpDataQueue;
        bool mbStop;
    };

    class IMU_Writer : public TabularTextWriter<IMU_DataPtr> {
    public:
        explicit IMU_Writer(const std::string& outFileName) : TabularTextWriter<IMU_DataPtr>(outFileName) {}

        void write(std::ofstream& ofs, const IMU_DataPtr& pImuData) override {

            mFileStream << std::fixed << static_cast<long>(pImuData->ts * 1e9) << " " << std::setprecision(9);
            for (const float& g : pImuData->gyro) {
                mFileStream << g << " ";
            }
            for (const float& a : pImuData->accel) {
                mFileStream << a << " ";
            }
            mFileStream << std::endl;
        }

    };

    class GtPoseWriter: public TabularTextWriter<PosePtr> {
    public:
        explicit GtPoseWriter(const std::string& outFileName) : TabularTextWriter<PosePtr>(outFileName) {}

        void write(std::ofstream& ofs, const PosePtr& pGtPose) override {

            mFileStream << std::fixed << static_cast<long>(pGtPose->ts * 1e9) << " " << std::setprecision(9);
            for (const float& pos : pGtPose->p) {
                mFileStream << pos << " ";
            }
            for (const float& quat : pGtPose->q) {
                mFileStream << quat << " ";
            }
            mFileStream << std::endl;
        }
    };

} // NAV24


#endif //NAV24_TABULARTEXTWRITER_H
