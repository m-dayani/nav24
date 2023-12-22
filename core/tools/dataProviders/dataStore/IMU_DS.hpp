//
// Created by root on 5/16/21.
//

#ifndef NAV24_IMU_DS_H
#define NAV24_IMU_DS_H

#include <iostream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/any.hpp>

#include <glog/logging.h>

#include "IMU.hpp"
#include "TabularTextDS.hpp"


namespace NAV24 {

    class IMU_DS : public TabularTextDS {
    public:
        //IMU_DS() = default;
        IMU_DS(const std::string &filePath, bool _gFirst, double tsFactor);
        ~IMU_DS() override;

        void setGyroOrder(bool _gFirst) { gFirst = _gFirst; }

        unsigned long getNextChunk(size_t offset, unsigned long chunkSize, std::vector<IMU_DataPtr> &outData);
        // This will change object's mSIdx internally
        // Be careful about ts units, some datasets are in sec others are in nsecs!
        unsigned long getNextChunk(double tsEndNsec, std::vector<IMU_DataPtr> &vImuMeas, double tsFactor = 1.0);
        unsigned long getNumData() { return mvImuData.size(); }

        void incIdx() { mSIdx++; }
        void decIdx() { mSIdx--; }
        void resetIdx() { mSIdx = 0; }

        void reset() override;

    protected:
        boost::any parseLine(const std::string &evStr) override;
    private:
        std::vector<IMU_DataPtr> mvImuData;
        size_t mSIdx = 0;
        // For EuRoC type data gyro is first then accel
        // For Ethz public event data the ordering is reverse
        bool gFirst = true;
    };

    typedef std::shared_ptr<IMU_DS> IMU_DS_Ptr;
    typedef std::unique_ptr<IMU_DS> IMU_DS_UPtr;

} // NAV24


#endif //NAV24_IMU_DS_H
