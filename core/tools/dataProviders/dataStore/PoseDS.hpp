//
// Created by root on 5/16/21.
//

#ifndef NAV24_POSEDS_H
#define NAV24_POSEDS_H

#include <iostream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/any.hpp>

#include <glog/logging.h>

#include "Pose.hpp"
#include "TabularTextDS.hpp"


namespace NAV24 {

    class PoseDS : public TabularTextDS {
    public:
        //PoseDS() = default;
        PoseDS(const std::string &filePath, bool _qwFirst, bool _posFirst, double tsFactor);
        ~PoseDS() override;

        void setPosOrder(bool _posFirst) { posFirst = _posFirst; }
        void setQwOrder(bool _qwFirst) { qwFirst = _qwFirst; }

        unsigned long getNextChunk(size_t offset, unsigned long chunkSize, std::vector<PosePtr> &outData);
        unsigned long getNextChunk(double ts, std::vector<PosePtr> &outData, double tsFactor = 1.0);

        unsigned long getNumData() { return mvGtData.size(); }

        void incIdx() { mSIdx++; }
        void decIdx() { mSIdx--; }
        void resetIdx() { mSIdx = 0; }

        void reset() override;

    protected:
        boost::any parseLine(const std::string &evStr) override;
    private:
        std::vector<PosePtr> mvGtData;
        size_t mSIdx;
        // For EuRoC type data qw is the first in quat data
        // For Ethz public event data the ordering is reverse
        // For both first is p, then q
        bool posFirst;
        bool qwFirst;
    };

    typedef std::shared_ptr<PoseDS> PoseDS_Ptr;
    typedef std::unique_ptr<PoseDS> PoseDS_UPtr;

} // NAV24


#endif //NAV24_POSEDS_H
