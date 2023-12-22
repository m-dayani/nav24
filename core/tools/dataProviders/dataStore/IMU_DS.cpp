//
// Created by root on 5/16/21.
//

#include "IMU_DS.hpp"

using namespace std;

namespace NAV24 {

    IMU_DS::IMU_DS(const string &filePath, bool _gFirst, const double tsFactor) : gFirst(_gFirst) {

        boost::filesystem::path imuFile(filePath);
        mTsFactor = tsFactor;

        try {
            // Check image file names
            if (exists(imuFile) && is_regular_file(imuFile))
            {
                this->mDataPath = filePath;
                this->mPathState = PathState::FILE;

                if (checkExtension(imuFile, ".txt")) {
                    mLoadState = LoadState::READY;
                }
                else if (checkExtension(imuFile, ".csv")) {
                    mPathState = PathState::FILE_CSV;
                    mLoadState = LoadState::READY;
                }
                else {
                    mLoadState = LoadState::BAD_PATH;
                }

                if (mLoadState == LoadState::READY) {
                    this->openFile();
                    this->getTxtData(0, mvImuData);
                    if (!mvImuData.empty())
                        mLoadState = LoadState::GOOD;
                    else
                        mLoadState = LoadState::BAD_DATA;
                }
            }
            else {
                cerr << imuFile << " does not exist\n";
            }
        }
        catch (const boost::filesystem::filesystem_error& ex)
        {
            cerr << ex.what() << '\n';
        }
    }

    IMU_DS::~IMU_DS() {
        if (mTxtDataFile && mTxtDataFile.is_open())
        {
            mTxtDataFile.close();
        }
    }

    boost::any IMU_DS::parseLine(const string &evStr) {

        std::istringstream stream(evStr);

        double ts = 0.0;
        char c;
        float gx = 0.f, gy = 0.f, gz = 0.f, ax = 0.f, ay = 0.f, az = 0.f;

        if (mPathState == PathState::FILE) {
            if (this->gFirst)
                stream >> ts >> gx >> gy >> gz >> ax >> ay >> az;
            else
                stream >> ts >> ax >> ay >> az >> gx >> gy >> gz;
        }
        else if (mPathState == PathState::FILE_CSV) {
            if (this->gFirst)
                stream >> ts >> c >> gx >> c >> gy >> c >> gz >> c >> ax >> c >> ay >> c >> az;
            else
                stream >> ts >> c >> ax >> c >> ay >> c >> az >> c >> gx >> c >> gy >> c >> gz;
        }

        IMU_DataPtr imuData = make_shared<IMU_Data>(ts/mTsFactor, gx, gy, gz, ax, ay, az);
        return boost::any(imuData);
    }

    unsigned long IMU_DS::getNextChunk(size_t offset, unsigned long chunkSize, vector<IMU_DataPtr> &outData) {

        if (offset + chunkSize >= mvImuData.size())
            return 0;
        outData.resize(chunkSize);
        unsigned long dtCount = 0;
        for (; dtCount < chunkSize; dtCount++) {
            outData[dtCount] = mvImuData[offset+dtCount];
        }
        return dtCount;
    }

    unsigned long IMU_DS::getNextChunk(double tsEnd, vector<IMU_DataPtr> &vImuMeas, const double tsFactor) {

        unsigned long imuCnt = 0;
        if (mSIdx < 0 || mSIdx >= mvImuData.size())
            return imuCnt;

        const double invTsFactor = 1.0 / tsFactor;

        while (mSIdx < mvImuData.size() && mvImuData[mSIdx]->ts * invTsFactor <= tsEnd) {

            vImuMeas.push_back(mvImuData[mSIdx]);
            this->incIdx();
            imuCnt++;
        }
        return imuCnt;
    }

    void IMU_DS::reset() {

        mSIdx = 0;
    }

} // NAV24
