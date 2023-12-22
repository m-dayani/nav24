//
// Created by root on 5/16/21.
//

#include "PoseDS.hpp"

using namespace std;

namespace NAV24 {

    PoseDS::PoseDS(const string &filePath, bool _qwFirst, bool _posFirst, const double tsFactor) :
            mSIdx(0), posFirst(_posFirst), qwFirst(_qwFirst) {

        boost::filesystem::path gtFile(filePath);
        mTsFactor = tsFactor;

        try {
            // Check image file names
            if (exists(gtFile) && is_regular_file(gtFile))
            {
                this->mDataPath = filePath;
                this->mPathState = PathState::FILE;

                if (checkExtension(gtFile, ".txt")) {
                    mLoadState = LoadState::READY;
                }
                else if (checkExtension(gtFile, ".csv")) {
                    mPathState = PathState::FILE_CSV;
                    mLoadState = LoadState::READY;
                }
                else {
                    mLoadState = LoadState::BAD_PATH;
                }

                if (mLoadState == LoadState::READY) {
                    this->openFile();
                    this->getTxtData(0, mvGtData);
                    if (!mvGtData.empty())
                        mLoadState = LoadState::GOOD;
                    else
                        mLoadState = LoadState::BAD_DATA;
                }
            }
            else {
                cerr << gtFile << " does not exist\n";
            }
        }
        catch (const boost::filesystem::filesystem_error& ex)
        {
            cerr << ex.what() << '\n';
        }
    }

    PoseDS::~PoseDS() {
        if (mTxtDataFile && mTxtDataFile.is_open())
        {
            mTxtDataFile.close();
        }
    }

    boost::any PoseDS::parseLine(const string &evStr) {

        std::istringstream stream(evStr);

        double ts = 0.0;
        char c;
        float qw = 0.f, qx = 0.f, qy = 0.f, qz = 0.f, px = 0.f, py = 0.f, pz = 0.f;

        if (mPathState == PathState::FILE) {
            if (this->qwFirst) {
                if (this->posFirst)
                    stream >> ts >> px >> py >> pz >> qw >> qx >> qy >> qz;
                else
                    stream >> ts >> qw >> qx >> qy >> qz >> px >> py >> pz;
            }
            else {
                if (this->posFirst)
                    stream >> ts >> px >> py >> pz >> qx >> qy >> qz >> qw;
                else
                    stream >> ts >> qx >> qy >> qz >> qw >> px >> py >> pz;
            }
        }
        else if (mPathState == PathState::FILE_CSV) {
            if (this->qwFirst) {
                if (this->posFirst)
                    stream >> ts >> c >> px >> c >> py >> c >> pz >> c >> qw >> c >> qx >> c >> qy >> c >> qz;
                else
                    stream >> ts >> c >> qw >> c >> qx >> c >> qy >> c >> qz >> c >> px >> c >> py >> c >> pz;
            } else {
                if (this->posFirst)
                    stream >> ts >> c >> px >> c >> py >> c >> pz >> c >> qx >> c >> qy >> c >> qz >> c >> qw;
                else
                    stream >> ts >> c >> qx >> c >> qy >> c >> qz >> c >> qw >> c >> px >> c >> py >> c >> pz;
            }
        }

        PosePtr gtData = make_shared<Pose>(ts/mTsFactor, px, py, pz, qw, qx, qy, qz);
        return boost::any(gtData);
    }

    unsigned long PoseDS::getNextChunk(size_t offset, unsigned long chunkSize, vector<PosePtr> &outData) {

        if (offset + chunkSize >= mvGtData.size())
            return 0;
        outData.resize(chunkSize);
        unsigned long dtCount = 0;
        for (; dtCount < chunkSize; dtCount++) {
            outData[dtCount] = mvGtData[offset+dtCount];
        }
        return dtCount;
    }

    unsigned long PoseDS::getNextChunk(double tsEnd, vector<PosePtr> &vpPoseGT, const double tsFactor) {

        unsigned long poseCnt = 0;
        if (mSIdx < 0 || mSIdx >= mvGtData.size())
            return poseCnt;

        const double invTsFactor = 1.0 / tsFactor;

        while (mSIdx < mvGtData.size() && mvGtData[mSIdx]->ts * invTsFactor <= tsEnd) {

            vpPoseGT.push_back(mvGtData[mSIdx]);
            this->incIdx();
            poseCnt++;
        }
        return poseCnt;
    }

    void PoseDS::reset() {
        mSIdx = 0;
    }

} // NAV24
