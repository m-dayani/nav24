
#include "TabularTextDS.hpp"

using namespace std;
using namespace boost::filesystem;


namespace NAV24 {

    TabularTextDS::TabularTextDS() :
            mnFiles(0), mnCurrFileIdx(0), mnCurrByteIdx(0), mPathState(PathState::INVALID),
            mLoadState(LoadState::BAD_PATH), mTsFactor(1.0)
    {}

    void TabularTextDS::openFile()
    {
        this->mTxtDataFile.open(this->mDataPath);
        if (mTxtDataFile.is_open()) {
            mLoadState = LoadState::GOOD;
        }
        else {
            mLoadState = LoadState::BAD_DATA;
        }
    }

    void TabularTextDS::openNextDirFile()
    {
        if (this->mTxtDataFile.is_open()) {
            this->mTxtDataFile.close();
            this->mnCurrByteIdx = 0;
        }
        if (mnCurrFileIdx < mvFileNames.size())
        {
            this->mTxtDataFile.open(mvFileNames[mnCurrFileIdx]);
            if (mTxtDataFile.is_open()) {
                mnCurrFileIdx++;
                mLoadState = LoadState::GOOD;
            }
            else {
                mLoadState = LoadState::BAD_DATA;
            }
        }
    }

    void TabularTextDS::loadTxtFile()
    {
        if (mPathState == PathState::FILE && mLoadState == LoadState::READY)
        {
            this->openFile();
        }
        else if (mPathState == PathState::DIR && mLoadState == LoadState::READY)
        {
            this->openNextDirFile();
        }
    }

    bool TabularTextDS::checkTxtStream()
    {
        if (this->mPathState == PathState::FILE || this->mPathState == PathState::FILE_CSV)
        {
            return this->mTxtDataFile.good();
        }
        else if (this->mPathState == PathState::DIR)
        {
            if (this->mTxtDataFile) {
                return true;
            }
            else {
                this->openNextDirFile();
                return this->mTxtDataFile.good();
            }
        }
        return false;
    }

    bool TabularTextDS::isComment(const string &txt) {
        size_t idx = txt.find_first_not_of(" \t\n");
        return txt[idx] == '#';
    }

//    template<typename T>
//    unsigned long TabularTextDS::getTxtData(const unsigned long chunkSize, vector<T> &outData)

    bool TabularTextDS::checkFileAndExtension(const string &strPath, const string &ext) {
        path p(strPath);
        return exists(p) && is_regular_file(p) && p.has_extension() && p.extension().string() == ext;
    }

    bool TabularTextDS::checkDirectory(const string &strPath) {
        path p(strPath);
        return exists(p) && is_directory(p);
    }

    bool TabularTextDS::checkFile(const string &strPath) {
        path p(strPath);
        return exists(p) && is_regular_file(p);
    }

} // NAV24
