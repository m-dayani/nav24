
#include "TabularTextDS.hpp"

#include <utility>

#include "Parameter.hpp"

using namespace std;
using namespace boost::filesystem;


namespace NAV24 {

    TabularTextDS::TabularTextDS() :
            mnFiles(0), mnCurrFileIdx(0), mnCurrByteIdx(0), mPathState(PathState::INVALID),
            mLoadState(LoadState::BAD_PATH), mOpenState(OpenState::READ), mExt(), mDelim(" "), mSortFiles(true)
    {}

    TabularTextDS::TabularTextDS(const std::string &baseDir, const std::string &fileName, std::string ext,
                                 std::string  delim, const NAV24::TabularTextDS::OpenState &openState, bool sortFiles) :
            mnFiles(0), mnCurrFileIdx(0), mnCurrByteIdx(0), mPathState(PathState::INVALID),
            mLoadState(LoadState::BAD_PATH), mOpenState(openState), mExt(std::move(ext)),
            mDelim(std::move(delim)), mSortFiles(sortFiles) {

        this->resolveFilePaths(baseDir, fileName);
    }

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
        if (mnCurrFileIdx < mvFileNames.size()) {

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

    void TabularTextDS::loadTxtFile() {

        if (mPathState == PathState::FILE && mLoadState == LoadState::READY) {
            this->openFile();
        }
        else if (mPathState == PathState::DIR && mLoadState == LoadState::READY) {
            this->openNextDirFile();
        }
    }

    bool TabularTextDS::checkTxtStream() {

        if (this->mPathState == PathState::FILE) {
            return this->mTxtDataFile.good();
        }
        else if (this->mPathState == PathState::DIR){
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

    bool TabularTextDS::checkExtension(const boost::filesystem::path &p, const std::string &ext) {
        return p.has_extension() && p.extension().string() == ext;
    }

    void TabularTextDS::resolveFilePaths(const string &baseDir, const string &fileName) {

        // We can have these options for baseDir and fileName:
        // 1. baseDir is always non-empty and exists
        // 2. if fileName is not empty, we read/write from/to that file
        // 3. if fileName is empty, we get all files with specific extension from that dir
        // 4. sometimes we just need the name of files (for images)
        //    and other times the content (like event files, imu.txt, or images.txt

        if (!checkDirectory(baseDir)) {
            DLOG(WARNING) << "TabularTextDS::resolveFilePaths, base dir doesn't exist: " << baseDir << "\n";
            return;
        }

        boost::filesystem::path basePath(baseDir);
        boost::filesystem::path filePath(fileName);

        if (fileName.empty()) {
            mPathState = DIR;
            mDataPath = baseDir;

            for (auto&& x : boost::filesystem::directory_iterator(basePath)) {

                const boost::filesystem::path& currFile = x.path();
                if (checkExtension(currFile, mExt)) {

                    mvFileNames.push_back(currFile.string());
                    mnFiles++;
                }
            }
            if (mnFiles > 0) {
                mLoadState = LoadState::READY;
                if (mSortFiles) {
                    std::sort(mvFileNames.begin(), mvFileNames.end());
                }
            }
            else {
                mLoadState = LoadState::BAD_DATA;
            }
        }
        else {
            mPathState = FILE;
            boost::filesystem::path fullPath = basePath / filePath;
            mDataPath = fullPath.string();

            if (checkExtension(fullPath, mExt)) {
                mLoadState = LoadState::READY;
            }
            else {
                mLoadState = LoadState::BAD_PATH;
            }
        }
    }

    std::string TabularTextDS::getNextFile() {

        if (mnCurrFileIdx < mnFiles) {
            return mvFileNames[mnCurrFileIdx++];
        }
        return {};
    }

    ulong TabularTextDS::getNextData(vector <std::string> &vTxtData) {

        if (!this->mTxtDataFile.is_open()) {

            DLOG(ERROR) << "TabularTextDS::getNextData, Text data file is not open\n";
            return 0;
        }

        std::string line;
        vTxtData.reserve(10);

        getline(this->mTxtDataFile, line);
        this->mnCurrByteIdx += line.length();

        if (line.empty()) {
            DLOG(INFO) << "TabularTextDS::getNextData, line is empty\n";
            return 0;
        }

        if (isComment(line)) {
            vTxtData.push_back(line);
        }
        else {
            // todo: merge text split/merge with parameters in a util class
            Parameter::splitKey(vTxtData, line, mDelim);
        }

        return vTxtData.size();
    }

    std::string TabularTextDS::getNextLine() {

        if (!this->mTxtDataFile.is_open()) {
            LOG(ERROR) << "Text data file is not open\n";
            return {};
        }

        std::string line;
        getline(this->mTxtDataFile, line);
        this->mnCurrByteIdx += line.length();

        return line;
    }

    void TabularTextDS::open() {
        this->loadTxtFile();
    }

    void TabularTextDS::close() {
        if (mTxtDataFile.is_open()) {
            mTxtDataFile.close();
        }
    }

    void TabularTextDS::reset() {

        this->close();
        mnCurrFileIdx = 0;
        mnCurrByteIdx = 0;
        mLoadState = LoadState::READY;
        this->open();
    }

    std::string TabularTextDS::printStr(const string &prefix) {

        ostringstream oss;

        oss << prefix << "Data Path: " << mDataPath << "\n";
        oss << prefix << "Num Files: " << mnFiles << "\n";
        oss << prefix << "File Index: " << mnCurrFileIdx << "\n";
        oss << prefix << "Path State: " << mPathState << "\n";
        oss << prefix << "Load State: " << mLoadState << "\n";
        oss << prefix << "Open State: " << mOpenState << "\n";

        return oss.str();
    }

} // NAV24
