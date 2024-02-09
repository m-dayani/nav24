//
// Created by root on 5/7/21.
//

#ifndef NAV24_TABULARTEXTDS_H
#define NAV24_TABULARTEXTDS_H

#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/any.hpp>

#include <glog/logging.h>


namespace NAV24 {

    class TabularTextDS {
    public:
        enum PathState {
            INVALID,
            FILE,
            DIR
        };
        enum LoadState {
            BAD_PATH,
            BAD_DATA,
            READY,
            GOOD
        };
        enum OpenState {
            READ,
            WRITE
        };

        TabularTextDS();
        TabularTextDS(const std::string& baseDir, const std::string& fileName,
                      std::string  ext=".txt", std::string  delim=" ",
                      const OpenState& openState=OpenState::READ, bool sortFiles_=true);
        //virtual ~TabularTextDS() = default;

        void open();
        void close();
        void reset();

        std::string printStr(const std::string& prefix);

        // returns the next file path
        std::string getNextFile();
        // breaks the next line by delim and returns text data
        ulong getNextData(std::vector<std::string>& vTxtData);
        // get next line
        std::string getNextLine();
        //ulong readLines(const ulong& nLines, std::vector<std::string>& vLines);

        static bool checkDirectory(const std::string &strPath);
        static bool checkFile(const std::string &strPath);
        static bool checkFileAndExtension(const std::string &strPath, const std::string &ext);
        static bool checkExtension(const boost::filesystem::path &p, const std::string &ext);
        static bool isComment(const std::string &txt);

        unsigned long getCurrByteIdx() const { return this->mnCurrByteIdx; }

        //virtual void reset() = 0;

    protected:
        bool checkTxtStream();

        void openFile();
        void openNextDirFile();
        void loadTxtFile();

        void resolveFilePaths(const std::string& baseDir, const std::string& fileName);


        std::string mDataPath;

        std::ifstream mTxtDataFile;

        std::vector<std::string> mvFileNames;
        unsigned int mnFiles;
        unsigned int mnCurrFileIdx;

        //Number of bytes read from the file
        //This can be used to seek text file
        unsigned long mnCurrByteIdx;

        PathState mPathState;
        LoadState mLoadState;
        OpenState mOpenState;

        std::string mExt;
        std::string mDelim;
        bool mSortFiles;
    private:
    };

} // NAV24

#endif //NAV24_TABULARTEXTDS_H
