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
            FILE_CSV,
            DIR
        };
        enum LoadState {
            BAD_PATH,
            BAD_DATA,
            READY,
            GOOD
        };

        TabularTextDS();
        virtual ~TabularTextDS() = default;

        static bool checkDirectory(const std::string &strPath);
        static bool checkFile(const std::string &strPath);
        static bool checkFileAndExtension(const std::string &strPath, const std::string &ext);
        static bool checkExtension(const boost::filesystem::path &p, const std::string &ext) {
            return p.has_extension() && p.extension().string() == ext;
        }
        static bool isComment(const std::string &txt);

        unsigned long getCurrByteIdx() const { return this->mnCurrByteIdx; }

        virtual void reset() = 0;

    protected:
        bool checkTxtStream();

        void openFile();
        void openNextDirFile();
        void loadTxtFile();

        template<typename T>
        unsigned long getTxtData(unsigned long chunkSize, std::vector<T> &outData) {

            if (!this->mTxtDataFile.is_open()) {

                LOG(ERROR) << "Text data file is not open\n";
                return 0;
            }

            std::string line;

            unsigned long dtCount = 0;

            //If data manager constantly supply the same outData,
            // all data will be stacked together.
            if (chunkSize > 0)
                outData.reserve(chunkSize);

            while (this->checkTxtStream())
            {
                if (chunkSize > 0 && dtCount >= chunkSize)
                    break;

                getline(this->mTxtDataFile, line);
                this->mnCurrByteIdx += line.length();

                if (isComment(line))
                    continue;

                boost::any res = parseLine(line);

                outData.push_back(boost::any_cast<T>(res));

                dtCount++;
            }
            return dtCount;
        }

        virtual boost::any parseLine(const std::string &dataStr) = 0;


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

        double mTsFactor;
    private:
    };

} // NAV24

#endif //NAV24_TABULARTEXTDS_H
