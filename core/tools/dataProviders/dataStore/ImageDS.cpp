//
// Created by root on 5/16/21.
//

#include "ImageDS.hpp"

using namespace std;
using namespace boost::filesystem;

namespace NAV24 {

    ImageDS::ImageDS(const string &imTxtFile, const string &imBase, const double tsFactor)
    {
        boost::filesystem::path imFiles(imTxtFile);
        boost::filesystem::path imBasePath(imBase);
        mTsFactor = tsFactor;

        try {
            // Check image file names
            if (exists(imFiles) && is_regular_file(imFiles) && is_directory(imBasePath))
            {
                this->mDataPath = imTxtFile;
                this->mImagesBasePath = imBase;
                this->mPathState = PathState::FILE;

                if (checkExtension(imFiles, ".txt")) {
                    mLoadState = LoadState::READY;
                }
                else if (checkExtension(imFiles, ".csv")) {
                    mPathState = PathState::FILE_CSV;
                    mLoadState = LoadState::READY;
                }
                else {
                    mLoadState = LoadState::BAD_PATH;
                }
            }
            else {
                cerr << imFiles << " does not exist\n";
            }
            if (mLoadState == LoadState::READY) {
                this->openFile();
                this->getTxtData(0, mvImageData);
                // Check if number of images in directory matches
                // the number of file names in image file.
                if (!mvImageData.empty()) {
                    boost::filesystem::path firstIm = boost::filesystem::path(imBase + '/' + mvImageData[0].second);
                    bool res = is_regular_file(firstIm) && checkExtension(firstIm, ".png");
                    if (res) {
                        mnFiles = mvImageData.size();
                        mLoadState = LoadState::GOOD;
                    }
                    else
                        mLoadState = LoadState::BAD_DATA;
                }
                else {
                    cerr << "Image file names is empty\n";
                }
            }
        }
        catch (const boost::filesystem::filesystem_error& ex)
        {
            cerr << ex.what() << '\n';
        }

    }

    ImageDS::~ImageDS() {
        if (mTxtDataFile && mTxtDataFile.is_open())
        {
            mTxtDataFile.close();
        }
    }

    boost::any ImageDS::parseLine(const string &evStr) {

        std::istringstream stream(evStr);

        double ts;
        char c;
        string s;

        if (mPathState == PathState::FILE)
            stream >> ts >> s;
        else if (mPathState == PathState::FILE_CSV)
            stream >> ts >> c >> s;

        return boost::any(make_pair(ts/mTsFactor, s));
    }

    void ImageDS::getImage(unsigned int idx, cv::Mat &image, double &ts) {

        image = cv::Mat();
        ts = 0.0;
        if (idx >= mvImageData.size())
            return;
        image = cv::imread(mImagesBasePath+'/'+mvImageData[idx].second, cv::IMREAD_UNCHANGED);
        ts = mvImageData[idx].first;
    }

    void ImageDS::getImage(unsigned int idx, cv::Mat &image, double &ts, string& imPath) {

        image = cv::Mat();
        ts = 0.0;
        if (idx >= mvImageData.size())
            return;
        imPath = mImagesBasePath+'/'+mvImageData[idx].second;
        image = cv::imread(imPath, cv::IMREAD_UNCHANGED);
        ts = mvImageData[idx].first;
    }

    string ImageDS::getFileName(size_t idx, bool fullPath) {
        if (idx < 0 || idx >= mvImageData.size())
            return string();
        if (fullPath)
            return mImagesBasePath + '/' + mvImageData[idx].second;
        else
            return mvImageData[idx].second;
    }

    double ImageDS::getTimeStamp(size_t idx) {
        if (idx >= mvImageData.size())
            return 0.0;
        return this->mvImageData[idx].first;
    }

    // Nothing to do!
    void ImageDS::reset() {}

} // NAV24
