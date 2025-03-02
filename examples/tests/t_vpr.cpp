//
// Created by masoud on 3/2/25.
//

#include <iostream>
#include <fstream>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>

#include "OP_VPR_DBoW2.hpp"
#include "OP_FtDtOrbSlam.hpp"

using namespace std;
using namespace NAV24;

void getImageFrames(const string& pathImages, vector<FramePtr>& vpFrames) {

    auto pFtDtORB = make_shared<OP::FtDtOrbSlam>(500, 1.2, 8, 20, 7);

    ifstream ifs{pathImages + ".csv"};
    string line;

    while (getline(ifs, line, '\n')) {

        if (line[0] == '#') {
            continue;
        }

        unsigned long ts;
        string imgFile;
        char sep;

        istringstream iss{line};

        iss >> ts >> sep >> imgFile;
        imgFile.back() = '\0';

        imgFile = pathImages + "/" + imgFile;
        cv::Mat img = cv::imread(imgFile, cv::IMREAD_GRAYSCALE);

        auto pImage = make_shared<ImageTs>(img, ts, imgFile);
        FramePtr pFrame = make_shared<FrameImgMono>(ts, nullptr, vector<OB::ObsPtr>(), pImage);
        pFtDtORB->detect(pFrame);

        pImage->mImage = cv::Mat();
        vpFrames.push_back(pFrame);
    }
}


int main(int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    if (argc < 3) {
        cerr << "Usage: " << argv[0] << " path_EuRoC_DS path_ORB_Vocab\n";
        return 1;
    }

    string pathEuroc = argv[1];
    string pathOrbVoc = argv[2];

    string pathMH1 = pathEuroc + "/MH_01/mav0/cam0/data";
    string pathMH4 = pathEuroc + "/MH_04/mav0/cam0/data";

    // retrieve frames
    vector<FramePtr> vpFrames1, vpFrames2;
    getImageFrames(pathMH1, vpFrames1);
    getImageFrames(pathMH4, vpFrames2);

    // create a vocab from the first set of frames
    // create a db from the same set
    // retrieve similar frames from the second set

    auto pVprDbow2 = make_shared<OP::VPR_DBoW2>(pathOrbVoc, "", "");
    
    return 0;
}