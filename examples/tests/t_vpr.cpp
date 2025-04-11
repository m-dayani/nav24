//
// Created by masoud on 3/2/25.
//

#include <iostream>
#include <fstream>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <DBoW2/DBoW2.h>

#include "OP_VPR_DBoW2.hpp"
#include "OP_FtDtOrbSlam.hpp"

using namespace std;
using namespace NAV24;

void getImageFrames(const string& pathImages, const int skip, vector<FramePtr>& vpFrames,
                    vector<unsigned long>& vTsMap) {

    auto pFtDtORB = make_shared<OP::FtDtOrbSlam>(500, 1.2, 8, 20, 7);

    ifstream ifs{pathImages + ".csv"};
    string line;

    int cnt = -1;
    while (getline(ifs, line, '\n')) {

        if (line[0] == '#') {
            continue;
        }

        cnt++;
        if (cnt % skip != 0) {
            continue;
        }

        unsigned long ts;
        string imgFile;
        char sep;

        istringstream iss{line};

        iss >> ts >> sep >> imgFile;
//        imgFile.back() = '\0';

        imgFile = pathImages + "/" + imgFile;
        cv::Mat img = cv::imread(imgFile, cv::IMREAD_GRAYSCALE);

        auto pImage = make_shared<ImageTs>(img, ts, imgFile);
        FramePtr pFrame = make_shared<FrameImgMono>(ts, nullptr, vector<OB::ObsPtr>(), pImage);
        pFtDtORB->detect(pFrame);

        pImage->mImage = cv::Mat();
        vpFrames.push_back(pFrame);
        vTsMap.push_back(ts);
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
    const int skip = 5;
    vector<FramePtr> vpFrames1, vpFrames2;
    vector<unsigned long> tsMap1, tsMap2;
    getImageFrames(pathMH1, skip, vpFrames1, tsMap1);
    getImageFrames(pathMH4, 1, vpFrames2, tsMap2);

#ifdef LIB_DBOW2_FOUND
    // VPR using bag of binary words
    auto pVprDbow2 = make_shared<OP::VPR_DBoW2>(pathOrbVoc, "", "");
    pVprDbow2->setTsMap(tsMap1);
    // todo: implement this way
//    auto pVprDbow2 = make_shared<OP::VPR_DBoW2>(mpSystem);
//    mpSystem->registerChannel(pVprDbow2);
    // init mpSystem operators

    // create a vocab from the first set of frames
    shared_ptr<OrbVocabulary> pOrbVoc;
    pVprDbow2->createVocab(vpFrames1, pOrbVoc);
    pVprDbow2->reloadDbWithVocab(pOrbVoc);

    // create a db from the same set
    int cnt = 0;
    for (const auto& pFrame : vpFrames1) {
        pVprDbow2->add(pFrame);
        cnt++;
    }

    // retrieve similar frames from the second set
    vector<unsigned long> vResult;
    auto pKfQuery = vpFrames1[(size_t) (vpFrames1.size() / 2)];
    pVprDbow2->getBestMatches(pKfQuery, 3, vResult);

    string queryTs = to_string((unsigned long) pKfQuery->getTs());
    string pathImgQuery = pathMH1 + "/" + queryTs + ".png";
    cv::Mat imgQuery = cv::imread(pathImgQuery, cv::IMREAD_UNCHANGED);
    cv::cvtColor(imgQuery, imgQuery, cv::COLOR_GRAY2BGR);
    cv::putText(imgQuery, "Query", cv::Point(20, 30), cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0, 0, 255));
    cv::putText(imgQuery, queryTs, cv::Point(20, 70), cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0, 255, 0));
    int h = imgQuery.rows;
    int w = imgQuery.cols;

    cv::Mat imgShow = cv::Mat::zeros(h * 2, w * 2, CV_8UC3);
    imgQuery.copyTo(imgShow(cv::Rect(0, 0, w, h)));

    int i = 1;
    for (const auto& res : vResult) {
        cout << res << "\n";
        string pathImgFound = pathMH1 + "/" + to_string(res) + ".png";
        cv::Mat imgFound = cv::imread(pathImgFound, cv::IMREAD_UNCHANGED);
        cv::cvtColor(imgFound, imgFound, cv::COLOR_GRAY2BGR);
        cv::putText(imgFound, to_string(res), cv::Point(20, 70), cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0, 255, 0));

        if (i == 1) {
            imgFound.copyTo(imgShow(cv::Rect(w, 0, w, h)));
        }
        else if (i == 2) {
            imgFound.copyTo(imgShow(cv::Rect(0, h, w, h)));
        }
        else if (i == 3) {
            imgFound.copyTo(imgShow(cv::Rect(w, h, w, h)));
        }
        i++;
    }

    cv::resize(imgShow, imgShow, cv::Size(w, h));
    cv::imshow("Query Result", imgShow);
    cv::waitKey();

    cv::destroyAllWindows();

#endif

    return 0;
}