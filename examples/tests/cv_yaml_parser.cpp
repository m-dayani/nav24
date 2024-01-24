//
// Created by root on 12/25/23.
//

#include <iostream>
#include <memory>
#include <thread>

#include <opencv2/core.hpp>

#include "YamlParserCV.hpp"
#include "Parameter.hpp"

using namespace std;
using namespace cv;
using namespace NAV24;


int main(int argc, char** argv) {

    string fileName = "../../config/AUN_ARM.yaml";
    string writeFile = "../../config/AUN_ARM1.yaml";

    FileStorage fs, fs_write;
    fs.open(fileName, FileStorage::READ);
    fs_write.open(writeFile, FileStorage::WRITE);
    if (!fs.isOpened())
    {
        cerr << "Failed to open " << fileName << endl;
        return 1;
    }

    cv::FileNode rootNode = fs.root();
    ParamPtr pParam = make_shared<ParamCV>(rootNode);

    ParamPtr pCamIntrinsics = pParam->read("Sensors/Camera/calib/intrinsics");
    ParamPtr pImagePath = pParam->read("DS/0/paths/imageBase");
    auto pImgPathSeq = static_pointer_cast<ParamSeq<string>>(pImagePath);
    pImgPathSeq->push_back("a-folder");

    pImagePath->write("DS/0/paths/imageBase", rootNode);

    ParamPtr ppParam = make_shared<ParamCV>(rootNode);
    ParamPtr ppImagePath = ppParam->read("DS/0/paths/imageBase");

    cout << pCamIntrinsics->printStr() << endl;
    // this shows that a READ fileStorage cannot be overwritten
    if (ppImagePath)
        cout << ppImagePath->printStr() << endl;

    YamlParserCV::saveParams(static_pointer_cast<ParamCV>(ppParam)->getNode(), fs_write);

    fs.release();
    fs_write.release();

    return 0;
}