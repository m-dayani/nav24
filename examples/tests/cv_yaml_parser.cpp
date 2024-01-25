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

    vector<ParamPtr> vAllParams{};

    ParamPtr pParam = YamlParserCV::loadParams(fileName, vAllParams);

    ParamPtr pCamIntrinsics = pParam->read("Sensors/Camera/calib/intrinsics");
    ParamPtr pImagePath = pParam->read("DS/0/paths/imageBase");
    auto pImgPathSeq = static_pointer_cast<ParamSeq<string>>(pImagePath);
    if (pImgPathSeq)
        pImgPathSeq->push_back("a-folder");

    cout << pParam->printStr("\t") << endl;
    cout << pCamIntrinsics->printStr() << endl;
    if (pImgPathSeq)
        cout << pImgPathSeq->printStr() << endl;

    // this strips the comments and reverses the order of all key:value pairs!
    YamlParserCV::saveParams(writeFile, pParam);

    return 0;
}