//
// Created by root on 12/25/23.
//

#include <iostream>
#include <memory>
#include <thread>

#include <opencv2/core.hpp>
#include <glog/logging.h>

#include "YamlParserCV.hpp"
//#include "Parameter.hpp"

using namespace std;
using namespace cv;
using namespace NAV24;


int main([[maybe_unused]] int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    // A simple config file
    string fileName = "../../config/BluePrint.yaml";
    // A much harder file
    string configComplex = "../../config/EvETHZ.yaml";
    // A non-existing file
    string noFile = "path/to/nowhere.yaml";
    // Write changes
    string writeFile = "../../config/AUN_ARM1.yaml";
    string writeFile1 = "../../config/DUMMY.yaml";

    vector<ParamPtr> vAllParams{}, vAllParams1{}, vAllParams2{};

    // 1. YamlParserCV::loadParams

    ParamPtr pParam = YamlParserCV::loadParams(fileName, vAllParams);
    ParamPtr pParamCplx = YamlParserCV::loadParams(configComplex, vAllParams1);
    ParamPtr pParamNull = YamlParserCV::loadParams(noFile, vAllParams2);

    // 2. Parameter operations

    // Normal operation
    // Read
    ParamPtr pCamIntrinsics = pParam->read("Input/Camera/calib/intrinsics");
    ParamPtr pImagePath = pParam->read("Process/DS/0/paths/imageBase");
    auto pImgPathSeq = dynamic_pointer_cast<ParamSeq<string>>(pImagePath);
    if (pImgPathSeq) {
        pImgPathSeq->push_back("a-folder");
    }
    else if (dynamic_pointer_cast<NAV24::ParamType<string>>(pImagePath)) {
        auto pImgPath = dynamic_pointer_cast<NAV24::ParamType<string>>(pImagePath);
        auto pImgPathParent = pImgPath->getParent();
        pImgPathSeq = make_shared<ParamSeq<string>>(pImgPath->getName(), pImgPathParent, vector<string>());
        // the setTypes are needed if you want to save parameters
        pImgPathSeq->setType(Parameter::NodeType::SEQ_STR);
        vector<string> vImgPaths{pImgPath->getValue(), "a-folder"};
        pImgPathSeq->setValue(vImgPaths);
        if (pImgPathParent)
            pImgPathParent->insertChild(pImgPath->getName(), pImgPathSeq);
    }
    auto pTbc = find_param<NAV24::ParamType<cv::Mat>>("Tbc", pParamCplx);
    if (pTbc) {
        cout << "Tbc: " << pTbc->getValue() << endl;
    }

    // Create/Insert/Remove
    auto pEvMaxNumFts = find_param<NAV24::ParamType<int>>("Event.fts.maxNumPts", pParamCplx);
    int evMaxNumFts = -1;
    if (pEvMaxNumFts)
        evMaxNumFts = pEvMaxNumFts->getValue();
    ParamPtr pEvent = make_shared<Parameter>("Event", pParamCplx, Parameter::NodeType::MAP_NODE);
    ParamPtr pEvFts = make_shared<Parameter>("fts", pEvent, Parameter::NodeType::MAP_NODE);
    pEvent->insertChild("fts", pEvFts);
    ParamPtr pMaxNumPts = make_shared<NAV24::ParamType<int>>("maxNumPts", pEvFts, evMaxNumFts);
    pMaxNumPts->setType(Parameter::NodeType::INT);
    pEvFts->insertChild("maxNumPts", pMaxNumPts);
    pParamCplx->insertChild("Event", pEvent);
    pParamCplx->removeChild("Event.fts.maxNumPts");
    shared_ptr<ParamSeq<string>> pEvSeqNames = make_shared<ParamSeq<string>>("DS", pEvent, vector<string>());
    pEvSeqNames->setType(Parameter::NodeType::SEQ_STR);
    auto pCurrEvSeqNames = find_param<ParamSeq<string>>("DS.Seq.names", pParamCplx);
    if (pCurrEvSeqNames) {
        pEvSeqNames->setValue(pCurrEvSeqNames->getValue());
    }
    pEvent->insertChild("DS", pEvSeqNames);

    // Print
    cout << pParam->printStr("\t") << endl;
    cout << pCamIntrinsics->printStr("") << endl;
    if (pImgPathSeq)
        cout << pImgPathSeq->printStr("") << endl;
    cout << pEvent->printStr("") << endl;

    // Other operations
    vector<string> mergeKeys{"root", "parent", "child"};
    cout << Parameter::mergeKey(mergeKeys) << endl;

    if (pMaxNumPts)
        pMaxNumPts->setName("MaxNumPts");

    // Bad operation
    ParamPtr pDummyParam = pParam->read("Key/to/nothing");
    if (pDummyParam) {
        cout << "Wrong key: " << pDummyParam->printStr("")  << endl;
    }
    ParamPtr pEmptyKey = pParam->read("");
    if (pEmptyKey) {
        cout << "Empty-key: " << pEmptyKey->printStr("") << endl;
    }
    auto pNoCvMat = find_param<NAV24::ParamType<cv::Mat>>("T_non-cv-mat", pParamCplx);
    if (pNoCvMat) {
        cout << "Non-existing Mat: " << pNoCvMat->getValue() << endl;
    }
    pParam->removeChild("wrong/key");
    pParamCplx->removeChild("");
    pParamCplx->insertChild("DummyKey", nullptr);
    pEvSeqNames->setName("");
    // this won't even compile!
    //pEvSeqNames->push_back(43);

    // 3. YamlParserCV::saveParams

    // this strips the comments!
    YamlParserCV::saveParams(writeFile, pParamNull);
    YamlParserCV::saveParams(writeFile, pParam);
    YamlParserCV::saveParams(writeFile1, pParamCplx);
    YamlParserCV::saveParams(noFile, pParam);

    return 0;
}