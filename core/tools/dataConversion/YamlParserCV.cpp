//
// Created by root on 5/8/21.
//

#include "YamlParserCV.hpp"

#include <iostream>
#include <stdexcept>
#include <glog/logging.h>
#include <iomanip>

using namespace std;

namespace NAV24 {

    /*bool is_number(const std::string& s) {
        std::string::const_iterator it = s.begin();
        while (it != s.end() && std::isdigit(*it)) ++it;
        return !s.empty() && it == s.end();
    }*/

    ParamPtr YamlParserCV::loadParams(const string &fileName, std::vector<ParamPtr>& vAllParams) {

        ParamPtr pRootParam{make_shared<Parameter>("Root", nullptr)};
        vAllParams.push_back(pRootParam);

        cv::FileStorage fs;
        fs.open(fileName, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            DLOG(ERROR) << "YamlParserCV::setup, Config file not found: " << fileName << "\n";
            return pRootParam;
        }

        YamlParserCV::readParam(fs.root(), pRootParam, vAllParams);

        fs.release();

        return pRootParam;
    }

    void YamlParserCV::saveParams(const string& fileName, const ParamPtr& rootParam) {

        cv::FileStorage fs;
        fs.open(fileName, cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            DLOG(ERROR) << "YamlParserCV::saveParams, Config file not found: " << fileName << "\n";
            return;
        }

        writeParam(fs, rootParam);

        fs.release();
    }

    void YamlParserCV::readParam(const cv::FileNode &node, ParamPtr& pParentParam, vector<ParamPtr>& vAllParams) {

        Parameter::NodeType nodeType = getNodeType(node);
        //bool initialized = false;

        switch (nodeType) {
            case Parameter::NodeType::MAP_NODE: {
                pParentParam->setType(nodeType);
                vector<string> keys = node.keys();
                for (const auto& key : keys) {
                    cv::FileNode newNode = node[key];
                    Parameter::NodeType nodeType1 = getNodeType(newNode);
                    if (nodeType1 == Parameter::NodeType::MAP_NODE || nodeType1 == Parameter::NodeType::SEQ_NODE) {
                        ParamPtr pParam{make_shared<Parameter>(key, pParentParam)};
                        pParentParam->insertChild(key, pParam);
                        vAllParams.push_back(pParam);
                        readParam(newNode, pParam, vAllParams);
                    }
                    else {
                        // don't create a node link when we have a key:value pair
                        readParam(newNode, pParentParam, vAllParams);
                    }
                }
                break;
            }
            case Parameter::NodeType::SEQ_NODE: {
                pParentParam->setType(nodeType);
                int i = 0;
                for (auto iter = node.begin(); iter != node.end(); iter++, i++) {
                    cv::FileNode newNode = *iter;
                    string arr_key = to_string(i);
                    ParamPtr pParam{make_shared<Parameter>(arr_key, pParentParam)};
                    pParentParam->insertChild(arr_key, pParam);
                    vAllParams.push_back(pParam);
                    readParam(newNode, pParam, vAllParams);
                    //pParam->setType(nodeType);
                }
                break;
            }
            case Parameter::NodeType::SEQ_REAL:
            case Parameter::NodeType::SEQ_INT:
            case Parameter::NodeType::SEQ_STR:
            case Parameter::NodeType::STRING:
            case Parameter::NodeType::INT:
            case Parameter::NodeType::REAL:
            case Parameter::NodeType::CV_MAT: {
                string key = "un";
                if (node.isNamed())
                    key = node.name();
                ParamPtr pParam;

                if (nodeType == Parameter::NodeType::SEQ_REAL) {
                    vector<double> val{};
                    for (auto && iter : node) {
                        val.push_back((double) iter);
                    }
                    pParam = make_shared<ParamSeq<double>>(key, pParentParam, val);
                }
                else if (nodeType == Parameter::NodeType::SEQ_INT) {
                    vector<int> val{};
                    for (auto && iter : node) {
                        val.push_back((int) iter);
                    }
                    pParam = make_shared<ParamSeq<int>>(key, pParentParam, val);

                }
                else if (nodeType == Parameter::NodeType::SEQ_STR) {
                    vector<string> val{};
                    for (auto && iter : node) {
                        val.push_back((string) iter);
                    }
                    pParam = make_shared<ParamSeq<string>>(key, pParentParam, val);
                }
                else if (nodeType == Parameter::NodeType::CV_MAT) {
                    cv::Mat val;
                    node >> val;
                    pParam = make_shared<ParamType<cv::Mat>>(key, pParentParam, val.clone());
                }
                else if (nodeType == Parameter::NodeType::INT) {
                    int val = (int) node;
                    pParam = make_shared<ParamType<int>>(key, pParentParam, val);
                }
                else if (nodeType == Parameter::NodeType::REAL) {
                    auto val = (double) node;
                    pParam = make_shared<ParamType<double>>(key, pParentParam, val);
                }
                else {
                    string val = (string) node;
                    pParam = make_shared<ParamType<string>>(key, pParentParam, val);
                }

                if (pParam) {
                    pParam->setType(nodeType);
                    pParentParam->insertChild(key, pParam);
                    vAllParams.push_back(pParam);
                }
                break;
            }
            default:
                DLOG(WARNING) << "YamlParserCV: couldn't identify node type: " << nodeType << "\n";
                break;
        }
    }

    void YamlParserCV::writeParam(cv::FileStorage &fs, const ParamPtr &param) {

        // due to the recursive nature of this method (and readParam),
        // this prints a file in reverse order

        if (!param) {
            DLOG(WARNING) << "YamlParserCV::writeParam, null parameter input detected\n";
            return;
        }

        int nodeType = param->getType();
        auto allChildren = param->getAllChildren();

        if (nodeType == Parameter::NodeType::MAP_NODE) {
            vector<string> vChildKeys = param->getAllChildKeys();
            for (const auto& key : vChildKeys) {
                auto pParam = allChildren[key].lock();
                if (pParam) {
                    int nodeType1 = pParam->getType();
                    if (nodeType1 == Parameter::NodeType::MAP_NODE) {
                        fs << sanitizeKey(key) << "{";
                        writeParam(fs, pParam);
                        fs << "}";
                    }
                    else {
                        writeParam(fs, pParam);
                    }
                }
            }
        }
        else if (nodeType == Parameter::NodeType::SEQ_NODE) {
            fs << sanitizeKey(param->getName()) << "[";
            for (const auto& child : allChildren) {
                fs << "{";
                writeParam(fs, child.second.lock());
                fs << "}";
            }
            fs << "]";
        }
        else if (nodeType == Parameter::NodeType::SEQ_STR ||
                 nodeType == Parameter::NodeType::SEQ_INT ||
                 nodeType == Parameter::NodeType::SEQ_REAL) {

            fs << sanitizeKey(param->getName()) << "[";
            if (dynamic_pointer_cast<ParamSeq<int>>(param)) {
                shared_ptr<ParamSeq<int>> paramVecInt = dynamic_pointer_cast<ParamSeq<int>>(param);
                vector<int> data = paramVecInt->getValue();
                for (auto d : data) {
                    fs << d;
                }
            }
            if (dynamic_pointer_cast<ParamSeq<double>>(param)) {
                shared_ptr<ParamSeq<double>> paramVecReal = dynamic_pointer_cast<ParamSeq<double>>(param);
                vector<double> data = paramVecReal->getValue();
                for (auto d : data) {
                    fs << d;
                }
            }
            if (dynamic_pointer_cast<ParamSeq<string>>(param)) {
                shared_ptr<ParamSeq<string>> paramVecStr = dynamic_pointer_cast<ParamSeq<string>>(param);
                vector<string> data = paramVecStr->getValue();
                for (const auto& d : data) {
                    fs << d;
                }
            }
            fs << "]";
        }
        else if (nodeType == Parameter::NodeType::STRING ||
                 nodeType == Parameter::NodeType::INT ||
                 nodeType == Parameter::NodeType::REAL ||
                 nodeType == Parameter::NodeType::CV_MAT) {
            fs << sanitizeKey(param->getName());
            if (dynamic_pointer_cast<ParamType<string>>(param)) {
                shared_ptr<ParamType<string>> paramString = dynamic_pointer_cast<ParamType<string>>(param);
                //try {
                fs << paramString->getValue();
                //}
                //catch (const exception& e) {
                //    cerr << e.what() << endl;
                //}
            }
            if (dynamic_pointer_cast<ParamType<int>>(param)) {
                shared_ptr<ParamType<int>> paramInt = dynamic_pointer_cast<ParamType<int>>(param);
                fs << paramInt->getValue();
            }
            if (dynamic_pointer_cast<ParamType<double>>(param)) {
                shared_ptr<ParamType<double>> paramReal = dynamic_pointer_cast<ParamType<double>>(param);
                //ostringstream oss;
                //oss << fixed << std::setprecision(6) << paramReal->getValue();
                fs << paramReal->getValue();
            }
            if (dynamic_pointer_cast<ParamType<cv::Mat>>(param)) {
                shared_ptr<ParamType<cv::Mat>> paramMat = dynamic_pointer_cast<ParamType<cv::Mat>>(param);
//                cv::Mat mat = paramMat->getValue();
//                mat.convertTo(mat, CV_32FC1);
                fs << paramMat->getValue();
            }
        }
    }

    Parameter::NodeType YamlParserCV::getNodeType(const cv::FileNode &node) {

        if (node.isMap()) {
            // check for cv::Mat
            if (isNodeCvMat(node)) {
                return Parameter::NodeType::CV_MAT;
            }
            return Parameter::NodeType::MAP_NODE;
        }
        else if (node.isSeq()) {
            cv::FileNode newNode = *(node.begin());
            if (newNode.isMap()) {
                return Parameter::NodeType::SEQ_NODE;
            }
            else if (newNode.isInt()) {
                return Parameter::NodeType::SEQ_INT;
            }
            else if (newNode.isString()) {
                return Parameter::NodeType::SEQ_STR;
            }
            else if (newNode.isReal()) {
                return Parameter::NodeType::SEQ_REAL;
            }
        }
        else if (node.isString()) {
            return Parameter::NodeType::STRING;
        }
        else if (node.isInt()) {
            return Parameter::NodeType::INT;
        }
        else if (node.isReal()) {
            return Parameter::NodeType::REAL;
        }
        return Parameter::NodeType::DEFAULT;
    }

    bool YamlParserCV::isNodeCvMat(const cv::FileNode &node) {

        bool bcols = false, brows = false;
        if (node.isMap()) {
            vector<string> keys = node.keys();
            for (const auto& key : keys) {
                if (key == "rows") brows = true;
                if (key == "cols") bcols = true;
            }
        }
        return bcols && brows;
    }

    string YamlParserCV::sanitizeKey(const string &key) {
        string newKey = key;
        if (key.find('.') != std::string::npos) {
            std::replace(newKey.begin(), newKey.end(), '.', '_');
        }
        return newKey;
    }

} // NAV24
