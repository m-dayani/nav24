//
// Created by root on 5/8/21.
//

#include "YamlParserCV.hpp"
#include <iostream>

using namespace std;

namespace NAV24 {

    bool is_number(const std::string& s) {
        std::string::const_iterator it = s.begin();
        while (it != s.end() && std::isdigit(*it)) ++it;
        return !s.empty() && it == s.end();
    }

    ParamPtr YamlParserCV::loadParams(const string &fileName, std::vector<ParamPtr>& vAllParams) {

        ParamPtr pRootParam{make_shared<Parameter>("Root", nullptr)};
        vAllParams.push_back(pRootParam);

        cv::FileStorage fs;
        fs.open(fileName, cv::FileStorage::READ);
        if (!fs.isOpened()) {
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
            return;
        }

        writeParam(fs, rootParam);

        fs.release();
    }

    void YamlParserCV::readParam(const cv::FileNode &node, ParamPtr& pParentParam, vector<ParamPtr>& vAllParams) {

        NodeType nodeType = getNodeType(node);
        //bool initialized = false;

        switch (nodeType) {
            case NodeType::MAP_NODE: {
                pParentParam->setType(nodeType);
                vector<string> keys = node.keys();
                for (auto key : keys) {
                    cv::FileNode newNode = node[key];
                    NodeType nodeType1 = getNodeType(newNode);
                    if (nodeType1 == NodeType::MAP_NODE || nodeType1 == NodeType::SEQ_NODE) {
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
            case NodeType::SEQ_NODE: {
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
            case NodeType::SEQ_REAL:
            case NodeType::SEQ_INT:
            case NodeType::SEQ_STR:
            case NodeType::STRING:
            case NodeType::INT:
            case NodeType::REAL:
            case NodeType::CV_MAT: {
                string key = "un";
                if (node.isNamed())
                    key = node.name();
                ParamPtr pParam = nullptr;

                if (nodeType == NodeType::SEQ_REAL) {
                    vector<double> val{};
                    for (auto iter = node.begin(); iter != node.end(); iter++) {
                        val.push_back((double) (*iter));
                    }
                    pParam = make_shared<ParamSeq<double>>(key, pParentParam, val);
                }
                else if (nodeType == NodeType::SEQ_INT) {
                    vector<int> val{};
                    for (auto iter = node.begin(); iter != node.end(); iter++) {
                        val.push_back((int) (*iter));
                    }
                    pParam = make_shared<ParamSeq<int>>(key, pParentParam, val);

                }
                else if (nodeType == NodeType::SEQ_STR) {
                    vector<string> val{};
                    for (auto iter = node.begin(); iter != node.end(); iter++) {
                        val.push_back((string) (*iter));
                    }
                    pParam = make_shared<ParamSeq<string>>(key, pParentParam, val);
                }
                else if (nodeType == NodeType::CV_MAT) {
                    cv::Mat val;
                    node >> val;
                    pParam = make_shared<ParamType<cv::Mat>>(key, pParentParam, val.clone());
                }
                else if (nodeType == NodeType::INT) {
                    int val = (int) node;
                    pParam = make_shared<ParamType<int>>(key, pParentParam, val);
                }
                else if (nodeType == NodeType::REAL) {
                    double val = (double) node;
                    pParam = make_shared<ParamType<double>>(key, pParentParam, val);
                }
                else if (nodeType == NodeType::STRING) {
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
                cerr << "YamlParserCV: couldn't identify node type\n";
                break;
        }
    }

    void YamlParserCV::writeParam(cv::FileStorage &fs, const ParamPtr &param) {

        // due to the recursive nature of this method (and readParam),
        // this prints a file in reverse order
        // todo: improve this

        int nodeType = param->getType();
        auto allChildren = param->getAllChildren();

        if (nodeType == NodeType::MAP_NODE) {
            for (auto child : allChildren) {
                string key = child.first;
                auto pParam = child.second.lock();
                int nodeType1 = pParam->getType();
                if (nodeType1 == NodeType::MAP_NODE) {
                    fs << key << "{";
                    writeParam(fs, pParam);
                    fs << "}";
                }
                else {
                    writeParam(fs, pParam);
                }
            }
        }
        else if (nodeType == NodeType::SEQ_NODE) {
            fs << param->getName() << "[";
            for (auto child : allChildren) {
                fs << "{";
                writeParam(fs, child.second.lock());
                fs << "}";
            }
            fs << "]";
        }
        else if (nodeType == NodeType::SEQ_STR ||
                 nodeType == NodeType::SEQ_INT ||
                 nodeType == NodeType::SEQ_REAL) {

            fs << param->getName() << "[";
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
                for (auto d : data) {
                    fs << d;
                }
            }
            fs << "]";
        }
        else if (nodeType == NodeType::STRING ||
                 nodeType == NodeType::INT ||
                 nodeType == NodeType::REAL ||
                 nodeType == NodeType::CV_MAT) {
            fs << param->getName();
            if (dynamic_pointer_cast<ParamType<string>>(param)) {
                shared_ptr<ParamType<string>> paramString = dynamic_pointer_cast<ParamType<string>>(param);
                fs << paramString->getValue();
            }
            if (dynamic_pointer_cast<ParamType<int>>(param)) {
                shared_ptr<ParamType<int>> paramInt = dynamic_pointer_cast<ParamType<int>>(param);
                fs << paramInt->getValue();
            }
            if (dynamic_pointer_cast<ParamType<double>>(param)) {
                shared_ptr<ParamType<double>> paramReal = dynamic_pointer_cast<ParamType<double>>(param);
                fs << paramReal->getValue();
            }
            if (dynamic_pointer_cast<ParamType<cv::Mat>>(param)) {
                shared_ptr<ParamType<cv::Mat>> paramMat = dynamic_pointer_cast<ParamType<cv::Mat>>(param);
                fs << paramMat->getValue();
            }
        }
    }

    YamlParserCV::NodeType YamlParserCV::getNodeType(const cv::FileNode &node) {

        if (node.isMap()) {
            // check for cv::Mat
            if (isNodeCvMat(node)) {
                return NodeType::CV_MAT;
            }
            return NodeType::MAP_NODE;
        }
        else if (node.isSeq()) {
            cv::FileNode newNode = *(node.begin());
            if (newNode.isMap()) {
                return NodeType::SEQ_NODE;
            }
            else if (newNode.isInt()) {
                return NodeType::SEQ_INT;
            }
            else if (newNode.isString()) {
                return NodeType::SEQ_STR;
            }
            else if (newNode.isReal()) {
                return NodeType::SEQ_REAL;
            }
        }
        else if (node.isString()) {
            return NodeType::STRING;
        }
        else if (node.isInt()) {
            return NodeType::INT;
        }
        else if (node.isReal()) {
            return NodeType::REAL;
        }
        return NodeType::DEFAULT;
    }

    bool YamlParserCV::isNodeCvMat(const cv::FileNode &node) {

//        cv::Mat mat;
//        node >> mat;
//        return mat.rows > 0 && mat.cols > 0;
        bool bcols = false, brows = false;
        if (node.isMap()) {
            vector<string> keys = node.keys();
            for (auto key : keys) {
                if (key == "rows") brows = true;
                if (key == "cols") bcols = true;
            }
        }
        return bcols && brows;
    }

} // NAV24
