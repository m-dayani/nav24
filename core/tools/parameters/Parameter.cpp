//
// Created by root on 12/21/23.
//

#include "Parameter.hpp"
#include "YamlParserCV.hpp"

using namespace std;


namespace NAV24 {

    std::string ParamCV::printStr(const std::string &prefix) {

        return YamlParserCV::printNode(mFileNode);
    }

    void ParamCV::write(const string &key, cv::FileNode& write_node) const {

        auto newNode = Parameter::seekNode(write_node, key);
        if (newNode)
            *newNode = mFileNode;
    }

    ParamPtr ParamCV::read(const string &key) {

        ParamPtr pParam = nullptr;
        auto pNewNode = Parameter::seekNode(mFileNode, key);
        if (!pNewNode)
            return nullptr;
        auto node = *pNewNode;

        if (node.isMap()) {
            pParam = make_shared<ParamCV>(node);
        }
        else {
            cv::FileNode newNode = node;
            if (node.isSeq()) {
                newNode = *(node.begin());
            }

            if (newNode.isInt()) {
                shared_ptr<ParamSeq<int>> pParamInt = make_shared<ParamSeq<int>>(ParamSeq<int>::SeqType::INT);
                if (node.isSeq()) {
                    for (auto iter = node.begin(); iter != node.end(); iter++) {
                        pParamInt->push_back((int) *iter);
                    }
                }
                else {
                    pParamInt->push_back((int) node);
                }
                pParam = pParamInt;
            }
            else if (newNode.isString()) {
                shared_ptr<ParamSeq<string>> pParamStr = make_shared<ParamSeq<string>>(ParamSeq<string>::SeqType::STRING);
                if (node.isSeq()) {
                    for (auto iter = node.begin(); iter != node.end(); iter++) {
                        pParamStr->push_back((string) *iter);
                    }
                }
                else {
                    pParamStr->push_back((string) node);
                }
                pParam = pParamStr;
            }
            else if (newNode.isReal()) {
                shared_ptr<ParamSeq<double>> pParamFloat = make_shared<ParamSeq<double>>(ParamSeq<double>::SeqType::DOUBLE);
                if (node.isSeq()) {
                    for (auto iter = node.begin(); iter != node.end(); iter++) {
                        pParamFloat->push_back((double) *iter);
                    }
                }
                else {
                    pParamFloat->push_back((double) node);
                }
                pParam = pParamFloat;
            }

            if (pParam) {
                static_pointer_cast<ParamCV>(pParam)->setNode(node);
            }
        }

        return pParam;
    }

    void Parameter::splitKey(std::vector<std::string> &vKeys, const string &s, const std::string& delim) {

        string key = s;
        size_t pos = 0;
        std::string token;

        while ((pos = key.find(delim)) != std::string::npos) {
            token = key.substr(0, pos);
            vKeys.push_back(token);
            key.erase(0, pos + delim.length());
        }
        vKeys.push_back(key);
    }

    string &Parameter::mergeKey(const std::vector<std::string> &vKey, const std::string& delim) {

        string res = "";
        size_t n = vKey.size();


        for (size_t i = 0; i < n; i++) {

            string d = (i == n - 1) ? "" : delim;
            res += vKey[i] + d;
        }

        return res;
    }

    bool is_number(const std::string& s) {
        std::string::const_iterator it = s.begin();
        while (it != s.end() && std::isdigit(*it)) ++it;
        return !s.empty() && it == s.end();
    }

    cv::FileNode* Parameter::seekNode(const cv::FileNode& nd, const std::string& key) {

        if (nd.isNone()) {
            return nullptr;
        }

        std::vector<std::string> vKeys{};
        Parameter::splitKey(vKeys, key);

        cv::FileNode node = nd;

        for (auto k : vKeys) {
            if (is_number(k)) {
                node = node[std::stoi(k)];
            }
            else {
                node = node[k];
            }
        }

        auto newNode = &node;
        return newNode;
    }

    template<typename T>
    std::string ParamSeq<T>::printStr(const std::string &prefix) {

        ostringstream oss{};

        oss << "[" << prefix;

        for (size_t i = 0; i < mData.size(); i++) {

            string d = (i == mData.size() - 1) ? "]" : ", ";
            oss << mData[i] << d;
        }

        return oss.str();
    }

}