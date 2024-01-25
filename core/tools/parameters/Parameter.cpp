//
// Created by root on 12/21/23.
//

#include "Parameter.hpp"
#include "YamlParserCV.hpp"

using namespace std;


namespace NAV24 {

    Parameter::Parameter(const string &name_, const ParamPtr &parent_) : name(name_), parent(parent_), children(), type(0) {}

    ParamPtr Parameter::getChild(const string &key) {
        return children[key].lock();
    }

    ParamPtr Parameter::seekNode(const string &key) {

        ParamPtr pParam;
        bool initialized = false;

        vector<string> vKeys{};
        Parameter::splitKey(vKeys, key);

        for (auto k : vKeys) {
            if (!initialized) {
                pParam = this->getChild(k);
                initialized = true;
            }
            else {
                pParam = pParam->getChild(k);
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

    ParamPtr Parameter::read(const string &key) {
        //todo: implement this
        auto pParam = seekNode(key);
        return pParam;
    }

    void Parameter::write(const string &key, const ParamPtr& paramPtr) {
        // this is wrong (discards child/parent relations)
        // todo: improve this
        ParamPtr pParam = seekNode(key);
        pParam = paramPtr;
    }

    std::string Parameter::printStr(const std::string &prefix) const {

        ostringstream oss{};
        string pref = "";
        if (this->parent.lock()) {
            pref = prefix;
        }
        string sep = " ";
        if (prefix.size() > 0) {
            sep = prefix[0];
        }
        for (auto child : children) {
            oss << "\n" << pref << child.first << ": " << child.second.lock()->printStr(pref + sep);
        }
        return oss.str();
    }

    void Parameter::insertChild(const string& key, const ParamPtr &pChild) {

        children.insert(make_pair(key, pChild));
    }


    template<typename T>
    std::string ParamType<T>::printStr(const std::string &prefix) const {

        ostringstream oss{};
        string pref = prefix;
        if (prefix.size() > 0) {
            pref = " ";
        }
        oss << pref << mData;
        return oss.str();
    }

    template<typename T>
    std::string ParamSeq<T>::printStr(const std::string &prefix) const {

        ostringstream oss{};
        string pref = prefix;
        if (prefix.size() > 0) {
            pref = " ";
        }
        oss << pref << "[";
        for (int i = 0; i < mvData.size(); i++) {
            oss << mvData[i];
            string sep = (i == mvData.size() - 1) ? "]" : ", ";
            oss << sep;
        }
        return oss.str();
    }

    template std::string ParamType<string>::printStr(const std::string &prefix) const;
    template std::string ParamType<int>::printStr(const std::string &prefix) const;
    template std::string ParamType<double>::printStr(const std::string &prefix) const;
    template std::string ParamSeq<int>::printStr(const std::string &prefix) const;
    template std::string ParamSeq<double>::printStr(const std::string &prefix) const;
    template std::string ParamSeq<string>::printStr(const std::string &prefix) const;
    template std::string ParamType<cv::Mat>::printStr(const std::string &prefix) const;
}