//
// Created by root on 12/21/23.
//

#include "Parameter.hpp"
#include "YamlParserCV.hpp"

#include <glog/logging.h>

using namespace std;


namespace NAV24 {

    Parameter::Parameter(const string &name_, const ParamPtr &parent_) :
        parent(parent_), children(), vChildKeys(), type(NodeType::DEFAULT) {

        this->setName(name_);
    }

    Parameter::Parameter(const string &name_, const ParamPtr &parent_, NodeType type_) :
        Parameter(name_, parent_) {

        type = type_;
    }

    ParamPtr Parameter::getChild(const string &key) {

        if (children.count(key) > 0) {
            return children[key].lock();
        }
        else {
            DLOG(WARNING) << "Parameter::getChild, Parameter node contains no child with key: " << key << "\n";
            return shared_ptr<Parameter>();
        }
    }

    ParamPtr Parameter::read(const string &key) {

        ParamPtr pParam{nullptr};
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
            if (!pParam) {
                break;
            }
        }
        return pParam;
    }

    void Parameter::insertChild(const string& key, const ParamPtr &pChild) {

        if (key.empty()) {
            DLOG(WARNING) << "Parameter::insertChild, cannot insert child with empty key\n";
            return;
        }
        if (children.contains(key)) {
            // update existing param
            ParamPtr pOldChild = children[key].lock();
            pOldChild->parent.lock() = nullptr;
            //pChild->parent = shared_from_this();
            children[key] = pChild;
        }
        else {
            // insert new param
            children.insert(make_pair(key, pChild));
            vChildKeys.push_back(key);
        }
    }

    void Parameter::removeChild(const string &key) {

        if (children.contains(key)) {

            vector<string> newKeys{};
            for (const string& k : vChildKeys) {
                if (key != k) {
                    newKeys.push_back(k);
                }
            }
            children[key].lock()->parent.lock() = nullptr;
            children.erase(key);
        }
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

    string Parameter::mergeKey(const std::vector<std::string> &vKey, const std::string& delim) {

        string res = "";
        size_t n = vKey.size();

        for (size_t i = 0; i < n; i++) {
            string d = (i == n - 1) ? "" : delim;
            res += vKey[i] + d;
        }
        return res;
    }

    /*inline std::string const& Parameter::getName() {

        return name;
    }*/

    inline void Parameter::setName(const string &name_) {

        if (name_.empty()) {
            DLOG(WARNING) << "Parameter::setName, name cannot be empty\n";
            return;
        }
        name = name_;
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
    template std::string ParamType<cv::Mat>::printStr(const std::string &prefix) const;
    template std::string ParamSeq<double>::printStr(const std::string &prefix) const;
    template std::string ParamSeq<string>::printStr(const std::string &prefix) const;


    template<typename T>
    shared_ptr<T> find_param(const std::string& tag, const ParamPtr& pParam) {

        if (pParam) {
            auto pParamNew = pParam->read(tag);
            if (pParamNew && dynamic_pointer_cast<T>(pParamNew)) {
                return dynamic_pointer_cast<T>(pParamNew);
            }
        }
        return nullptr;
    }

    template shared_ptr<ParamType<string>> find_param(const std::string& tag, const ParamPtr& pParam);
    template shared_ptr<ParamType<int>> find_param(const std::string& tag, const ParamPtr& pParam);
    template shared_ptr<ParamType<double>> find_param(const std::string& tag, const ParamPtr& pParam);
    template shared_ptr<ParamType<cv::Mat>> find_param(const std::string& tag, const ParamPtr& pParam);
    template shared_ptr<ParamSeq<string>> find_param(const std::string& tag, const ParamPtr& pParam);
    template shared_ptr<ParamSeq<int>> find_param(const std::string& tag, const ParamPtr& pParam);
    template shared_ptr<ParamSeq<double>> find_param(const std::string& tag, const ParamPtr& pParam);
}