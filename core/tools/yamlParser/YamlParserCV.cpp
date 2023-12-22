//
// Created by root on 5/8/21.
//

#include "YamlParserCV.hpp"

namespace NAV24 {

    bool YamlParserCV::readBool(const cv::FileNode &n, const std::string &strArg, const bool defVal) {

        cv::FileNode node = n[strArg];

        if (node.empty())
            return defVal;

        if (node.isInt())
            return ((int) node) != 0;

        if (node.isString())
            return (node.string() == "true");
        return defVal;
    }

    /*void YamlParserCV::writeBool(cv::FileNode &n, const std::string &strArg, const bool val) {

        n[strArg] = (val) ? "true" : "false";
    }*/

    bool YamlParserCV::readBool(const cv::FileStorage &fs, const std::string &strArg, const bool defVal) {

        cv::FileNode node = fs[strArg];

        if (node.empty())
            return defVal;

        if (node.isInt())
            return ((int) node) != 0;

        if (node.isString())
            return (node.string() == "true");
        return defVal;
    }

    void YamlParserCV::writeBool(cv::FileStorage &fs, const std::string &strArg, const bool val) {

        fs << strArg << val;
    }

    int YamlParserCV::readInt(const cv::FileNode &n, const std::string &strArg, const int &defVal) {

        cv::FileNode node = n[strArg];

        if (!node.empty() && node.isInt())
            return (int) node;
        return defVal;
    }

    int YamlParserCV::readInt(const cv::FileStorage &fs, const std::string &strArg, const int &defVal) {

        cv::FileNode node = fs[strArg];

        if (!node.empty() && node.isInt())
            return (int) node;
        return defVal;
    }

    /*void YamlParserCV::writeInt(cv::FileNode &n, const std::string &strArg, const int &val) {

    }*/

    void YamlParserCV::writeInt(cv::FileStorage &fs, const std::string &strArg, const int &val) {

        fs << strArg << val;
    }

    /*void YamlParserCV::writeReal(cv::FileNode &n, const std::string &strArg, const double &val,
                                 const std::string &prec) {
    }*/

    void YamlParserCV::writeReal(cv::FileStorage &fs, const std::string &strArg, const double &val,
                                 const std::string &prec) {
        fs << strArg << val;
    }

    std::string
    YamlParserCV::readString(const cv::FileStorage &fs, const std::string &strArg, const std::string &defVal) {

        cv::FileNode node = fs[strArg];

        if (!node.empty() && node.isString())
            return node.string();
        return defVal;
    }

    void YamlParserCV::writeString(cv::FileStorage &fs, const std::string &strArg, const std::string &val) {

        fs << strArg << val;
    }

    std::string YamlParserCV::readString(const cv::FileNode &n, const std::string &strArg, const std::string &defVal) {

        cv::FileNode node = n[strArg];

        if (!node.empty() && node.isString())
            return node.string();
        return defVal;
    }

    /*void YamlParserCV::writeString(cv::FileNode &n, const std::string &strArg, const std::string &val) {

    }*/

    cv::Mat YamlParserCV::readMat(const cv::FileStorage &fs, const std::string &strArg, const cv::Mat &defMat) {

        cv::Mat outMat = defMat.clone();
        fs[strArg] >> outMat;
        return outMat.clone();
    }

    void YamlParserCV::writeMat(cv::FileStorage &fs, const std::string &strArg, const cv::Mat &inCvMat) {

        fs << strArg << inCvMat;
    }

    cv::Mat YamlParserCV::readMat(const cv::FileNode &n, const std::string &strArg, const cv::Mat &defMat) {

        cv::Mat outMat = defMat.clone();
        n[strArg] >> outMat;
        return outMat.clone();
    }

    /*void YamlParserCV::writeMat(cv::FileNode &n, const std::string &strArg, const cv::Mat &inCvMat) {

        n << strArg << inCvMat;
    }*/

} // NAV24
