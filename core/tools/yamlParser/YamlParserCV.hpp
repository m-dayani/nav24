//
// Created by root on 5/8/21.
//

#ifndef NAV24_YAMLPARSERCV_HPP
#define NAV24_YAMLPARSERCV_HPP

#include <iostream>
#include <sstream>
#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>


namespace NAV24 {

    class YamlParserCV {
    public:
        static bool readBool(const cv::FileNode &n, const std::string &strArg, bool defVal = false);
        //static void writeBool(cv::FileNode &n, const std::string &strArg, bool val);
        static bool readBool(const cv::FileStorage &fs, const std::string &strArg, bool val);
        static void writeBool(cv::FileStorage &fs, const std::string &strArg, bool defVal = false);

        static int readInt(const cv::FileNode &n, const std::string &strArg, const int &defVal = 0);
        //static void writeInt(cv::FileNode &n, const std::string &strArg, const int &val);
        static int readInt(const cv::FileStorage &fs, const std::string &strArg, const int &defVal = 0);
        static void writeInt(cv::FileStorage &fs, const std::string &strArg, const int &val);

        template<typename T>
        static T readReal(const cv::FileNode &n, const std::string &strArg, const T &defVal = 0.0) {

            cv::FileNode node = n[strArg];

            if (!std::is_floating_point<T>::value || node.empty() || !(node.isReal() || node.isInt()))
                return defVal;
            return static_cast<T>(node.real());
        }
        template<typename T>
        static T readReal(const cv::FileStorage &fs, const std::string &strArg, const T &defVal = 0.0) {

            cv::FileNode node = fs[strArg];

            if (!std::is_floating_point<T>::value || node.empty() || !(node.isReal() || node.isInt()))
                return defVal;
            return static_cast<T>(node.real());
        }
        //static void writeReal(cv::FileNode &n, const std::string &strArg, const double &val, const std::string& prec = "double");
        static void writeReal(cv::FileStorage &fs, const std::string &strArg,
                              const double &val, const std::string& prec = "double");

        static std::string readString(const cv::FileStorage &fs, const std::string &strArg,
                                      const std::string &defVal = std::string());
        static void writeString(cv::FileStorage &fs, const std::string &strArg, const std::string &val);
        static std::string readString(const cv::FileNode &n, const std::string &strArg,
                                      const std::string &defVal = std::string());
        //static void writeString(cv::FileNode &n, const std::string &strArg, const std::string &val);

        static cv::Mat readMat(const cv::FileStorage &fs, const std::string &strArg, const cv::Mat& defMat);
        static void writeMat(cv::FileStorage &fs, const std::string &strArg, const cv::Mat& inCvMat);
        static cv::Mat readMat(const cv::FileNode &n, const std::string &strArg, const cv::Mat& defMat);
        //static void writeMat(cv::FileNode &n, const std::string &strArg, const cv::Mat& inCvMat);

        template<typename T>
        static uint readSequence(const cv::FileStorage &fs, const std::string &strArg, std::vector<T> &outSeq) {

            unsigned int cnt = 0;
            cv::FileNode n = fs[strArg];

            if (n.empty() || !n.isSeq()) {
                return cnt;
            }
            cv::FileNodeIterator it = n.begin(), it_end = n.end();
            for (; it != it_end; ++it) {
                outSeq.push_back((T) *it);
                cnt++;
            }
            return cnt;
        }
        template<typename T>
        static uint writeSequence(cv::FileStorage &fs, const std::string &strArg, const std::vector<T> &inSeq) {

            fs << strArg << "[";
            for (const T& seqVal : inSeq) {
                fs << seqVal;
            }
            fs << "]";
        }
        template<typename T>
        static uint readSequence(const cv::FileNode &node, const std::string &strArg, std::vector<T> &outSeq) {

            unsigned int cnt = 0;
            cv::FileNode n = node[strArg];

            if (n.empty() || !n.isSeq()) {
                return cnt;
            }
            cv::FileNodeIterator it = n.begin(), it_end = n.end();
            for (; it != it_end; ++it) {
                outSeq.push_back((T) *it);
                cnt++;
            }
            return cnt;
        }
        //template<typename T>
        //static uint writeSequence(cv::FileNode &n, const std::string &strArg, const std::vector<T> &inSeq);

        template<typename T>
        static uint readMap(const cv::FileStorage &fs, const std::string &strArg, std::map<std::string, T> &outSeq) {

            unsigned int cnt = 0;
            cv::FileNode n = fs[strArg];

            if (n.empty() || !n.isMap()) {
                return cnt;
            }
            cv::FileNodeIterator it = n.begin(), it_end = n.end();
            for (; it != it_end; ++it) {
                outSeq.insert(std::make_pair((*it).name(), (T) *it));
                cnt++;
            }
            return cnt;
        }
        template<typename T>
        static uint writeMap(cv::FileStorage &fs, const std::string &strArg, const std::map<std::string, T> &inSeq) {

            fs << strArg << "{";
            for (const std::pair<std::string, T>& seqVal : inSeq) {
                fs << seqVal.first << seqVal.second;
            }
            fs << "}";
        }
        template<typename T>
        static uint readMap(const cv::FileNode &node, const std::string &strArg, std::map<std::string, T> &outSeq) {

            unsigned int cnt = 0;
            cv::FileNode n = node[strArg];

            if (n.empty() || !n.isMap()) {
                return cnt;
            }
            cv::FileNodeIterator it = n.begin(), it_end = n.end();
            for (; it != it_end; ++it) {
                outSeq.insert(std::make_pair((*it).name(), (T) *it));
                cnt++;
            }
            return cnt;
        }
        //template<typename T>
        //static uint writeMap(cv::FileNode &n, const std::string &strArg, const std::map<std::string, T> &inSeq);

        static std::string printNode(const cv::FileNode &node, int depth = 0);

        static void saveParams(const cv::FileNode& fNode, cv::FileStorage& fs);

    protected:
        static void printNode(const cv::FileNode &node, std::ostringstream& oss, int depth = 0);
    };

} // NAV24

#endif //NAV24_YAMLPARSERCV_HPP
