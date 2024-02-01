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

#include "Parameter.hpp"


namespace NAV24 {

    class YamlParserCV {
    public:
        static ParamPtr loadParams(const std::string& fileName, std::vector<ParamPtr>& vAllParams);
        static void saveParams(const std::string& fileName, const ParamPtr& rootParam);

    protected:
        static void readParam(const cv::FileNode& node, ParamPtr& pParentParam, std::vector<ParamPtr>& vAllParams);
        static void writeParam(cv::FileStorage& fs, const ParamPtr& param);

        static Parameter::NodeType getNodeType(const cv::FileNode& node);
        static bool isNodeCvMat(const cv::FileNode& node);

        static std::string sanitizeKey(const std::string& key);
    };

} // NAV24

#endif //NAV24_YAMLPARSERCV_HPP
