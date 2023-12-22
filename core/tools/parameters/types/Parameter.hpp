//
// Created by root on 12/21/23.
//

#ifndef NAV24_PARAMETER_HPP
#define NAV24_PARAMETER_HPP

#include <memory>
#include <string>

#include <opencv2/core/core.hpp>


namespace NAV24 {

    class Parameter {
    public:
        virtual void write(cv::FileStorage& fs) const = 0;
        virtual void read(const cv::FileNode& node) = 0;

        virtual std::string printStr(const std::string& prefix = "") = 0;
    };

    typedef std::shared_ptr<Parameter> ParamPtr;

}

#endif //NAV24_PARAMETER_HPP
