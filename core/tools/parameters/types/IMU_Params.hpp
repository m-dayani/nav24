//
// Created by root on 5/13/21.
//

#ifndef NAV24_IMU_PARAMS_HPP
#define NAV24_IMU_PARAMS_HPP

#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include "Parameter.hpp"
#include "YamlParserCV.hpp"


namespace NAV24 {

    struct IMU_Params : public Parameter {
        inline static const std::string TAG{"IMU_Params"};

        IMU_Params();
        explicit IMU_Params(const cv::FileStorage& fSettings);

        void write(cv::FileStorage& fs) const override;
        void read(const cv::FileNode& imuNode) override;

        std::string printStr(const std::string& prefix = "") override;

        bool missParams;

        float freq;
        float Ng, Na, Ngw, Naw;

        float sf;
        //cv::Mat Tbs;
    };

    typedef std::shared_ptr<IMU_Params> IMU_ParamsPtr;

} // NAV24


#endif //NAV24_IMU_PARAMS_HPP
