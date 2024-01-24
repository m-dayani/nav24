//
// Created by root on 5/10/21.
//

#ifndef NAV24_DS_PARAMS_HPP
#define NAV24_DS_PARAMS_HPP

#include <memory>

#include "Parameter.hpp"
#include "YamlParserCV.hpp"


namespace NAV24 {

    class DS_Params;
    typedef std::shared_ptr<DS_Params> DS_ParamsPtr;
    struct DS_Params : public Parameter {

        const std::string &getName() const { return mName; }
        //DS_Formats getFormat() const { return mFormat; }
        const std::string &getType() const { return mType; }

        static DS_ParamsPtr getDsParams(const cv::FileStorage& fsSettings);

    protected:
        std::string mName;
        //DS_Formats mFormat;
        std::string mType;
    };

    struct DS_ParamsOffline : public DS_Params {

        inline static const std::string TAG{"DS_Params"};

        enum DS_Formats {
            NOT_SUPPORTED,
            EUROC,
            EV_ETHZ,
            EV_MVSEC
        };

        DS_ParamsOffline();
        explicit DS_ParamsOffline(const cv::FileStorage& fsSettings);

        static DS_Formats mapDsFormats(const std::string& dsFormat);
        static std::string mapDsFormats(DS_Formats dsFormat);

        // This is absolute or relative to execution path
        std::string getDatasetRoot();
        // All of these are relative to each sequence (root+seq_name)
        std::string getImageFilePath();
        std::string getImageBasePath();
        std::string getIMU_Path();
        std::string getGroundTruthPosePath();
        std::string getEventsPath();
        // This is relative to execution path
        std::string getResultsBasePath();

        void write(cv::FileStorage& fs) const;
        void read(const cv::FileNode& node);

        std::string printStr(const std::string& prefix = "") override;

        int getMaxIter() const { return mnMaxIter; }

        double getTsFactor() const { return mfTsFactor; }

        bool isImuGyroFirst() const { return mbImuGyroFirst; }

        bool isGtQwFirst() const { return mbGtQwFirst; }

        bool isGtPosFirst() const { return mbGtPosFirst; }

        int getSeqTarget() const { return mnSeqTarget; }

        std::vector<std::string> getSeqNames() const { return mvSeqNames; }

    protected:
        void parseDsName(const cv::FileStorage& blueprint, const cv::FileStorage& fsSettings);

        std::map<std::string, std::string> mmPaths;

        int mnSeqTarget;
        std::vector<std::string> mvSeqNames;

        int mnMaxIter;
        double mfTsFactor;

        bool mbImuGyroFirst;
        bool mbGtQwFirst;
        bool mbGtPosFirst;
    };

    struct DS_ParamsStream {

        struct Camera {
            int port;
            std::vector<int> resolution;
            int fps;
        } mCam;
    };

} // NAV24

#endif //NAV24_DS_PARAMS_HPP
