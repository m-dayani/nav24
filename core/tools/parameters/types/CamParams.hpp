//
// Created by root on 5/10/21.
//

#ifndef NAV24_CAMPARAMS_HPP
#define NAV24_CAMPARAMS_HPP

#include <string>
#include <vector>
#include <map>
#include <memory>

#include <opencv2/core/core.hpp>

#include "YamlParserCV.hpp"
#include "Parameter.hpp"


namespace NAV24 {

    struct CameraType {

        enum CameraTypeEnum {
            NOT_SUPPORTED,
            PINHOLE,
            PINHOLE_DISTORTED,
            FISHEYE,
            KANNALA_BRANDT8
        };

        CameraType() : mCamType(NOT_SUPPORTED) {}
        explicit CameraType(CameraTypeEnum camType) : mCamType(camType) {}
        explicit CameraType(const cv::FileNode& cameraNode);
        explicit CameraType(CameraType* pCamType);

        void write(cv::FileStorage& fs) const;
        void read(const cv::FileNode& node);

        bool isFishEye() const;
        bool isPinHole() const;

        static std::string mapCameraType(CameraTypeEnum camType);
        static CameraTypeEnum mapCameraType(const std::string& camType);

        std::string printStr(const std::string& prefix = std::string()) const;

        CameraTypeEnum mCamType;
    };
    typedef std::shared_ptr<CameraType> CameraTypePtr;

    struct CalibParams {

        enum DistortionType {
            NONE,
            RADIAL_TANGENTIAL,
            EQUIDISTANT
        };

        CalibParams();
        explicit CalibParams(const cv::FileNode& calibNode);
        explicit CalibParams(CalibParams* pCalibParams);

        bool checkIntrinsics();
        bool checkDistCoefs();

        void initK();
        void initD();

        static DistortionType mapDistType(const std::string& distType);
        static std::string mapDistType(DistortionType distType);

        void write(cv::FileStorage& fs) const;
        void read(const cv::FileNode& calibNode);

        std::string printStr(const std::string& prefix = std::string()) const;

        //std::map<std::string, float> mmIntrinsics;
        //std::map<std::string, float> mmDistCoefs;
        bool missParams;

        DistortionType mDistType;

        std::vector<float> mvIntrinsics;
        std::vector<float> mvDistCoefs;

        cv::Mat mK; // Intrinsic matrix
        cv::Mat mD; // Distortion coefficients
        cv::Mat mR; // Rectification matrix
        cv::Mat mP; // Projection matrix

        std::vector<std::string> mvDistCoefKeys;
        static std::vector<std::string> mvIntrinsicsKeys;
        static std::vector<std::string> mvRTDistKeys;
        static std::vector<std::string> mvKBDistKeys;
    };
    typedef std::shared_ptr<CalibParams> CalibParamsPtr;

    struct CamParams : public Parameter {

        inline static const std::string TAG{"CamParams"};

        CamParams();
        explicit CamParams(const cv::FileStorage& fSettings);
        explicit CamParams(CamParams* pCamParams);
        ~CamParams();

        void write(cv::FileStorage& fs, CamParams* pCamParams);
        void read(const cv::FileNode& node, CamParams* pCamParams);

        static std::string printStr(CamParams* pCamParams, const std::string& prefix = std::string());

        bool missParams;

        void write(cv::FileStorage &fs) const override;

        void read(const cv::FileNode &node) override;

        std::string printStr(const std::string &prefix) override;

        cv::Size mImageSize;

        bool leftCam;
        int mLappingBegin, mLappingEnd;

        float mbf;
        cv::Mat mTbc;

        float fps;
        float mMinFrames, mMaxFrames;

        bool mbRGB;

        float mThDepth, mDepthMapFactor, mThFarPoints;

        CameraTypePtr mpCameraType;
        CalibParamsPtr mpCalibParams;

        CamParams* mpLinkedCam;
    };
    typedef std::shared_ptr<CamParams> CamParamsPtr;

} // NAV24

#endif //NAV24_CAMPARAMS_HPP
