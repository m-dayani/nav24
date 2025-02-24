//
// Created by masoud on 2/22/25.
//

#ifndef NAV24_OP_OBJDETML_HPP
#define NAV24_OP_OBJDETML_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>

#include "OP_ObjDet.hpp"


namespace NAV24::OP {

#define SCORE_THRESHOLD 0.5f
#define NMS_THRESHOLD 0.45f
#define CONFIDENCE_THRESHOLD 0.45f
#define INPUT_WIDTH 640
#define INPUT_HEIGHT 640

    struct ModelInfo {
        enum ModelType {
            UNKNOWN,
            TENSORFLOW_PB,
            TORCH_ONNX
        };

        ModelInfo() : mModelType(UNKNOWN), mInputShape(INPUT_WIDTH, INPUT_HEIGHT) {}

        ModelInfo(ModelType modelType, const cv::Size& inputShape,
                  float inputScale=1.f, float inputMean=0,
                  float thConf=CONFIDENCE_THRESHOLD,
                  float thScore=SCORE_THRESHOLD,
                  float thNms=NMS_THRESHOLD);

        ModelType mModelType;

        float mThConf = CONFIDENCE_THRESHOLD;
        float mThScore = SCORE_THRESHOLD;
        float mThNms = NMS_THRESHOLD;

        cv::Size mInputShape;
        float mInputScale = 1.f;
        float mInputMean = 0;
    };

    class ObjDetMlCv : public ObjDet {
    public:
        ObjDetMlCv(const std::string& pathModel, const std::string& pathDesc,
                   const std::string& pathLabels, const ModelInfo& modelInfo);

        void detect(const ImagePtr& pImage, std::vector<OB::ObsPtr> &vpObs) override;

    private:
        void readLabels(const std::string& pathLabels);
        void preProcess(const ImagePtr& pImage, cv::Mat& outBlob);
        void postProcessTF(const std::vector<cv::Mat>& vDetections, const cv::Size &imgSize,
                           std::vector<OB::ObsPtr> &vpObs);
        void postProcessYolo(const std::vector<cv::Mat>& vDetections, const cv::Size &imgSize,
                             std::vector<OB::ObsPtr> &vpObs);

    private:
        std::vector<std::string> mLabels;

        std::string mPathModel;
        std::string mPathDesc;
        cv::dnn::Net mModel;

        ModelInfo mModelInfo;

    };
}   // NAV24::OP

#endif //NAV24_OP_OBJDETML_HPP
