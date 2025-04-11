//
// Created by masoud on 2/22/25.
//

#include <fstream>
#include <boost/filesystem.hpp>
#include <utility>
#include <glog/logging.h>

#include "OP_ObjDetMl.hpp"
#include "ObsMl.hpp"


using namespace std;

namespace NAV24::OP {

    ModelInfo::ModelInfo(ModelInfo::ModelType modelType, const cv::Size &inputShape, const float inputScale,
                         const float inputMean, const float thConf, const float thScore, const float thNms):
            mModelType(modelType), mThConf(thConf), mThScore(thScore), mThNms(thNms),
            mInputShape(inputShape), mInputScale(inputScale), mInputMean(inputMean) {}

    void ModelInfo::getModelInfo(const NAV24::ParamPtr &pParam, ModelInfo& modelInfo) {

        auto pPathModel = find_param<ParamType<string>>("model", pParam);
        modelInfo.modelPath = (pPathModel) ? pPathModel->getValue() : "unknown";

        auto pPathDesc = find_param<ParamType<string>>("config", pParam);
        modelInfo.descPath = (pPathDesc) ? pPathDesc->getValue() : "unknown";

        auto pPathLabels = find_param<ParamType<string>>("labels", pParam);
        modelInfo.labelsPath = (pPathLabels) ? pPathLabels->getValue() : "unknown";

        auto pInputShape = find_param<ParamSeq<int>>("input_shape", pParam);
        vector<int> vInputShape = (pInputShape) ? pInputShape->getValue() : vector<int>();
        if (vInputShape.size() == 2) {
            modelInfo.mInputShape = cv::Size(vInputShape[0], vInputShape[1]);
        }

        auto pInputScale = find_param<ParamType<double>>("input_scale", pParam);
        modelInfo.mInputScale = (pInputScale) ? pInputScale->getValue() : modelInfo.mInputScale;

        auto pInputMean = find_param<ParamType<double>>("input_mean", pParam);
        modelInfo.mInputMean = (pInputMean) ? pInputMean->getValue() : modelInfo.mInputMean;

        auto pThConf = find_param<ParamType<double>>("th_conf", pParam);
        modelInfo.mThConf = (pThConf) ? pThConf->getValue() : modelInfo.mThConf;

        auto pThScore = find_param<ParamType<double>>("th_score", pParam);
        modelInfo.mThScore = (pThScore) ? pThScore->getValue() : modelInfo.mThScore;

        auto pThNms = find_param<ParamType<double>>("th_nms", pParam);
        modelInfo.mThNms = (pThNms) ? pThNms->getValue() : modelInfo.mThNms;
    }

    /* ============================================================================================================== */

    /*ObjDetMlCv::ObjDetMlCv(const std::string &pathModel, const std::string &pathDesc,
                           const std::string &pathLabels, ModelInfo  modelInfo) :
            mPathModel(pathModel), mPathDesc(pathDesc), mModel(), mModelInfo(std::move(modelInfo)) {

        auto pModel = boost::filesystem::path(pathModel);
        if (!boost::filesystem::exists(pModel)) {
            DLOG(WARNING) << "Unable to find model: " << pathModel << ", abort\n";
            return;
        }

        string modelExt = pModel.extension().string();
        if (modelExt == ".pb") {
            // TensorFlow model
            mModelInfo.mModelType = ModelInfo::TENSORFLOW_PB;
            auto pDesc = boost::filesystem::path(pathDesc);
            string ext = pDesc.extension().string();
            if (!boost::filesystem::exists(pDesc) || ext != ".pbtxt") {
                DLOG(WARNING) << "TensorFlow models require a config file: " << pathDesc << ", abort\n";
                return;
            }
            // Tensorflow model: pathModel.pb nad pathDesc.pbtxt[.txt] (config)
            mModel = cv::dnn::readNet(pathModel, pathDesc, "TensorFlow");
        }
        else if (modelExt == ".onnx") {
            mModelInfo.mModelType = ModelInfo::TORCH_ONNX;
            // ONNX model (PyTorch, YOLOvX)
            mModel = cv::dnn::readNet(pathModel);
        }

        readLabels(pathLabels);
    }*/

    void ObjDetMlCv::detect(const ImagePtr& pImage, std::vector<OB::ObsPtr> &vpObs) {

        cv::Mat image = pImage->mImage;
        cv::Size imgSize(image.cols, image.rows);

        // preprocess: create blob from image
        cv::Mat blob;
        preProcess(pImage, blob);

        // set input
        mModel.setInput(blob);

        // forward pass through the model to carry out the detection
        //vector<cv::Mat> outputs;
        cv::Mat output = mModel.forward();

        // post-process results
        if (mModelInfo.mModelType == ModelInfo::TORCH_ONNX) {
            postProcessYolo(output, imgSize, vpObs);
        }
        else {
            postProcessTF(output, imgSize, vpObs);
        }
    }

    void ObjDetMlCv::readLabels(const std::string &pathLabels) {

        ifstream ifs{pathLabels};
        string line;
        mLabels.reserve(1000);
        while (getline(ifs, line)) {
            mLabels.push_back(line);
        }
    }

    void ObjDetMlCv::preProcess(const ImagePtr &pImage, cv::Mat &outBlob) const {

        //create blob from image
        float scale = mModelInfo.mInputScale;
        float mean = mModelInfo.mInputMean;
        cv::Scalar meanRGB = cv::Scalar(mean, mean, mean);
        outBlob = cv::dnn::blobFromImage(pImage->mImage, scale, mModelInfo.mInputShape, meanRGB, true, false);
    }

    void ObjDetMlCv::postProcessTF(const cv::Mat& results, const cv::Size &imgSize,
                                   std::vector<OB::ObsPtr> &vpObs) {

        cv::Mat output = results.clone();
        cv::Mat detections(output.size[2], output.size[3], CV_32F, output.ptr<float>());
        auto img_h = (float) imgSize.height;
        auto img_w = (float) imgSize.width;

        for (int i = 0; i < detections.rows; i++) {
            int class_id = (int) detections.at<float>(i, 1);
            float confidence = detections.at<float>(i, 2);

            // Check if the detection is of good quality
            if (confidence > 0.4){
                int box_x = static_cast<int>(detections.at<float>(i, 3) * img_w);
                int box_y = static_cast<int>(detections.at<float>(i, 4) * img_h);
                int box_width = static_cast<int>(detections.at<float>(i, 5) * img_w - (float) box_x);
                int box_height = static_cast<int>(detections.at<float>(i, 6) * img_h - (float) box_y);
                string className = mLabels[class_id-1];

                auto pObs = make_shared<OB::ObsMl>(className, confidence, cv::Rect(box_x, box_y, box_width, box_height));
                vpObs.push_back(pObs);
            }
        }
    }

    void ObjDetMlCv::postProcessYolo(const cv::Mat& detections, const cv::Size &imgSize,
                                     std::vector<OB::ObsPtr> &vpObs) {

        // Initialize vectors to hold respective outputs while unwrapping detections.
        vector<int> class_ids;
        vector<float> confidences;
        vector<cv::Rect> boxes;

        // Resizing factor.
        float x_factor = (float) imgSize.width / (float) mModelInfo.mInputShape.width;
        float y_factor = (float) imgSize.height / (float) mModelInfo.mInputShape.height;
        auto *data = (float *) detections.data;
        const int dimensions = 85;

        // 25200 for default size 640.
        const int rows = 25200;
//        const int rows = vDetections[0].rows;
        // Iterate through 25200 detections.
        for (int i = 0; i < rows; ++i) {
            float confidence = data[4];
            // Discard bad detections and continue.
            if (confidence >= mModelInfo.mThConf) {
                float *classes_scores = data + 5;
                // Create a 1x85 Mat and store class scores of 80 classes.
                cv::Mat scores(1, (int) mLabels.size(), CV_32FC1, classes_scores);
                // Perform minMaxLoc and acquire the index of best class  score.
                cv::Point class_id;
                double max_class_score;
                cv::minMaxLoc(scores, nullptr, &max_class_score, nullptr, &class_id);
                // Continue if the class score is above the threshold.
                if (max_class_score > mModelInfo.mThScore) {
                    // Store class ID and confidence in the pre-defined respective vectors.
                    confidences.push_back(confidence);
                    class_ids.push_back(class_id.x);
                    // Center.
                    float cx = data[0];
                    float cy = data[1];
                    // Box dimension.
                    float w = data[2];
                    float h = data[3];
                    // Bounding box coordinates.
                    int left = int((cx - 0.5 * w) * x_factor);
                    int top = int((cy - 0.5 * h) * y_factor);
                    int width = int(w * x_factor);
                    int height = int(h * y_factor);
                    // Store good detections in the boxes vector.
                    boxes.emplace_back(left, top, width, height);
                }
            }
            // Jump to the next row.
            data += dimensions;
        }

//        cv::Scalar BLUE = cv::Scalar(255, 0, 0);
//        int THICKNESS = 1;

        // Perform Non-Maximum Suppression and draw predictions.
        vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, mModelInfo.mThScore, mModelInfo.mThNms, indices);
        for (int idx : indices) {

            cv::Rect box = boxes[idx];
//            int left = box.x;
//            int top = box.y;
//            int width = box.width;
//            int height = box.height;
            // Draw bounding box.
//            rectangle(image, cv::Point(left, top), cv::Point(left + width, top + height), BLUE, 3*THICKNESS);
            // Get the label for the class name and its confidence.
            float confidence = confidences[idx];
            string label = mLabels[class_ids[idx]];

            auto pObs = make_shared<OB::ObsMl>(label, confidence, box);
            vpObs.push_back(pObs);
            // Draw class labels.
//            draw_label(image, label, left, top);
        }
//        return image;
    }

    void ObjDetMlCv::setup(const MsgPtr &configMsg) {
//        Operator::setup(configMsg);
        if (configMsg && dynamic_pointer_cast<MsgConfig>(configMsg)) {
            auto pParam = dynamic_pointer_cast<MsgConfig>(configMsg)->getConfig();
            if (pParam) {
                // Model info:
                ModelInfo::getModelInfo(pParam, mModelInfo);

                mPathModel = mModelInfo.modelPath;
                mPathDesc = mModelInfo.descPath;
//                mPathLabels = mModelInfo.labelsPath;

                auto pModel = boost::filesystem::path(mPathModel);
                if (!boost::filesystem::exists(pModel)) {
                    DLOG(WARNING) << "Unable to find model: " << mPathModel << ", abort\n";
                    return;
                }

                string modelExt = pModel.extension().string();
                if (modelExt == ".pb") {
                    // TensorFlow model
                    mModelInfo.mModelType = ModelInfo::TENSORFLOW_PB;
                    auto pDesc = boost::filesystem::path(mPathDesc);
                    string ext = pDesc.extension().string();
                    if (!boost::filesystem::exists(pDesc) || ext != ".pbtxt") {
                        DLOG(WARNING) << "TensorFlow models require a config file: " << mPathDesc << ", abort\n";
                        return;
                    }
                    // Tensorflow model: pathModel.pb nad pathDesc.pbtxt[.txt] (config)
                    mModel = cv::dnn::readNet(mPathModel, mPathDesc, "TensorFlow");
                }
                else if (modelExt == ".onnx") {
                    mModelInfo.mModelType = ModelInfo::TORCH_ONNX;
                    // ONNX model (PyTorch, YOLOvX)
                    mModel = cv::dnn::readNet(mPathModel);
                }

                readLabels(mModelInfo.labelsPath);
            }
        }
    }

}   // NAV24::OP

