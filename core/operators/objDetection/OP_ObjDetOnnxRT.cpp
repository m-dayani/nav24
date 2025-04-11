//
// Created by masoud on 2/24/25.
//

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <regex>
#include <iostream>
#include <thread>
#include <boost/filesystem.hpp>
#include <glog/logging.h>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include "OP_ObjDetOnnxRT.hpp"
#include "ObsMl.hpp"

using namespace std;

namespace NAV24::OP {

    /*ObjDetOnnxRT::ObjDetOnnxRT(const std::string &pathModel, const std::string &pathLabels,
                               ModelInfo modelInfo) :
            mPathModel(pathModel), mPathLabels(pathLabels), mModelInfo(std::move(modelInfo)) {

        boost::filesystem::path pModel(pathModel);
        if (!boost::filesystem::exists(pModel) || pModel.extension().string() != ".onnx") {
            DLOG(WARNING) << "ObjDetOnnxRT::ObjDetOnnxRT, Bad model path: " << pathModel << "\n";
            return;
        }

        try {
#ifdef LIB_ONNX_RUNTIME_FOUND
            env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "Yolo");
            Ort::SessionOptions sessionOption;
            if (mModelInfo.cudaEnable) {

                cudaEnable = mModelInfo.cudaEnable;
                OrtCUDAProviderOptions cudaOption;
                cudaOption.device_id = 0;
                sessionOption.AppendExecutionProvider_CUDA(cudaOption);
            }
            sessionOption.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
            sessionOption.SetIntraOpNumThreads(mModelInfo.intraOpNumThreads);
            sessionOption.SetLogSeverityLevel(mModelInfo.logSeverityLevel);

            session = make_unique<Ort::Session>(env, mPathModel.c_str(), sessionOption);
            Ort::AllocatorWithDefaultOptions allocator;
            size_t inputNodesNum = session->GetInputCount();
            for (size_t i = 0; i < inputNodesNum; i++)
            {
                Ort::AllocatedStringPtr input_node_name = session->GetInputNameAllocated(i, allocator);
                char* temp_buf = new char[50];
                strcpy(temp_buf, input_node_name.get());
                inputNodeNames.push_back(temp_buf);
            }
            size_t OutputNodesNum = session->GetOutputCount();
            for (size_t i = 0; i < OutputNodesNum; i++)
            {
                Ort::AllocatedStringPtr output_node_name = session->GetOutputNameAllocated(i, allocator);
                char* temp_buf = new char[10];
                strcpy(temp_buf, output_node_name.get());
                outputNodeNames.push_back(temp_buf);
            }
            options = Ort::RunOptions{ nullptr };
            //WarmUpSession();
#endif
        }
        catch (const std::exception& e) {

            const char* str1 = "[YOLO_V8]:";
            const char* str2 = e.what();
            std::string resultStr = std::string(str1) + std::string(str2);
            char* merged = new char[resultStr.length() + 1];
            std::strcpy(merged, resultStr.c_str());
            std::cout << merged << std::endl;
            delete[] merged;
//            return "[YOLO_V8]:Create session failed.";
        }

        readLabels(pathLabels);
    }*/

    void ObjDetOnnxRT::readLabels(const std::string &pathLabels) {

        ifstream ifs{pathLabels};
        string line;
        mLabels.reserve(1000);
        while (getline(ifs, line)) {
            mLabels.push_back(line);
        }
    }

    int64_t elements1(const std::vector<int64_t> &shape) {

        int64_t target = 1;
        for (auto item : shape) target *= item;
        return target;
    }

    void ObjDetOnnxRT::detect(const ImagePtr &pImage, std::vector<OB::ObsPtr> &vpObs) {

        cv::Mat image = pImage->mImage;
        double img_width = image.cols;
        double img_height = image.rows;

//        std::vector<Results> results;
#ifdef LIB_ONNX_RUNTIME_FOUND
        if (!session) {
            DLOG(WARNING) << "ObjTrYoloOnnx::detect, NULL session, abort\n";
            return;
        }

        Array blob;
        preProcess(pImage, blob);

        if (mInputShape.size() == 3) mInputShape.insert(mInputShape.begin(), 1);
        auto iw = static_cast<double>(mInputShape.at(2)), ih = static_cast<double>(mInputShape.at(3));
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        auto input = Ort::Value::CreateTensor<float>(memory_info, blob.data(), elements1(mInputShape),
                                                     mInputShape.data(), mInputShape.size());

        //const char *input_names[] = {"images"};
        //const char *output_names[] = {"output"};
        auto output = session->Run({}, inputNodeNames.data(), &input,
                                   1, outputNodeNames.data(), outputNodeNames.size());

//        const char** input_names = inputNodeNames.data();
//        const char** output_names = outputNodeNames.data();
//        Ort::Value output_tensor{nullptr};
//        session->Run(Ort::RunOptions{nullptr}, input_names, &input, 1, output_names, &output_tensor, 1);

        vpObs.reserve(vpObs.size() + output.size());
//        Shape shape;

//        results.resize(output.size());
        for (auto & res : output) {

//            shape = i.GetTensorTypeAndShapeInfo().GetShape();
            auto ptr = res.GetTensorData<float>();

//            Results &temp = results[i];
//            temp.resize(mInputShape[0]);
            for (int64_t i = 0; i < mInputShape[0]; i++) {

                double x = img_width * ptr[1] / iw;
                double y = img_height * ptr[2] / ih;
                double w = img_width * (ptr[3] - ptr[1]) / iw;
                double h = img_height * (ptr[4] - ptr[2]) / ih;
                auto index = static_cast<int64_t>(ptr[5]);
                string name = mLabels[index];
                float confidence = ptr[6];
                ptr += mInputShape[1];

                cv::Rect bbox((int) x, (int) y, (int) w, (int) h);
                auto pObs = make_shared<OB::ObsMl>(name, confidence, bbox);
                vpObs.push_back(pObs);
            }
        }
#endif
    }

    void ObjDetOnnxRT::preProcess(const ImagePtr &pImage, Array &blob) {

        cv::Mat image = pImage->mImage;
//        mInputShape = {1, image.channels(), image.rows, image.cols};
        mInputShape = {1, image.channels(), mModelInfo.mInputShape.height, mModelInfo.mInputShape.width};
        float scale = mModelInfo.mInputScale;
        float mean = mModelInfo.mInputMean;
        cv::Scalar meanRGB = cv::Scalar(mean, mean, mean);
        cv::Mat nchw = cv::dnn::blobFromImage(pImage->mImage, scale, mModelInfo.mInputShape, meanRGB, true, false);
        blob = Array(nchw.ptr<float>(), nchw.ptr<float>() + nchw.total());
    }

    void ObjDetOnnxRT::setup(const MsgPtr &configMsg) {
//        Operator::setup(configMsg);

        if (configMsg && dynamic_pointer_cast<MsgConfig>(configMsg)) {
            auto pParam = dynamic_pointer_cast<MsgConfig>(configMsg)->getConfig();
            if (pParam) {
                // Model info:
                ModelInfo::getModelInfo(pParam, mModelInfo);

                mPathModel = mModelInfo.modelPath;
//                mPathDesc = mModelInfo.descPath;
                mPathLabels = mModelInfo.labelsPath;

                boost::filesystem::path pModel(mPathModel);
                if (!boost::filesystem::exists(pModel) || pModel.extension().string() != ".onnx") {
                    DLOG(WARNING) << "ObjDetOnnxRT::ObjDetOnnxRT, Bad model path: " << mPathModel << "\n";
                    return;
                }

                try {
#ifdef LIB_ONNX_RUNTIME_FOUND
                    env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "Yolo");
                    Ort::SessionOptions sessionOption;
                    if (mModelInfo.cudaEnable) {

                        cudaEnable = mModelInfo.cudaEnable;
                        OrtCUDAProviderOptions cudaOption;
                        cudaOption.device_id = 0;
                        sessionOption.AppendExecutionProvider_CUDA(cudaOption);
                    }
                    sessionOption.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
                    sessionOption.SetIntraOpNumThreads(mModelInfo.intraOpNumThreads);
                    sessionOption.SetLogSeverityLevel(mModelInfo.logSeverityLevel);

                    session = make_unique<Ort::Session>(env, mPathModel.c_str(), sessionOption);
                    Ort::AllocatorWithDefaultOptions allocator;
                    size_t inputNodesNum = session->GetInputCount();
                    for (size_t i = 0; i < inputNodesNum; i++) {
                        Ort::AllocatedStringPtr input_node_name = session->GetInputNameAllocated(i, allocator);
                        char *temp_buf = new char[50];
                        strcpy(temp_buf, input_node_name.get());
                        inputNodeNames.push_back(temp_buf);
                    }
                    size_t OutputNodesNum = session->GetOutputCount();
                    for (size_t i = 0; i < OutputNodesNum; i++) {
                        Ort::AllocatedStringPtr output_node_name = session->GetOutputNameAllocated(i, allocator);
                        char *temp_buf = new char[10];
                        strcpy(temp_buf, output_node_name.get());
                        outputNodeNames.push_back(temp_buf);
                    }
                    options = Ort::RunOptions{nullptr};
                    //WarmUpSession();
#endif
                }
                catch (const std::exception &e) {

                    const char *str1 = "[YOLO_V8]:";
                    const char *str2 = e.what();
                    std::string resultStr = std::string(str1) + std::string(str2);
                    char *merged = new char[resultStr.length() + 1];
                    std::strcpy(merged, resultStr.c_str());
                    std::cout << merged << std::endl;
                    delete[] merged;
//            return "[YOLO_V8]:Create session failed.";
                }

                readLabels(mPathLabels);
            }
        }
    }


} // NAV24::OP
