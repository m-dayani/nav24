//
// Created by masoud on 5/2/24.
//

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <memory>
#include <string>
#include <utility>
#include <regex>
#include <iostream>
#include <thread>
#include <glog/logging.h>

#include "Image.hpp"
#include "FrontEnd.hpp"
#include "OP_ObjTrackingYolo.hpp"

using namespace std;


namespace NAV24::OP {

#define RET_OK nullptr

    const char *class_names[] = {
            "person",         "bicycle",    "car",           "motorcycle",    "airplane",     "bus",           "train",
            "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",    "parking meter", "bench",
            "bird",           "cat",        "dog",           "horse",         "sheep",        "cow",           "elephant",
            "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",     "handbag",       "tie",
            "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball",  "kite",          "baseball bat",
            "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",       "wine glass",    "cup",
            "fork",           "knife",      "spoon",         "bowl",          "banana",       "apple",         "sandwich",
            "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",        "donut",         "cake",
            "chair",          "couch",      "potted plant",  "bed",           "dining table", "toilet",        "tv",
            "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",   "microwave",     "oven",
            "toaster",        "sink",       "refrigerator",  "book",          "clock",        "vase",          "scissors",
            "teddy bear",     "hair drier", "toothbrush"};

    const char *det_class[] = {"cap"};

#ifdef _WIN32   // ORTCHAR_T is wchar_t when _WIN32 is defined.
#define TO_ONNX_STR(stdStr) std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>>().from_bytes(stdStr).c_str()
#else
#define TO_ONNX_STR(stdStr) stdStr.c_str()
#endif

    Ort::SessionOptions create_options(int64_t cuda_device) {

        Ort::SessionOptions options;
        if (cuda_device >= 0)
            Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_CUDA(options,static_cast<int>(cuda_device)));
        return options;
    }

    struct Impl {

        Ort::Env env;
        Ort::SessionOptions options;
        Ort::Session session;

        Impl(const std::string &model_path, int64_t cuda_device)
                : env(ORT_LOGGING_LEVEL_WARNING, "YOLOv7_CXX"),
                  options(create_options(cuda_device)),
                  session(env, TO_ONNX_STR(model_path), options)
        {}
    };

    int64_t elements(const std::vector<int64_t> &shape) {

        int64_t target = 1;
        for (auto item : shape) target *= item;
        return target;
    }

    ObjTrYoloOnnx::ObjTrYoloOnnx(const ChannelPtr& pChannel) :
        ObjTracking(pChannel), mCudaDevice(-1), imgSize(),
        modelType(), mqpImages(), mMtxImgBuff(), mLastTs(-1.0) {}

    int64_t ObjTrYoloOnnx::image_size() const {

        if (session) {
            auto info = session->GetInputTypeInfo(0);
            return info.GetTensorTypeAndShapeInfo().GetShape().back();
        }
        else {
            return 640;
        }
    }

    std::vector<Results> ObjTrYoloOnnx::detect(float *data, Shape shape) {

        std::vector<Results> results;
        if (!session) {
            DLOG(WARNING) << "ObjTrYoloOnnx::detect, NULL session, abort\n";
            return results;
        }
        if (shape.size() == 3) shape.insert(shape.begin(), 1);
        auto iw = static_cast<double>(shape.at(2)), ih = static_cast<double>(shape.at(3));
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        auto input = Ort::Value::CreateTensor<float>(memory_info, data, elements(shape), shape.data(), shape.size());

        //const char *input_names[] = {"images"};
        //const char *output_names[] = {"output"};
        auto output = session->Run({}, inputNodeNames.data(), &input,
                                   1, outputNodeNames.data(), outputNodeNames.size());

        results.resize(output.size());
        for (size_t i = 0; i < output.size(); ++i) {

            shape = output[i].GetTensorTypeAndShapeInfo().GetShape();
            auto ptr = output[i].GetTensorData<float>();

            Results &temp = results[i];
            temp.resize(shape[0]);
            for (auto &d : temp) {

                d.x = ptr[1] / iw;
                d.y = ptr[2] / ih;
                d.w = (ptr[3] - ptr[1]) / iw;
                d.h = (ptr[4] - ptr[2]) / ih;
                d.index = static_cast<int64_t>(ptr[5]);
                d.name = class_names[d.index];
                d.confidence = ptr[6];
                ptr += shape[1];
            }
        }
        return results;
    }

    std::pair<Array, Shape> ObjTrYoloOnnx::convert_image(const cv::Mat &image) {

        Shape shape = {1, image.channels(), image.rows, image.cols};
        cv::Mat nchw = cv::dnn::blobFromImage(image, 1.0, {}, {}, true) / 255.f;
        Array array(nchw.ptr<float>(), nchw.ptr<float>() + nchw.total());
        return {array, shape};
    }

    void display_image(cv::Mat image, const std::vector<Result> &detections) {

        auto w = image.cols, h = image.rows;

        for (const auto &d : detections) {

            auto color = cv::Scalar(255, 255, 255);  // CV_RGB(rand() % 256, rand() % 256, rand() % 256)
            auto name = d.name + ":" + std::to_string(int(d.confidence * 100)) + "%";

            int dx = static_cast<int>(d.x * w);
            int dy = static_cast<int>(d.y * h);
            int dw = static_cast<int>(d.w * w);
            int dh = static_cast<int>(d.h * h);

            cv::rectangle(image, cv::Rect(dx, dy, dw, dh), color);
            cv::putText(image, name, cv::Point(dx, dy), cv::FONT_HERSHEY_DUPLEX, 1, color);
        }

        //cv::imshow("YOLOv7 Output", image);
    }

    void ObjTrYoloOnnx::receive(const MsgPtr &msg) {

        if (!msg) {
            DLOG(WARNING) << "ObjTrYoloOnnx::receive, null message detected\n";
            return;
        }

        if (dynamic_pointer_cast<MsgSensorData>(msg)) {

            auto msgSensor = dynamic_pointer_cast<MsgSensorData>(msg);
            auto sensorData = msgSensor->getData();
            if (sensorData && dynamic_pointer_cast<ImageTs>(sensorData)) {

                auto pImage = dynamic_pointer_cast<ImageTs>(sensorData);
                if (pImage && !pImage->mImage.empty()) {
                    mMtxImgBuff.lock();
                    mqpImages.push(pImage);
                    mMtxImgBuff.unlock();
                }
            }
        }

        if (dynamic_pointer_cast<MsgConfig>(msg)) {
            this->setup(msg);
        }

        if (dynamic_pointer_cast<MsgRequest>(msg)) {
            this->handleRequest(msg);
        }

        int action = msg->getTargetId();
        if (action == FCN_OBJ_TR_STOP) {
            this->stop();
        }
    }

    cv::Point2f ObjTrYoloOnnx::find_center(const Result &d, const cv::Size& imgSize) {

        auto w = imgSize.width, h = imgSize.height;
        auto dx = static_cast<float>(d.x * w);
        auto dy = static_cast<float>(d.y * h);
        auto dw = static_cast<float>(d.w * w);
        auto dh = static_cast<float>(d.h * h);

        return {dx + dw / 2.f, dy + dh / 2.f};
    }

    void ObjTrYoloOnnx::setup(const MsgPtr &msg) {

        auto configMsg = dynamic_pointer_cast<MsgConfig>(msg);
        if (configMsg) {
            auto pParam = configMsg->getConfig();
            if (!pParam) {
                DLOG(WARNING) << "ObjTrYoloOnnx::initialize, parameter is null\n";
                return;
            }

            if (dynamic_pointer_cast<ParamType<string>>(pParam)) {
                // this is model base path in sequence from data provider
                mModelBase = dynamic_pointer_cast<ParamType<string>>(pParam)->getValue();
            }
            else {
                // this is from parameter server
                auto pNameParam = find_param<ParamType<string>>("name", pParam);
                mName = (pNameParam) ? pNameParam->getValue() : "no_name";

                auto pIcParam = pParam->read("interface");
                if (pIcParam) {
                    auto pParamType = find_param<ParamType<string>>("type", pIcParam);
                    string type = (pParamType) ? pParamType->getValue() : "no_type";
                    auto pParamTarget = find_param<ParamType<string>>("target", pIcParam);
                    string target = (pParamTarget) ? pParamTarget->getValue() : "no_target";
                    auto pParamPort = find_param<ParamType<int>>("port", pIcParam);
                    int port = (pParamPort) ? pParamPort->getValue() : 0;
                    mpInterface = make_shared<SensorInterface>(SensorInterface::InterfaceType::OFFLINE, target, port);
                }

                auto pModelFile = find_param<ParamType<string>>("onnx_model", pParam);
                mModelName = (pModelFile) ? pModelFile->getValue() : "";
            }

            if (!mModelBase.empty() && !mModelName.empty()) {
                mModelPath = mModelBase + "/" + mModelName;

                impl = std::make_shared<Impl>(mModelPath, mCudaDevice);

                DL_INIT_PARAM params;
                params.rectConfidenceThreshold = 0.1;
                params.iouThreshold = 0.5;
                params.modelPath = mModelPath;
                params.imgSize = { 640, 640 };
#ifdef USE_CUDA
                params.cudaEnable = true;

    // GPU FP32 inference
    params.modelType = YOLO_DETECT_V8;
    // GPU FP16 inference
    //Note: change fp16 onnx model
    //params.modelType = YOLO_DETECT_V8_HALF;

#else
                // CPU inference
                params.modelType = YOLO_DETECT_V8;
                params.cudaEnable = false;

#endif
                this->CreateSession(params);
            }
        }
    }

    string ObjTrYoloOnnx::CreateSession(DL_INIT_PARAM& iParams) {

        //char* Ret = RET_OK;
        string Ret;
        std::regex pattern("[\u4e00-\u9fa5]");
        bool result = std::regex_search(iParams.modelPath, pattern);
        if (result)
        {
            Ret = "[YOLO_V8]:Your model path is error.Change your model path without chinese characters.";
            std::cout << Ret << std::endl;
            return Ret;
        }
        try
        {
            rectConfidenceThreshold = iParams.rectConfidenceThreshold;
            iouThreshold = iParams.iouThreshold;
            imgSize = iParams.imgSize;
            modelType = iParams.modelType;
            env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "Yolo");
            Ort::SessionOptions sessionOption;
            if (iParams.cudaEnable)
            {
                cudaEnable = iParams.cudaEnable;
                OrtCUDAProviderOptions cudaOption;
                cudaOption.device_id = 0;
                sessionOption.AppendExecutionProvider_CUDA(cudaOption);
            }
            sessionOption.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
            sessionOption.SetIntraOpNumThreads(iParams.intraOpNumThreads);
            sessionOption.SetLogSeverityLevel(iParams.logSeverityLevel);

#ifdef _WIN32
            int ModelPathSize = MultiByteToWideChar(CP_UTF8, 0, iParams.modelPath.c_str(), static_cast<int>(iParams.modelPath.length()), nullptr, 0);
        wchar_t* wide_cstr = new wchar_t[ModelPathSize + 1];
        MultiByteToWideChar(CP_UTF8, 0, iParams.modelPath.c_str(), static_cast<int>(iParams.modelPath.length()), wide_cstr, ModelPathSize);
        wide_cstr[ModelPathSize] = L'\0';
        const wchar_t* modelPath = wide_cstr;
#else
            const char* modelPath = iParams.modelPath.c_str();
#endif // _WIN32

            session = make_unique<Ort::Session>(env, modelPath, sessionOption);
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
            return Ret;
        }
        catch (const std::exception& e)
        {
            const char* str1 = "[YOLO_V8]:";
            const char* str2 = e.what();
            std::string resultStr = std::string(str1) + std::string(str2);
            char* merged = new char[resultStr.length() + 1];
            std::strcpy(merged, resultStr.c_str());
            std::cout << merged << std::endl;
            delete[] merged;
            return "[YOLO_V8]:Create session failed.";
        }

    }

    void ObjTrYoloOnnx::run() {

        while(true) {
            mMtxImgBuff.lock();
            ImagePtr pImage;
            if (!mqpImages.empty()) {
                pImage = mqpImages.front();
                mqpImages.pop();
            }
            mMtxImgBuff.unlock();

            this->process(pImage);

            mMtxStop.lock();
            bool bStop = mbStop;
            mMtxStop.unlock();
            if (bStop) {
                break;
            }
        }
    }

    void ObjTrYoloOnnx::stop() {
        MsgCallback::stop();
    }

    void ObjTrYoloOnnx::process(const ImagePtr &pImage) {

        if (!pImage || pImage->mImage.empty()) {
            DLOG(WARNING) << "ObjTrYoloOnnx::process, empty image detected\n";
            return;
        }

        if (dynamic_pointer_cast<ImageTs>(pImage)) {
            double currTs = dynamic_pointer_cast<ImageTs>(pImage)->mTimeStamp;
            if (mLastTs >= 0) {
                if (mLastTs >= currTs) {
                    DLOG(WARNING) << "ObjTrYoloOnnx::process, timestamp error: " << mLastTs << " >= " << currTs << "\n";
                }
            }
            mLastTs = currTs;
        }
        auto image = pImage->mImage.clone();
        assert(!image.empty() && image.channels() == 3);
        int image_size = static_cast<int>(this->image_size());
        //if (image_size < 0) image_size = 640;
        cv::resize(image, image, {image_size, image_size});

        auto [array, shape] = convert_image(image);
        auto detections = this->detect(array.data(), shape);

        if (detections.empty()) {
            DLOG(WARNING) << "ObjTrYoloOnnx::detect, detections is empty\n";
            return;
        }

        display_image(image, detections[0]);

        auto w = image.cols, h = image.rows;
        cv::Size imgSizeCv(w, h);
        for (const auto &d : detections[0]) {

            //cv::Point2f ptObs = find_center(d, imgSizeCv);
            //auto pMsgPtObs = make_shared<MsgType<cv::Point2f>>(FE::FrontEnd::TOPIC, ptObs);
            cv::Rect2f detRect(d.x * w, d.y * h, d.w * w, d.h * h);
            auto pMsgPtObs = make_shared<MsgType<cv::Rect2f>>(FE::FrontEnd::TOPIC, detRect);

            mpChannel->publish(pMsgPtObs);
        }
    }

    void ObjTrYoloOnnx::handleRequest(const MsgPtr &msg) {
        ObjTracking::handleRequest(msg);

        if (msg && dynamic_pointer_cast<MsgRequest>(msg)) {

            auto pReqMsg = dynamic_pointer_cast<MsgRequest>(msg);
            auto sender = pReqMsg->getCallback();
            if (sender) {
                int action = msg->getTargetId();
                if (action == FCN_OBJ_TR_RUN) {
                    auto pThRun = make_shared<thread>(&ObjTrYoloOnnx::run, this);
                    auto msgRes = make_shared<MsgType<shared_ptr<thread>>>(msg->getTopic(), pThRun);
                    sender->receive(msgRes);
                }
            }
        }
    }

} // NAV24::OP