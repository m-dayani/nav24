//
// Created by masoud on 2/24/25.
//

#ifndef NAV24_OP_OBJDETONNXRT_HPP
#define NAV24_OP_OBJDETONNXRT_HPP

#include <queue>
#ifdef LIB_ONNX_RUNTIME_FOUND
#include <onnxruntime/onnxruntime_cxx_api.h>
#endif

#include "OP_ObjDetMl.hpp"


namespace NAV24::OP {

    using Shape = std::vector<int64_t>;
    using Array = std::vector<float>;


    class ObjDetOnnxRT : public ObjDet {
    public:
        inline static const std::string TOPIC = "OP::ObjDetOnnxRT";

        ObjDetOnnxRT(const std::string& pathModel, const std::string& pathLabels, ModelInfo  modelInfo);

//        explicit ObjDetOnnxRT(const ChannelPtr&  pChannel);
//        void receive(const MsgPtr &msg) override;
        //static ParamPtr getDefParams(const std::string& model_base, const std::string& model_file);

    protected:
//        void setup(const MsgPtr& msg) override;
//        void handleRequest(const MsgPtr &reqMsg) override;
        //void run() override;
        //void stop() override;

//        [[nodiscard]] int64_t image_size() const;

        void readLabels(const std::string &pathLabels);
        void preProcess(const ImagePtr& pImage, Array& blob);

    public:
        void detect(const ImagePtr &pImage, std::vector<OB::ObsPtr> &vpObs) override;

    protected:
//        std::vector<Results> detect(float *data, Shape shape);

//        static std::pair<Array, Shape> convert_image(const cv::Mat &image);

        //static cv::Point2f find_center(const Result& d, const cv::Size& imgSize);

//        std::string CreateSession(DL_INIT_PARAM& iParams);

    private:
        std::string mPathModel;
        std::string mPathLabels;
        std::vector<std::string> mLabels;

//        std::shared_ptr<SensorInterface> mpInterface;

        ModelInfo mModelInfo;
        int mCudaDevice = 0;
        bool cudaEnable{};

//        std::shared_ptr<Impl> impl;
#ifdef LIB_ONNX_RUNTIME_FOUND
        Ort::Env env;
        std::unique_ptr<Ort::Session> session;
        Ort::RunOptions options;
#endif

        std::vector<const char*> inputNodeNames;
        std::vector<const char*> outputNodeNames;
        Shape mInputShape;
    };

} // NAV24::OP

#endif //NAV24_OP_OBJDETONNXRT_HPP
