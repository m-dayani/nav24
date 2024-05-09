//
// Created by masoud on 5/2/24.
//

#ifndef NAV24_OP_OBJTRACKINGYOLO_HPP
#define NAV24_OP_OBJTRACKINGYOLO_HPP

#include <queue>
#include <onnxruntime_cxx_api.h>

#include "Image.hpp"
#include "OP_ObjTracking.hpp"


namespace NAV24::OP {

    enum MODEL_TYPE
    {
        //FLOAT32 MODEL
        YOLO_DETECT_V8 = 1,
        YOLO_POSE = 2,
        YOLO_CLS = 3,

        //FLOAT16 MODEL
        YOLO_DETECT_V8_HALF = 4,
        YOLO_POSE_V8_HALF = 5,
        YOLO_CLS_HALF = 6
    };


    typedef struct DL_INIT_PARAM
    {
        std::string modelPath;
        MODEL_TYPE modelType = YOLO_DETECT_V8;
        std::vector<int> imgSize = { 640, 640 };
        float rectConfidenceThreshold = 0.6;
        float iouThreshold = 0.5;
        int	keyPointsNum = 2;//Note:kpt number for pose
        bool cudaEnable = false;
        int logSeverityLevel = 3;
        int intraOpNumThreads = 1;
    } dlInitParam;


    typedef struct DL_RESULT
    {
        int classId;
        float confidence;
        cv::Rect box;
        std::vector<cv::Point2f> keyPoints;
    } dlResult;

    /// This class represents a detection instance
    struct Result
    {
        /// Starting x point of the bounding box
        double x;

        /// Starting y point of the bounding box
        double y;

        /// Width of the bounding box
        double w;

        /// Height of the bounding box
        double h;

        /// Detected object class index
        int64_t index;

        /// Detected object class name
        std::string name;

        /// Confidence of object detection
        double confidence;
    };

    struct Impl;

    /// Shape used by the library
    using Shape = std::vector<int64_t>;

    /// List of detected instances in an image
    using Results = std::vector<Result>;

    using Array = std::vector<float>;

    /// The YOLO model class that does the detection and tracking
    class ObjTrYoloOnnx : public ObjTracking {
    public:
        inline static const std::string TOPIC = "OP::ObjTrYoloOnnx";

        explicit ObjTrYoloOnnx(const ChannelPtr&  pChannel);
        void receive(const MsgPtr &msg) override;

        //static ParamPtr getDefParams(const std::string& model_base, const std::string& model_file);

    protected:
        void setup(const MsgPtr& msg) override;
        void handleRequest(const MsgPtr &reqMsg) override;
        void run() override;
        void stop() override;

        void process(const ImagePtr& pImage);

        /// Get input image size to the model
        /// @return input image size of the model
        [[nodiscard]] int64_t image_size() const;

        /// Run the input through the model and return the detections
        /// @param data pointer to the start of image data in the form of continuous CHW or NCHW
        /// @param shape shape of the input image, can be CHW or NCHW
        /// @return list of list of detection instances, the size of top level vector is N
        std::vector<Results> detect(float *data, Shape shape);

        static std::pair<Array, Shape> convert_image(const cv::Mat &image);

        static cv::Point2f find_center(const Result& d, const cv::Size& imgSize);

        std::string CreateSession(DL_INIT_PARAM& iParams);

    protected:
        std::string mName;

        std::string mModelName;
        std::string mModelBase;
        std::string mModelPath;

        std::shared_ptr<SensorInterface> mpInterface;

        int mCudaDevice;
        std::shared_ptr<Impl> impl;

    private:
        Ort::Env env;
        std::unique_ptr<Ort::Session> session;
        bool cudaEnable{};
        Ort::RunOptions options;
        std::vector<const char*> inputNodeNames;
        std::vector<const char*> outputNodeNames;

        MODEL_TYPE modelType;
        std::vector<int> imgSize;
        float rectConfidenceThreshold{};
        float iouThreshold{};
        float resizeScales{};//letterbox scale

        std::queue<ImagePtr> mqpImages;
        std::mutex mMtxImgBuff;

        double mLastTs;
    };
} // NAV24::OP

#endif //NAV24_OP_OBJTRACKINGYOLO_HPP
