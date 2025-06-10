//
// Created by masoud on 2/6/24.
//

#ifndef NAV24_CALIBRATION_HPP
#define NAV24_CALIBRATION_HPP

#include <vector>
#include <memory>
#include <utility>
#include <cassert>
#include <vector>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/assume_abstract.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include <Eigen/Eigen>
//#include <eigen3/Eigen/Dense>
//#include <eigen3/Eigen/Geometry>

#include "Parameter.hpp"
#include "WorldObject.hpp"
#include "GeometricCamera.h"
#include "Pose.hpp"
#include "Message.hpp"


namespace NAV24 {

#define DEF_EC_DIST_COEF 3.84f
#define KB8_DEF_PRECISION 1e-6
//#define KB8_DEF_TH_EPC 0.0001f
//#define KB8_DEF_MIN_PLX 0.9998
//#define KB8_DEF_CHISQ_COEF 5.991

    class Calibration : public MsgCallback {
    public:
        enum DistType {
            GENERIC,
            PINHOLE,
            PINHOLE_RAD_TAN,
            KANNALA_BRANDT_8
        };

        static std::shared_ptr<Calibration> getNewInstance(const ChannelPtr& pChannel, const ParamPtr& pParam);

        explicit Calibration(const ChannelPtr &pChannel);

        void receive(const MsgPtr &msg) override;

        static bool wo2vec(const WO::WoPtr& p3d, Eigen::Vector3f& p3d_eig);
        static bool obs2vec(const OB::ObsPtr& p2d, Eigen::Vector2f& p2d_eig);
        static void vec2wo(const Eigen::Vector3f& p3d_eig, WO::WoPtr& p3d);
        static void vec2obs(const Eigen::Vector2f& p2d_eig, OB::ObsPtr& p2d);

        [[nodiscard]] virtual OB::ObsPtr project(const WO::WoPtr& Pc) const = 0;
        [[nodiscard]] virtual WO::WoPtr unproject(const OB::ObsPtr& pt2d) const = 0;

        [[nodiscard]] virtual Eigen::Matrix<double, 2, 3> projectJac(const WO::WoPtr& pt3d) const = 0;
        [[nodiscard]] virtual cv::Mat unprojectJac(const OB::ObsPtr& pt2d) const = 0;

        [[nodiscard]] virtual float uncertainty2(const Eigen::Matrix<double,2,1> &p2D) const = 0;

        [[nodiscard]] virtual bool epipolarConstrain(const std::shared_ptr<const Calibration> &pCalib2,
                                       const OB::ObsPtr& kp1, const OB::ObsPtr& kp2,
                                       const PosePtr& pPose_12, float sigmaLevel, float unc) const = 0;

        virtual void generateUndistMaps() = 0;
        virtual void undistort(const std::vector<OB::ObsPtr> &vpObsDist, std::vector<OB::ObsPtr>& vpObs) const = 0;
        virtual void distort(const std::vector<OB::ObsPtr> &pObs, std::vector<OB::ObsPtr>& vpObsDist) const = 0;
        virtual void undistortMaps(const std::vector<OB::ObsPtr> &srcPts, std::vector<OB::ObsPtr> &dstPts) const = 0;
        virtual void undistImageMaps(const cv::Mat& srcImage, cv::Mat& dstImage) const = 0;

        [[nodiscard]] unsigned int getId() const { return mnId; }

        // don't expose internal parameters: use methods instead

        static ParamPtr getCalibParams(const cv::Mat& K, const cv::Mat& D, const std::string& distType,
                                       std::vector<ParamPtr>& vpParamHolder);

        [[nodiscard]] std::vector<float> computeImageBounds(const cv::Mat &image) const;

        static bool isInImage(float x, float y, const cv::Scalar& imageSize);
        static bool isInImage(float x, float y, int imWidth, int imHeight);
        [[nodiscard]] bool isInImage(float x, float y) const;

        [[nodiscard]] cv::Size getImageSize() const { return mImSize; }

        // you can find the type of camera by dynamic pointer casting

        [[nodiscard]] virtual std::string printStr(const std::string& prefix) const;

    protected:
        void setup(const MsgPtr &configMsg) override;
        void run() override;
        void handleRequest(const MsgPtr &reqMsg) override;

    protected:
        // Don't mix pose (specific to each frame) with general calib params

        unsigned int mnId;

        DistType mDistType;

        int mImWidth, mImHeight;
        cv::Size mImSize;

    public:
        static long unsigned int nNextId;
    };
    typedef std::shared_ptr<Calibration> CalibPtr;
    typedef std::shared_ptr<const Calibration> CalibPtrRO;

    /* ============================================================================================================== */

    class CalibPinhole : public Calibration {
    public:
        explicit CalibPinhole(const ChannelPtr& pChannel);

        [[nodiscard]] OB::ObsPtr project(const WO::WoPtr &Pc) const override;
        [[nodiscard]] WO::WoPtr unproject(const OB::ObsPtr &pt2d) const override;

        [[nodiscard]] Eigen::Matrix<double, 2, 3> projectJac(const WO::WoPtr &pt3d) const override;
        [[nodiscard]] cv::Mat unprojectJac(const OB::ObsPtr &pt2d) const override;

        [[nodiscard]] bool epipolarConstrain(const CalibPtrRO &pCalib2, const OB::ObsPtr& kp1, const OB::ObsPtr& kp2,
                               const PosePtr& pPose_12, float sigmaLevel, float unc) const override;

        [[nodiscard]] float uncertainty2(const Eigen::Matrix<double,2,1> &p2D) const override;

        void generateUndistMaps() override;
        void undistort(const std::vector<OB::ObsPtr> &vpObsDist, std::vector<OB::ObsPtr>& vpObs) const override;
        void distort(const std::vector<OB::ObsPtr> &pObs, std::vector<OB::ObsPtr>& vpObsDist) const override;
        void undistortMaps(const std::vector<OB::ObsPtr> &srcPts, std::vector<OB::ObsPtr> &dstPts) const override;
        void undistImageMaps(const cv::Mat &srcImage, cv::Mat &dstImage) const override;

        [[nodiscard]] std::string printStr(const std::string &prefix) const override;
        friend std::ostream& operator<<(std::ostream& os, const CalibPinhole& ph);
        friend std::istream& operator>>(std::istream& os, CalibPinhole& ph);

        [[nodiscard]] cv::Mat getK_cv() const { return mK_cv.clone(); }

    protected:
        void setup(const MsgPtr &configMsg) override;

    private:
        static cv::Mat skewSymmetricMatrix(const cv::Mat &v);

    protected:
        //Parameters vector corresponds to: [fx, fy, cx, cy]
        float fx, fy, cx, cy, fx_1, fy_1;
        cv::Mat mK_cv;
        Eigen::Matrix3f mK_ei;
    };

    /* ============================================================================================================== */

    class CalibPinholeRadTan : public CalibPinhole {
    public:
        explicit CalibPinholeRadTan(const ChannelPtr& pChannel);

        void generateUndistMaps() override;

        void undistort(const std::vector<OB::ObsPtr> &vpObsDist, std::vector<OB::ObsPtr> &vpObs) const override;

        void distort(const std::vector<OB::ObsPtr> &pObs, std::vector<OB::ObsPtr> &vpObsDist) const override;

        void undistortMaps(const std::vector<OB::ObsPtr> &srcPts, std::vector<OB::ObsPtr> &dstPts) const override;

        void undistImageMaps(const cv::Mat &srcImage, cv::Mat &dstImage) const override;

        [[nodiscard]] std::string printStr(const std::string &prefix) const override;

    protected:
        void setup(const MsgPtr &configMsg) override;

    protected:
        cv::Mat mD_cv;
        cv::Mat mUndistMapX, mUndistMapY;
        cv::Mat mNewCamMatrix;

        // Rectification Matrix
        cv::Mat mR;

        // Projection Matrix
        cv::Mat mP;
    };

    /* ============================================================================================================== */

    // Kannala Brandt 8 model
    class CalibFisheye final : public CalibPinholeRadTan {
    public:
        explicit CalibFisheye(const ChannelPtr& pChannel, float precision_ = KB8_DEF_PRECISION);

        [[nodiscard]] OB::ObsPtr project(const WO::WoPtr &Pc) const override;

        [[nodiscard]] WO::WoPtr unproject(const OB::ObsPtr &pt2d) const override;

        [[nodiscard]] Eigen::Matrix<double, 2, 3> projectJac(const WO::WoPtr &pt3d) const override;

//        [[nodiscard]] cv::Mat unprojectJac(const OB::ObsPtr &pt2d) const override;

        [[nodiscard]] bool epipolarConstrain(const CalibPtrRO &pCalib2, const OB::ObsPtr &kp1, const OB::ObsPtr &kp2,
                               const PosePtr &pPose_12, float sigmaLevel, float unc) const override;

        [[nodiscard]] float uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D) const override;

        void generateUndistMaps() override;

        void undistort(const std::vector<OB::ObsPtr> &vpObsDist, std::vector<OB::ObsPtr> &vpObs) const override;

        [[nodiscard]] std::string printStr(const std::string &prefix) const override;

//        std::vector<cv::KeyPoint> UndistortKeyPoints(const std::vector<cv::KeyPoint>& vKPts) override;

        friend std::ostream& operator<<(std::ostream& os, const CalibFisheye& kb);
        friend std::istream& operator>>(std::istream& is, CalibFisheye& kb);

        std::vector<int> mvLappingArea;
    private:
        const float precision;

        //Parameters vector corresponds to
        //[fx, fy, cx, cy, k0, k1, k2, k3]
    };

}   //NAV24

#endif //NAV24_CALIBRATION_HPP
