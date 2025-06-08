//
// Created by masoud on 2/24/25.
//

#include "OP_MapPointManager.hpp"
#include "Point3D.hpp"
#include "DataConversion.hpp"

using namespace std;


namespace NAV24::OP {

    MapPointManager::MapPointManager(const ChannelPtr &pChannel) : Operator(pChannel) {

        mpFtMatcher = make_shared<FtAssocOrbSlam>(mpChannel);
        mpChannel->registerChannel(ID_CH_OP, mpFtMatcher);

        // request calibration params
        auto fp = [this](auto && PH1) { receive(std::forward<decltype(PH1)>(PH1)); };
        auto msgGetCalib = make_shared<MsgRequest>(ID_CH_PARAMS, fp, Operator::TOPIC);
        mpChannel->send(msgGetCalib);
    }

    bool MapPointManager::triangulate(Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2, Eigen::Matrix<float,3,4> &Tc1w,
                                      Eigen::Matrix<float,3,4> &Tc2w , Eigen::Vector3f &x3D) {

        Eigen::Matrix4f A;
        A.block<1,4>(0,0) = x_c1(0) * Tc1w.block<1,4>(2,0) - Tc1w.block<1,4>(0,0);
        A.block<1,4>(1,0) = x_c1(1) * Tc1w.block<1,4>(2,0) - Tc1w.block<1,4>(1,0);
        A.block<1,4>(2,0) = x_c2(0) * Tc2w.block<1,4>(2,0) - Tc2w.block<1,4>(0,0);
        A.block<1,4>(3,0) = x_c2(1) * Tc2w.block<1,4>(2,0) - Tc2w.block<1,4>(1,0);

        Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);

        Eigen::Vector4f x3Dh = svd.matrixV().col(3);

        if(x3Dh(3)==0)
            return false;

        // Euclidean coordinates
        x3D = x3Dh.head(3)/x3Dh(3);

        return true;
    }

    void MapPointManager::createMapPoints(const FramePtr &pKF, vector<WO::WoPtr>& vpPoints3d) {
        // This is based on ORB-SLAM's LocalMapping::CreateNewMapPoints()

        // Retrieve neighbor keyframes in covisibility graph
        vector<FramePtr> vpCovisKFs;
        findBestCovisibility(pKF, vpCovisKFs);
        if (vpCovisKFs.empty()) {
            // no covisible key frames found
            return;
        }

        // Retrieve current frame's intrinsic/extrinsic params
        Eigen::Matrix4d T_wc1 = pKF->getPose()->getPose();
        Eigen::Matrix4d T_cw1 = T_wc1.inverse();
        Eigen::Matrix<float,3,4> eigTcw1 = T_wc1.block<3,4>(0,0).cast<float>();
        Eigen::Matrix<float,3,3> Rcw1 = eigTcw1.block<3,3>(0,0);
        Eigen::Matrix<float,3,3> Rwc1 = Rcw1.transpose();
        Eigen::Vector3f tcw1 = T_cw1.block<3,1>(0,3).cast<float>();
        Eigen::Vector3f Ow1 = T_wc1.block<3,1>(0,3).cast<float>();

        vector<float> intrinsics1 = mpCamCalib->getIntrinsicsVector();
        const float &fx1 = intrinsics1[0];
        const float &fy1 = intrinsics1[1];
        const float &cx1 = intrinsics1[2];
        const float &cy1 = intrinsics1[3];
        const float &invfx1 = intrinsics1[4];
        const float &invfy1 = intrinsics1[5];

        auto vpObs1 = pKF->getObservations();

        const float ratioFactor = 1.5f*1.f;//mpCurrentKeyFrame->mfScaleFactor;

        // Loop through found key frames:
        for (const auto& pKF2 : vpCovisKFs) {

            // Retrieve second frame's intrinsic/extrinsic params
            Eigen::Matrix4d T_wc2 = pKF2->getPose()->getPose();
            Eigen::Matrix4d T_cw2 = T_wc1.inverse();
            Eigen::Vector3f Ow2 = T_wc2.block<3,1>(0,3).cast<float>();
            Eigen::Matrix<float,3,4> eigTcw2 = T_cw2.block<3,4>(0,0).cast<float>();
            Eigen::Matrix<float,3,3> Rcw2 = eigTcw2.block<3,3>(0,0);
            Eigen::Matrix<float,3,3> Rwc2 = Rcw2.transpose();
            Eigen::Vector3f tcw2 = T_cw2.block<3,1>(0,3).cast<float>();

            // assuming all camera frames use the same calibration
            // todo: refine this to include general cases
            const float &fx2 = fx1;
            const float &fy2 = fy1;
            const float &cx2 = cx1;
            const float &cy2 = cy1;
            const float &invfx2 = invfx1;
            const float &invfy2 = invfy1;

            // Check based line for close frames
            Eigen::Vector3f vBaseline = Ow2-Ow1;
            const float baseline = vBaseline.norm();

            // Check ratio of baseline to median scene depth in monocular case
            const float medianDepthKF2 = computeMedianDepth(pKF2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;
            if(ratioBaselineDepth<0.01)
                continue;

            // Find new matches (descriptor-based in ORB-SLAM)
            vector<pair<size_t,size_t>> vMatchedIndices;
//            bool bCoarse = mbInertial && mpTracker->mState==Tracking::RECENTLY_LOST && mpCurrentKeyFrame->GetMap()->GetIniertialBA2();
            bool bCoarse = false;
            mpFtMatcher->searchForTriangulation(pKF, pKF2, mpCamCalib, vMatchedIndices, false, bCoarse);

            // Loop through found matches:
            const int nmatches = vMatchedIndices.size();
            auto vpObs2 = pKF2->getObservations();
            for (const auto& mchIdx : vMatchedIndices) {

                const int &idx1 = mchIdx.first;
                const int &idx2 = mchIdx.second;

                const auto &kp1 = dynamic_pointer_cast<OB::Point2D>(vpObs1[idx1]);
                const auto &kp2 = dynamic_pointer_cast<OB::Point2D>(vpObs2[idx2]);

                // Check parallax between rays
                cv::Point3f cv_xn1 = dynamic_pointer_cast<WO::Point3D>(mpCamCalib->unproject({kp1}))->getPoint();
                Eigen::Vector3f xn1 = Converter::toVector3d(cv_xn1).cast<float>();
                cv::Point3f cv_xn2 = dynamic_pointer_cast<WO::Point3D>(mpCamCalib->unproject({kp2}))->getPoint();
                Eigen::Vector3f xn2 = Converter::toVector3d(cv_xn2).cast<float>();

                Eigen::Vector3f ray1 = Rwc1 * xn1;
                Eigen::Vector3f ray2 = Rwc2 * xn2;
                const float cosParallaxRays = ray1.dot(ray2)/(ray1.norm() * ray2.norm());

                Eigen::Vector3f x3D;
                bool goodProj = triangulate(xn1, xn2, eigTcw1, eigTcw2, x3D);

                if(!goodProj)
                    continue;

                // Check triangulation in front of cameras
                float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
                if(z1<=0)
                    continue;

                float z2 = Rcw2.row(2).dot(x3D) + tcw2(2);
                if(z2<=0)
                    continue;

                // Check reprojection error in first keyframe
                const float &sigmaSquare1 = 1.f;//mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
                const float x1 = Rcw1.row(0).dot(x3D)+tcw1(0);
                const float y1 = Rcw1.row(1).dot(x3D)+tcw1(1);
                const float invz1 = 1.0/z1;

                // monocular case:
                auto pWO1 = make_shared<WO::Point3D>(x1, y1, z1);
                auto pUV1 = mpCamCalib->project(pWO1);
                cv::Point2f uv1 = dynamic_pointer_cast<OB::Point2D>(pUV1)->getPointUd();
                float errX1 = uv1.x - kp1->getPointUd().x;
                float errY1 = uv1.y - kp1->getPointUd().y;

                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;

                // Check reprojection error in second keyframe
                const float sigmaSquare2 = 1.f; //pKF2->mvLevelSigma2[kp2.octave];
                const float x2 = Rcw2.row(0).dot(x3D)+tcw2(0);
                const float y2 = Rcw2.row(1).dot(x3D)+tcw2(1);
                const float invz2 = 1.0/z2;

                auto pWO2 = make_shared<WO::Point3D>(x2, y2, z2);
                auto pUV2 = mpCamCalib->project(pWO2);
                cv::Point2f uv2 = dynamic_pointer_cast<OB::Point2D>(pUV2)->getPointUd();
                float errX2 = uv2.x - kp2->getPointUd().x;
                float errY2 = uv2.y - kp2->getPointUd().y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;

                // todo: unify project/unproject/etc...

                // Check scale consistency
                Eigen::Vector3f normal1 = x3D - Ow1;
                float dist1 = normal1.norm();

                Eigen::Vector3f normal2 = x3D - Ow2;
                float dist2 = normal2.norm();

                if(dist1==0 || dist2==0)
                    continue;

//                if(mbFarPoints && (dist1>=mThFarPoints||dist2>=mThFarPoints)) // MODIFICATION
//                    continue;

                const float ratioDist = dist2/dist1;
//                const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

//                if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
//                    continue;

                // Triangulation is successful
                auto pMP = make_shared<WO::Point3D>(x3D);

                pMP->addObservation(kp1);
                pMP->addObservation(kp2);

                // todo: also set frames for fundamental variables (MP)

                kp1->setWorldObject(pMP);
                kp2->setWorldObject(pMP);

//                pMP->ComputeDistinctiveDescriptors();
//                pMP->UpdateNormalAndDepth();

//                mpAtlas->AddMapPoint(pMP);
                vpPoints3d.push_back(pMP);
//                mlpRecentAddedMapPoints.push_back(pMP);
            }
        }

        // ORB-SLAM has lots of options for other camera configurations (Stereo)
    }

    void MapPointManager::mapPointCulling(const std::vector<WO::WoPtr>& vpWorldObjs, const int nTotalKFs) {

        // Loop through recently added map points
        for (const auto& pMp : vpWorldObjs) {

            // Remove bad map points heuristically:
            bool c1 = true, c2 = true, c3 = true;

            if (nTotalKFs > 2) {
                // Number of observations
                size_t nObs = pMp->getNumObs();
                c1 = nObs < OP_MPM_DEF_TH_N_OBS;

                // Number of tracked (key) frames
                size_t nKF = pMp->getNumKFs();
                c2 = nKF < OP_MPM_DEF_TH_N_KF;
            }

            // Invalid optimization (large errors, ...)
            c3 = pMp->isValid();

            if (!c1 || !c2 || !c3) {
                // todo: remove the world object
            }
        }
    }

    void MapPointManager::checkNewKeyFrame(const FramePtr &pKF, std::vector<WO::WoPtr>& vpPoints3d) {

        this->createMapPoints(pKF, vpPoints3d);

//        this->mapPointCulling();
    }

    void MapPointManager::findBestCovisibility(const FramePtr &pKF, std::vector<FramePtr> &vpCovisKFs) {

        // for now, just return adjacent key frames
        FramePtr pPreFrame = pKF->getPrevFrame();
        while (pPreFrame) {
            if (pPreFrame->getPose()->getLevel() > 0) {
                vpCovisKFs.push_back(pPreFrame);
                break;
            }
            pPreFrame = pPreFrame->getPrevFrame();
        }
    }

    void MapPointManager::receive(const MsgPtr &msg) {
        Operator::receive(msg);

        if (msg) {
            if (dynamic_pointer_cast<MsgType<CalibPtr>>(msg)) {
                mpCamCalib = dynamic_pointer_cast<MsgType<CalibPtr>>(msg)->getData();
            }
        }
    }

    float MapPointManager::computeMedianDepth(const FramePtr &pFrame) {

        if (pFrame) {
            auto vpAllObservations = pFrame->getObservations();
            vector<float> vDepth;
            vDepth.reserve(vpAllObservations.size());
            for (const auto& pObs : vpAllObservations) {
                if (pObs) {
                    auto pWO = pObs->getWorldObject();
                    if (pWO && dynamic_pointer_cast<WO::Point3D>(pWO)) {
                        auto p3d = dynamic_pointer_cast<WO::Point3D>(pWO)->getPoint();
                        vDepth.push_back(cv::norm(p3d));
                    }
                }
            }
            if (vDepth.size() > 0) {
                return vDepth[static_cast<size_t>(vDepth.size() / 2)];
            }
        }
        return 1.f;
    }

} // NAV24::OP