//
// Created by masoud on 9/3/24.
//

#include <complex>
//#include <mutex>
#include <glog/logging.h>
#include <Eigen/StdVector>
//#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "../thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "../thirdparty/g2o/g2o/core/block_solver.h"
#include "../thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
//#include "../thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "../thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "../thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "../thirdparty/g2o/g2o/core/robust_kernel_impl.h"
//#include "../thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

#include "BE_GraphOptim.hpp"
//#include "G2oTypes.h"
#include "DataConversion.hpp"
#include "Point3D.hpp"
#include "OptimizableTypes.hpp"

using namespace std;

namespace NAV24::BE {

    void GraphOptim::solve(const ProblemPtr &problem) {

        if (!static_pointer_cast<PR_VBA>(problem)) {
            return;
        }
        const int nIterations = problem->getNumIter();
        auto pProblem = static_pointer_cast<PR_VBA>(problem);
        auto vpFrames = pProblem->mvpFrames;

        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

        auto* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        auto* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        // Set KeyFrame vertices
        TaggedVertexPose poseMap{};
        addFrameVertices(vpFrames, 0l, optimizer, poseMap);

        // Set MapPoint vertices
        TaggedVertexMP point3dMap{};
        addMapPointVertices(problem, poseMap.size(), optimizer, point3dMap);

        if (poseMap.empty() || point3dMap.empty()) {
            DLOG(WARNING) << "Optimization Problem is ill-conditioned: N_pose: "
                    << poseMap.size() << ", N_mp: " << point3dMap.size() << endl;
            return;
        }

        // Optimize!
        optimizer.setVerbose(false);
        optimizer.initializeOptimization();
        optimizer.optimize(nIterations);
//        Verbose::PrintMess("BA: End of the optimization", Verbose::VERBOSITY_NORMAL);

        // Recover optimized data
        //Keyframes
        recoverPose(poseMap);

        //Points
        recoverMapPoints(point3dMap);
    }

    void GraphOptim::addFrameVertices(const vector <FramePtr> &vpFrames, const unsigned long lastId,
                                      g2o::SparseOptimizer &optimizer, TaggedVertexPose &poseMap) {

        unsigned long idCnt = lastId;
        for(const auto& pFrame : vpFrames) {

            g2o::VertexSE3Expmap* vSE3 = new ORB_SLAM3::MyVertexSE3Expmap();
            auto pTcw = pFrame->getPose();
            if (!pTcw) {
                continue;
            }
            const unsigned long id = idCnt++;
            auto Tcw_ei = pTcw->getPose();
            vSE3->setEstimate(g2o::SE3Quat(Tcw_ei.block<3, 3>(0, 0),
                                           Tcw_ei.block<3, 1>(0, 3)));
            vSE3->setId((int)id);
            vSE3->setFixed(pFrame->isOptFixed());
            optimizer.addVertex(vSE3);
            pFrame->setOptId(id);
            poseMap.insert(make_pair(id, make_pair(vSE3, pFrame)));
        }
    }

    void GraphOptim::extractMapPoints(const vector <FramePtr> &vpFrames, set <WO::WoPtr> &spPoint3d) {

        for (const auto& pFrame : vpFrames) {
            auto vpObs = pFrame->getObservations();
            for (const auto& pObs : vpObs) {
                if (pObs) {
                    auto pMapPoint = pObs->getWorldObject();
                    if (pMapPoint) {
                        spPoint3d.insert(pMapPoint);
                    }
                }
            }
        }
    }

    void GraphOptim::addMapPointVertices(const ProblemPtr &problem, unsigned long lastId,
                                         g2o::SparseOptimizer &optimizer, TaggedVertexMP &point3dMap) {

        if (!static_pointer_cast<PR_VBA>(problem)) {
            return;
        }
        auto pProblem = static_pointer_cast<PR_VBA>(problem);

        const float thHuber2D = sqrt(5.99);
//        const float thHuber3D = sqrt(7.815);
        bool bRobust = pProblem->mbRobust;
        unsigned long idCnt = lastId;

        set<WO::WoPtr> spMapPoints;
        extractMapPoints(pProblem->mvpFrames, spMapPoints);

        vector<shared_ptr<WO::Point3D>> vpMapPoints;
        vpMapPoints.reserve(spMapPoints.size());
        for(const auto& pWO : spMapPoints) {

            if (!pWO || !dynamic_pointer_cast<WO::Point3D>(pWO)) {
                continue;
            }
            auto pPt3d = dynamic_pointer_cast<WO::Point3D>(pWO);
            vpMapPoints.push_back(pPt3d);

            if(!pPt3d->isValid())
                continue;

            g2o::VertexSBAPointXYZ* vPoint = new ORB_SLAM3::MyVertexSBAPointXYZ();
            vPoint->setEstimate(Converter::toVector3d(pPt3d->getPoint()));
            const unsigned long id = idCnt++;
            vPoint->setId(id);
            if (pWO->isOptFixed()) {
                vPoint->setFixed(true);
            }
            vPoint->setMarginalized(!pWO->isOptFixed());
            optimizer.addVertex(vPoint);

            auto vpObs = pWO->getObservations();

            //SET EDGES
            int nEdges = 0;
            for(const auto& pObs : vpObs) {

                auto pPt2d = pObs.lock();
                if(!pPt2d || !pPt2d->isValid() || !pPt2d->getFrame())
                    continue;

                const unsigned long frameId = pPt2d->getFrame()->getOptId();
                if(optimizer.vertex(id) == NULL || optimizer.vertex(frameId) == NULL)
                    continue;

                if (!dynamic_pointer_cast<OB::KeyPoint2D>(pPt2d)) {
                    continue;
                }
                auto pt2d = dynamic_pointer_cast<OB::KeyPoint2D>(pPt2d);

                nEdges++;

                cv::KeyPoint kpUn = pt2d->getKeyPoint();
                kpUn.pt = pt2d->getPointUd();

                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(frameId)));
                e->setMeasurement(obs);
                const float &invSigma2 = 1;//pPt2d->getUncertainty(); <- todo: implement
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                if (bRobust) {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->pCamera = pProblem->mpCalib;

                optimizer.addEdge(e);

                // todo: also keep track of edges??
            }

            if(nEdges==0) {
                optimizer.removeVertex(vPoint);
            }
            else {
                point3dMap.insert(make_pair(id, make_pair(vPoint, pWO)));
            }
        }
    }

    void GraphOptim::recoverPose(TaggedVertexPose &poseMap) {

        for(auto& poseEntry : poseMap) {

            g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(poseEntry.second.first);
            auto pFrame = poseEntry.second.second;

            g2o::SE3Quat SE3quat = vSE3->estimate();
            Eigen::Matrix4d PoseSE3 = Eigen::Matrix4d::Identity();
            PoseSE3.block<3, 3>(0, 0) = SE3quat.rotation().matrix();
            PoseSE3.block<3, 1>(0, 3) = SE3quat.translation();
            auto pLastPose = pFrame->getPose();
            pFrame->setPose(make_shared<TF::PoseSE3>(pLastPose->getRef(), pLastPose->getTarget(),
                                                     pLastPose->getTimestamp(), PoseSE3));
            pFrame->setOptId(-1);
        }
    }

    void GraphOptim::recoverMapPoints(TaggedVertexMP &point3dMap) {

        for(auto& p3dEntry : point3dMap) {

            g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(p3dEntry.second.first);
            auto pWO = p3dEntry.second.second;
            if (pWO) {
                auto pt3d = dynamic_pointer_cast<WO::Point3D>(pWO);
                Eigen::Vector3f point = vPoint->estimate().cast<float>();
                pt3d->setPoint(cv::Point3f(point.x(), point.y(), point.z()));
//            pMP->UpdateNormalAndDepth();
            }
        }
    }
} // NAV24::BE