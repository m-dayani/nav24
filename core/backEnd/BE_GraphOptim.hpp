//
// Created by masoud on 9/3/24.
//

#ifndef NAV24_BE_GRAPHOPTIM_HPP
#define NAV24_BE_GRAPHOPTIM_HPP

#include "../thirdparty/g2o/g2o/core/optimizable_graph.h"
#include "../thirdparty/g2o/g2o/core/sparse_optimizer.h"

#include "BackEnd.hpp"


namespace NAV24::BE {

    typedef std::map<unsigned long, std::pair<g2o::OptimizableGraph::Vertex*, FramePtr>> TaggedVertexPose;
    typedef std::map<unsigned long, std::pair<g2o::OptimizableGraph::Vertex*, WO::WoPtr>> TaggedVertexMP;

    class GraphOptim : public BackEnd {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        void solve(const ProblemPtr &problem) override;

    protected:
        static void addFrameVertices(const std::vector<FramePtr>& vpFrames, unsigned long lastId,
                                     g2o::SparseOptimizer& optimizer, TaggedVertexPose& poseMap);
        static void extractMapPoints(const std::vector<FramePtr>& vpFrames, std::set<WO::WoPtr>& spPoint3d);
        static void addMapPointVertices(const ProblemPtr &problem, unsigned long lastId,
                                        g2o::SparseOptimizer& optimizer, TaggedVertexMP& point3dMap);

        static void recoverPose(TaggedVertexPose& poseMap);
        static void recoverMapPoints(TaggedVertexMP& point3dMap);
    };
} // NAV24::BE

#endif //NAV24_BE_GRAPHOPTIM_HPP
