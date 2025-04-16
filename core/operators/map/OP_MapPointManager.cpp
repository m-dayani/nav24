//
// Created by masoud on 2/24/25.
//

#include "OP_MapPointManager.hpp"


namespace NAV24::OP {


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

    void MapPointManager::createMapPoints() {

        // Retrieve neighbor keyframes in covisibility graph

        // Retrieve current frame's intrinsic/extrinsic params

        // Loop through found key frames:

            // Check based line for close frames

            // Check ratio of baseline to median scene depth in monocular case

            // Find new matches (descriptor-based in ORB-SLAM)

            // Retrieve second frame's intrinsic/extrinsic params

            // Loop through found matches:

                // Check parallax between rays

                // Check triangulation in front of cameras

                // Check reprojection error in first keyframe

                // Check reprojection error in second keyframe

                // Check scale consistency

                // Triangulation is successful

        // ORB-SLAM has lots of options for other camera configurations (Stereo)
    }

    void MapPointManager::mapPointCulling() {

        // Loop through recently added map points

        // Remove bad map points heuristically:

        // Number of observations

        // Number of tracked (key) frames

        // Invalid optimization (large errors, ...)

    }
} // NAV24::OP