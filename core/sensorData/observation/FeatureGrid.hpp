//
// Created by masoud on 9/1/24.
//

#ifndef NAV24_FEATUREGRID_HPP
#define NAV24_FEATUREGRID_HPP

#include <vector>

#include "Observation.hpp"
#include "Calibration.hpp"


namespace NAV24::OB {

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64
#define FRAME_GRID_SIZE 10

    class FeatureGrid {
    public:
        explicit FeatureGrid(std::vector<ObsPtr> &mvpObservations);

        FeatureGrid(const FeatureGrid& ftGrid) = delete;
        FeatureGrid& operator=(const FeatureGrid& ftGrid) = delete;

        [[nodiscard]] std::vector<std::size_t> getFeaturesInArea(const float &x, const float  &y, const float  &r,
                                                                 int minLevel, int maxLevel) const;
        static void setImageBounds(const cv::Size &imgSize, const std::vector<float>& bounds);

    protected:
        void assignFeaturesToGrid();
        static bool posInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    protected:
        static bool mbInitImgBounds;
        static float mnMinX;
        static float mnMaxX;
        static float mnMinY;
        static float mnMaxY;

        static float mfGridElementWidthInv;
        static float mfGridElementHeightInv;

        static int mGridCols;
        static int mGridRows;

        std::size_t mN;
        const std::vector<OB::ObsPtr>& mvpObservations;
        std::vector<std::vector<std::vector<std::size_t>>> mGrid;
    };
}


#endif //NAV24_FEATUREGRID_HPP
