//
// Created by masoud on 9/1/24.
//

#include <cmath>
#include <opencv2/core.hpp>

#include "FeatureGrid.hpp"
#include "Point2D.hpp"

using namespace std;

namespace NAV24::OB {

    bool FeatureGrid::mbInitImgBounds = false;
    int FeatureGrid::mGridRows = FRAME_GRID_ROWS, FeatureGrid::mGridCols = FRAME_GRID_COLS;
    float FeatureGrid::mnMinX, FeatureGrid::mnMinY, FeatureGrid::mnMaxX, FeatureGrid::mnMaxY;
    float FeatureGrid::mfGridElementWidthInv, FeatureGrid::mfGridElementHeightInv;

    FeatureGrid::FeatureGrid(vector<OB::ObsPtr> &vpObservations) :
            mGrid(), mN(vpObservations.size()), mvpObservations(vpObservations) {

        mGrid.resize(mGridCols);
        for (auto& gridCol : mGrid) {
            gridCol.resize(mGridRows);
        }
        this->assignFeaturesToGrid();
    }

    vector<size_t> FeatureGrid::getFeaturesInArea(const float &x, const float  &y, const float  &r,
                                                  const int minLevel, const int maxLevel) const {
        vector<size_t> vIndices;
        vIndices.reserve(mN);

        float factorX = r;
        float factorY = r;

        const int nMinCellX = max(0,(int)floor((x-mnMinX-factorX)*mfGridElementWidthInv));
        if(nMinCellX>=mGridCols)
        {
            return vIndices;
        }

        const int nMaxCellX = min((int)mGridCols-1,(int)ceil((x-mnMinX+factorX)*mfGridElementWidthInv));
        if(nMaxCellX<0)
        {
            return vIndices;
        }

        const int nMinCellY = max(0,(int)floor((y-mnMinY-factorY)*mfGridElementHeightInv));
        if(nMinCellY>=mGridRows)
        {
            return vIndices;
        }

        const int nMaxCellY = min((int)mGridRows-1,(int)ceil((y-mnMinY+factorY)*mfGridElementHeightInv));
        if(nMaxCellY<0)
        {
            return vIndices;
        }

        const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

        for(int ix = nMinCellX; ix<=nMaxCellX; ix++) {
            for(int iy = nMinCellY; iy<=nMaxCellY; iy++) {

                const vector<size_t> vCell = mGrid[ix][iy];
                if(vCell.empty())
                    continue;

                for(unsigned long j : vCell) {

                    auto pObs = mvpObservations[j];
                    if (dynamic_pointer_cast<OB::KeyPoint2D>(pObs)) {
                        auto pt2d = dynamic_pointer_cast<OB::KeyPoint2D>(pObs);
                        cv::KeyPoint kpUn = pt2d->getKeyPoint();
                        kpUn.pt = pt2d->getPointUd();

                        if(bCheckLevels) {
                            if(kpUn.octave<minLevel)
                                continue;
                            if(maxLevel>=0)
                                if(kpUn.octave>maxLevel)
                                    continue;
                        }

                        const float distx = kpUn.pt.x-x;
                        const float disty = kpUn.pt.y-y;

                        if(fabs(distx)<factorX && fabs(disty)<factorY)
                            vIndices.push_back(j);
                    }
                }
            }
        }

        return vIndices;
    }

    void FeatureGrid::setImageBounds(const cv::Size &imgSize, const vector<float> &bounds) {
        if (mbInitImgBounds) {
            return;
        }
        mGridCols = imgSize.width / FRAME_GRID_SIZE;
        mGridRows = imgSize.height / FRAME_GRID_SIZE;
        mnMinX = bounds[0];
        mnMaxX = bounds[1];
        mnMinY = bounds[2];
        mnMaxY = bounds[3];
        mfGridElementWidthInv=static_cast<float>(mGridCols)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(mGridRows)/(mnMaxY-mnMinY);
        mbInitImgBounds = true;
    }

    void FeatureGrid::assignFeaturesToGrid() {

        // Fill matrix with points
        const int nCells = mGridCols * mGridRows;

        int nReserve = static_cast<int>(0.5f * ((float)mN) / ((float)nCells));

        for(unsigned int i=0; i<mGridCols;i++)
            for (unsigned int j=0; j<mGridRows;j++) {
                mGrid[i][j].reserve(nReserve);
            }

        for(int i=0;i<mN;i++) {

            cv::KeyPoint kp;
            auto pObs = mvpObservations[i];
            if (dynamic_pointer_cast<OB::Point2D>(pObs)) {
                kp.pt = dynamic_pointer_cast<OB::Point2D>(pObs)->getPointUd();
            }

            int nGridPosX, nGridPosY;
            if(posInGrid(kp,nGridPosX,nGridPosY)){
                mGrid[nGridPosX][nGridPosY].push_back(i);
            }
        }
    }

    bool FeatureGrid::posInGrid(const cv::KeyPoint &kp, int &posX, int &posY) {

        posX = (int) round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
        posY = (int) round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if(posX<0 || posX>=mGridCols || posY<0 || posY>=mGridRows)
            return false;

        return true;
    }
}