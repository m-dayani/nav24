//
// Created by root on 5/16/21.
//

#ifndef NAV24_IMAGEDS_H
#define NAV24_IMAGEDS_H

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include <boost/filesystem.hpp>
#include <boost/any.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <glog/logging.h>

#include "Image.hpp"
#include "TabularTextDS.hpp"


namespace NAV24 {

    typedef std::pair<double, std::string> ImageInfo;

    class ImageDS : public TabularTextDS {
    public:
        ImageDS() = default;
        ImageDS(const std::string &filePath, const std::string &imBase, double tsFactor);
        ~ImageDS() override;

        std::vector<ImageInfo> getImageData() { return mvImageData; }
        void getImage(unsigned idx, cv::Mat &image, double &ts);
        void getImage(unsigned idx, cv::Mat &image, double &ts, std::string& imPath);

        unsigned getNumFiles() { return this->mvImageData.size(); }
        std::string getFileName(size_t idx, bool fullPath = true);
        double getTimeStamp(size_t idx);

        void reset() override;

    protected:
        boost::any parseLine(const std::string &evStr) override;
    private:

        std::string mImagesBasePath;
        // TODO: Use a map for timestamped data instead of vector
        std::vector<ImageInfo> mvImageData;
    };

    typedef std::shared_ptr<ImageDS> ImageDS_Ptr;
    typedef std::unique_ptr<ImageDS> ImageDS_UPtr;

} // NAV24

#endif //NAV24_IMAGEDS_H
