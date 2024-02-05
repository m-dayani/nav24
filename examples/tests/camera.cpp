//
// Created by root on 2/1/24.
//

#include <iostream>
#include <utility>
//#include <thread>
//#include <chrono>

#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>


using namespace std;
//using namespace NAV24;

class Camera {
public:
    explicit Camera(std::string img_path) : imgPath(std::move(img_path)) {}

    void play() {
        vector<cv::String> fn;
        cv::glob(imgPath, fn, false);

        for (const auto& imgFile : fn) {
            cv::Mat img = cv::imread(imgFile);
            cv::imshow("Image", img);
            cv::waitKey(33);
            //std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
        cv::destroyAllWindows();
    }

private:
    std::string imgPath;
};


int main([[maybe_unused]] int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    if (argc < 2) {
        cout << "Usage: camera img_path\n";
        return 1;
    }

    auto myCam = Camera(argv[1]);
    myCam.play();

    return 0;
}