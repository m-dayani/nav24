# The Navigator (NAV24)

General state estimation, navigation, SLAM, and more


## Dependencies

### Required

- OpenCV >= 4.5.1
- Eigen3
- Boost
- Google Glog

### Optional

- G2O
- DBoW2
- Pangolin
- Serial
- Python (C++)
- ONNX Runtime

## Issues

Tested on newer versions of Ubuntu (20.04 LTS and above) and Raspian Bullseye.

- Remember to compile and install `g2o` and `DBoW2` libraries separately.
- Cross-compiling for ARM (Raspian OS Bullseye lite aarch64):
  - The default OpenCV installation (version 4.5.1) does not include the 
"opencv2/video/detail/tracking.private.hpp" header: Error no such file or directory. To resolve the issue, copy `/usr/include/opencv4/opencv2/video/detail` from a working installation of OpenCV to the corresponding path and change the **private** in declared header in `/usr/include/opencv4/opencv2/tracking/tracking_internals.hpp` to **detail**
  - Cannot load `libg2o_opengl_helper.so.0.1`: `$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/aarch64-linux-gnu/`


