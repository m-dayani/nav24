//
// Created by masoud on 5/15/24.
//

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include <opencv2/opencv.hpp>
#include <numpy/arrayobject.h>
#include <boost/filesystem.hpp>
#include <glog/logging.h>

#include "OP_ObjTrackingYoloPy.hpp"


using namespace std;

namespace NAV24::OP {

    ObjTrYoloPy::ObjTrYoloPy(const ChannelPtr &pChannel) : ObjTrYoloOnnx(pChannel) {

    }

    ObjTrYoloPy::~ObjTrYoloPy() {

        if (mpObjClass) {
            Py_DECREF(mpObjClass);
            mpObjClass = nullptr;
        }
        if (mpObjMod) {
            Py_DECREF(mpObjMod);
            mpObjMod = nullptr;
        }
    }

    void ObjTrYoloPy::receive(const MsgPtr &msg) {
        ObjTrYoloOnnx::receive(msg);

        if (msg) {
            if (dynamic_pointer_cast<MsgConfig>(msg)) {
                auto pParam = dynamic_pointer_cast<MsgConfig>(msg)->getConfig();
                if (pParam) {
                    // Retrieve python script path
                    auto pParamPyPath = find_param<ParamType<string>>("script", pParam);
                    mPyFile = (pParamPyPath) ? pParamPyPath->getValue() : "";

                    if (!mPyFile.empty()) {
                        this->setup(msg);
                    }
                }
            }
        }
    }

    void ObjTrYoloPy::run() {
        //ObjTracking::run();

        while (true) {

            ImagePtr pImage;
            mMtxImgQ.lock();
            if (!mqpImages.empty()) {
                pImage = mqpImages.front();
                mqpImages.pop();
            }
            mMtxImgQ.unlock();

            if (pImage && mpObjClass) {
                cv::Mat image = pImage->mImage.clone();
                if (!mbInitialized) {
                    auto pArgs = PyTuple_New(2);
                    auto pPyImage = convertImage(image);
                    if (pPyImage) {
                        PyTuple_SetItem(pArgs, 0, PyTuple_GetItem(pPyImage, 0));
                    }
                    npy_int dim = 4;
                    vector<double> array = {bbox.x, bbox.y, bbox.width, bbox.height};
                    PyObject *pBbox = PyList_New(dim);
                    for (size_t i = 0; i != 4; ++i) {
                        PyList_SET_ITEM(pBbox, i, PyFloat_FromDouble(array[i]));
                    }
                    PyTuple_SetItem(pArgs, 1, pBbox);
                    auto res = PyObject_CallMethod(mpObjClass, "init", "(O)", pArgs);
                    if (res) {
                        DLOG(INFO) << "ObjTrYoloPy::run:init received non-null response\n";
                    }
                    mbInitialized = true;

                    Py_DECREF(pArgs);
                    Py_DECREF(pPyImage);
                    Py_DECREF(res);
                }
                else {
                    auto res = PyObject_CallMethod(mpObjClass, "update", "(O)", convertImage(image));
                    if (res) {
                        DLOG(INFO) << "ObjTrYoloPy::run:update received non-null response\n";
                    }

                    Py_DECREF(res);
                }
            }

            if (this->isStopped()) {
                break;
            }
        }
    }

    void ObjTrYoloPy::init(const MsgPtr &msg) {
        //ObjTracking::init(msg);


    }

    void ObjTrYoloPy::setup(const MsgPtr &msg) {
        ObjTrYoloOnnx::setup(msg);

        if (mPyFile.empty()) {
            DLOG(WARNING) << "ObjTrYoloPy::setup, received msg with empty file path\n";
            return;
        }

        // Also add the current path for dependency resolution
        auto bPath = boost::filesystem::path(mPyFile);
        string base = bPath.parent_path().string();
        PyRun_SimpleString("import sys");
        PyRun_SimpleString(("sys.path.append(\"" + base + "\")").c_str());

        // module path must be plain file name without the .py extension
        string fileName = basename(bPath);
        mpObjMod = PyImport_ImportModule(fileName.c_str());
        if (mpObjMod) {
            auto pObjClass = PyObject_GetAttrString(mpObjMod, "TrackerYOLO");

            // Path model as argument
            auto pArgs = PyTuple_New(1);
            auto pValue = PyUnicode_FromString(mModelPath.c_str());
            PyTuple_SetItem(pArgs, 0, pValue);
            mpObjClass = PyObject_CallObject(pObjClass, pArgs);

            if (mpObjClass) {
                DLOG(INFO) << "ObjTrYoloPy::init, instantiated PyClass successfully\n";
            }

            Py_DECREF(pValue);
            Py_DECREF(pArgs);
            Py_DECREF(pObjClass);
        }
    }

    void ObjTrYoloPy::update(const ImagePtr &pImage) {
        //ObjTrYoloOnnx::update(pImage);

        if (mpObjClass) {
            auto res = PyObject_CallMethod(mpObjClass, "update", "(0)", convertImage(cv::Mat()));
            if (res) {

            }
        }
    }

    PyObject *ObjTrYoloPy::convertImage(const cv::Mat &image) {

        auto pArgs = PyTuple_New(1);
        import_array ()

        npy_intp dimensions[3] = {image.rows, image.cols, image.channels()};
        auto pValue = PyArray_SimpleNewFromData(image.dims + 1, (npy_intp*)&dimensions, NPY_UINT8, image.data);

        PyTuple_SetItem(pArgs, 0, pValue);

        return pArgs;
    }
} // NAV24::OP