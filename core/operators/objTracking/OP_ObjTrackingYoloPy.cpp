//
// Created by masoud on 5/15/24.
//

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include <opencv2/opencv.hpp>
#ifdef LIB_PYTHON_FOUND
#include <numpy/arrayobject.h>
#endif
#include <boost/filesystem.hpp>
#include <glog/logging.h>

#include "OP_ObjTrackingYoloPy.hpp"


using namespace std;

namespace NAV24::OP {

    ObjTrYoloPy::ObjTrYoloPy(const ChannelPtr &pChannel) : ObjTrYoloOnnx(pChannel), mbFlagRun(false) {

    }

    ObjTrYoloPy::~ObjTrYoloPy() {
#ifdef LIB_PYTHON_FOUND
        if (mpObjClass) {
            Py_DECREF(mpObjClass);
            mpObjClass = nullptr;
        }
        if (mpObjMod) {
            Py_DECREF(mpObjMod);
            mpObjMod = nullptr;
        }
#endif
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
        if (mbFlagRun) {
            DLOG(WARNING) << "ObjTrYoloPy::run, the pipeline is already running, abort\n";
            return;
        }
        mbFlagRun = true;
#ifdef LIB_PYTHON_FOUND
        // NOTE: You must call all these methods from the same thread
        // That's why these are here instead of main()
        //Py_SetProgramName(Py_DecodeLocale("prog_name", nullptr)); // optional but strongly recommended
        Py_Initialize();

        if (!Py_IsInitialized()) {
            LOG(ERROR) << "Python API is not initialized, abort\n";
            return;
        }

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

        if (!mpObjClass) {
            return;
        }

        while (true) {

            FramePtr pImage;
            mMtxImgQ.lock();
            if (!mqpImages.empty()) {
                pImage = mqpImages.front();
                mqpImages.pop();
            }
            mMtxImgQ.unlock();

            double ts = -1.0;
            cv::Mat image;
            OB::ObsPtr pObs;
            fetchFrameInfo(pImage, ts, image, pObs);
            cv::Rect2d bbox = fetchBbox(pObs);

            if (!image.empty()) {
                if (!mbInitialized) {
                    auto pPyImage = convertImage(image);
                    npy_int dim = 4;
                    vector<double> array = {bbox.x, bbox.y, bbox.width, bbox.height};
                    auto pBboxArg = PyList_New(dim);
                    for (size_t i = 0; i != 4; ++i) {
                        PyList_SET_ITEM(pBboxArg, i, PyFloat_FromDouble(array[i]));
                    }

                    // segmentation fault if called twice, why??!
                    PyObject* res = PyObject_CallMethod(mpObjClass, "init", "OO", pPyImage, pBboxArg);
                    if (res) {
                        DLOG(INFO) << "ObjTrYoloPy::run:init received non-null response\n";
                        // parse return values
                        bool ret = false;
                        parseRetValue(res, ret, bbox);
                    }
//                    if (pArgs) Py_DECREF(pArgs);
                    if (pPyImage) Py_DECREF(pPyImage);
                    if (res) Py_DECREF(res);
                    if (pBboxArg) Py_DECREF(pBboxArg);
                    mbInitialized = true;
                }
                else {
                    PyObject* res = PyObject_CallMethod(mpObjClass, "update", "O", convertImage(image));
                    if (res) {
                        DLOG(INFO) << "ObjTrYoloPy::run:update received non-null response\n";
                        bool ret = false;
                        parseRetValue(res, ret, bbox);
                    }

                    if (res) Py_DECREF(res);
                }
            }

            if (this->isStopped()) {
                break;
            }
        }

        Py_Finalize();
#endif
    }

#ifdef LIB_PYTHON_FOUND
    void ObjTrYoloPy::parseRetValue(PyObject *pObj, bool &ret, cv::Rect2d &bbox) {

        PyObject* pRet, *pBbox;
        PyArg_ParseTuple(pObj, "OO", &pRet, &pBbox);
        if (pRet) {
            ret = PyObject_IsTrue(pRet);
        }
        if (pBbox) {
            // segmentation fault, why???
            int m_numCols = min((int)PyObject_Length(pBbox), 4);
            if (m_numCols >= 0) {
                vector<double> vBbox(m_numCols);
                std::cout << "[C++] the size of the array returned = " << m_numCols << std::endl;
                for (int j = 0; j < m_numCols; ++j) {
                    PyObject *piItem = PySequence_GetItem(pBbox, j);
                    if (piItem) {
                        double value = PyFloat_AsDouble(piItem);
                        vBbox[j] = value;
                        std::cout << "[C++] Predict[" << j << "] : " << value << std::endl;
                    }
                    if (piItem) Py_DECREF(piItem);
                }
                if (m_numCols >= 4) {
                    bbox = cv::Rect2d(vBbox[0], vBbox[1], vBbox[2], vBbox[3]);
                }
                else if (m_numCols >= 2) {
                    bbox = cv::Rect2d(vBbox[0], vBbox[1], 0, 0);
                }
            }
        }
        if (pRet) Py_DECREF(pRet);
        if (pBbox) Py_DECREF(pBbox);
    }
#endif

    void ObjTrYoloPy::init(const MsgPtr &msg) {
        //ObjTracking::init(msg);

    }

    void ObjTrYoloPy::setup(const MsgPtr &msg) {
        ObjTrYoloOnnx::setup(msg);

    }

    void ObjTrYoloPy::update(const FramePtr &pImage) {
        //ObjTrYoloOnnx::update(pImage);
#ifdef LIB_PYTHON_FOUND
        if (mpObjClass) {
//            auto res = PyObject_CallMethod(mpObjClass, "update", "(0)", convertImage(cv::Mat()));
//            if (res) {
//
//            }
        }
#endif
    }

#ifdef LIB_PYTHON_FOUND
    PyObject *ObjTrYoloPy::convertImage(const cv::Mat &image) {

        auto pArgs = PyTuple_New(1);
        import_array ()

        npy_intp dimensions[3] = {image.rows, image.cols, image.channels()};
        auto pValue = PyArray_SimpleNewFromData(image.dims + 1, (npy_intp*)&dimensions, NPY_UINT8, image.data);

        PyTuple_SetItem(pArgs, 0, pValue);

        return pArgs;
    }
#endif
} // NAV24::OP