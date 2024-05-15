//
// Created by masoud on 5/14/24.
//

#include <Python.h>
#include <numpy/arrayobject.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

using namespace cv;
using namespace std;

int main(int argc, char *argv[])
{
    //QCoreApplication a(argc, argv);

    Mat image = imread("test.jpg", IMREAD_UNCHANGED);

    Py_Initialize();
    PyObject *pName, *pModule, *pDict, *pFunc, *pArgs, *pValue;

    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append(\".\")");

//    PyObject* mydef = PyImport_ImportModule("ex_python_funcs");
    pName = PyUnicode_FromString("prog");
    if (pName == NULL)
    {
        PyErr_Print();
        return 0;
    }

    pModule = PyImport_Import(pName);
    if (pModule == NULL)
    {
        PyErr_Print();
        return 0;
    }
    pDict = PyModule_GetDict(pModule);
    pFunc = PyDict_GetItemString(pDict, "process");
    if (pFunc == NULL)
    {
        PyErr_Print();
        return 0;
    }

    pArgs = PyTuple_New(1);
    import_array ();

    npy_intp dimensions[3] = {image.rows, image.cols, image.channels()};
    pValue = PyArray_SimpleNewFromData(image.dims + 1, (npy_intp*)&dimensions, NPY_UINT8, image.data);

    PyTuple_SetItem(pArgs, 0, pValue);
    PyObject* pResult = PyObject_CallObject(pFunc, pArgs);

    if(pResult == NULL)
        cout<<"Calling the process method failed"<<endl;

    std::string m_gettextFunction = std::string(PyUnicode_AsUTF8(pResult));
    cout<< "Result = " << m_gettextFunction << endl;

    Py_Finalize();
    return 0;
}

