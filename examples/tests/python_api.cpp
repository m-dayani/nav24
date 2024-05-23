//
// Created by masoud on 5/14/24.
//

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include <Python.h>
#include <iostream>
#include <random>
#include <opencv2/opencv.hpp>
#include <numpy/arrayobject.h>

using namespace std;

class CPyInstance {
public:
    CPyInstance() : _wargv(nullptr) { Py_Initialize(); }
    CPyInstance(int argc, char* argv[]) {
        _wargv = new wchar_t*[argc];
        for (int i = 0; i < argc; ++i) {
            wchar_t* arg = Py_DecodeLocale(argv[i], nullptr);
            _wargv[i] = arg;
        }
        Py_SetProgramName(_wargv[0]); // optional but strongly recommended
        Py_Initialize();
    }
    ~CPyInstance() {
        if(_wargv) {
            delete[] _wargv;
            _wargv = nullptr;
        }
        Py_Finalize();
    }

    [[nodiscard]] wchar_t** argv() const { return _wargv;}
private:
    wchar_t** _wargv;
};

class CPyObject {
public:
    CPyObject() : _obj(nullptr) {}
    CPyObject(PyObject* obj) : _obj(obj) {}

    ~CPyObject() { Release(); }

    void setObject(PyObject* obj) noexcept { _obj = obj; }
    [[nodiscard]] PyObject* getObject() const noexcept { return _obj; }

    [[nodiscard]] bool is() const noexcept { return _obj != nullptr; }
    operator bool() const noexcept { return _obj != nullptr; }

    PyObject* AddRef() {
        if(_obj) Py_INCREF(_obj);
        return _obj;
    }

    void Release() {
        if(_obj) Py_DECREF(_obj);
        _obj = nullptr;
    }

    inline PyObject* operator ->() const noexcept { return _obj; }
    inline operator PyObject* () const noexcept { return _obj; }

    PyObject* operator = (PyObject* obj) {
        _obj = obj;
        return _obj;
    }

private:
    PyObject* _obj;
};

void testMyClass(int i_size, int *inputs) {

    CPyObject mydef = PyImport_ImportModule("ex_class");

    if (mydef) {
        CPyObject myClass = PyObject_GetAttrString(mydef, "MyClass");
        CPyObject instance = PyObject_CallObject(myClass, nullptr);

        if (instance) {
            CPyObject returnSum = PyObject_CallMethod(instance, "Sum", nullptr);
            if (returnSum) {
                //double result = PyFloat_AS_DOUBLE(returnSum.getObject()); //find using Search . Type this "PyFloat_"
                double result = PyFloat_AS_DOUBLE(
                        (PyObject *) returnSum); //find using Search . Type this "PyFloat_"
                std::cout << "[C++] sum = " << result << std::endl;
            } else std::cout << "[C++] No Sum Method or Error" << std::endl;

            CPyObject returnAvr = PyObject_CallMethod(instance, "Avr", "i", 4);
            if (returnAvr) {
                double result = PyFloat_AS_DOUBLE(
                        (PyObject *) returnAvr); //find using Search . Type this "PyFloat_"
                std::cout << "[C++] Avr = " << result << std::endl;
            } else std::cout << "[C++] No Avr Method or Error" << std::endl;

            CPyObject nParam = PyTuple_New(i_size);
            for (int i = 0; i < i_size; ++i) {
                PyObject *item = Py_BuildValue("i", inputs[i]);
                PyTuple_SetItem(nParam, i, item);
            }

            if (nParam) {
                CPyObject returnPredict = PyObject_CallMethod(instance, "Predict", "(O)", (PyObject *) nParam);
                if (returnPredict) {
                    int m_numCols = PyObject_Length(returnPredict);
                    std::cout << "[C++] the size of the array returned = " << m_numCols << std::endl;
                    for (int j = 0; j < m_numCols; ++j) {
                        CPyObject piItem = PySequence_GetItem(returnPredict, j);
                        if (piItem) {
                            double value = PyFloat_AS_DOUBLE(piItem.getObject());
                            std::cout << "[C++] Predict[" << j << "] : " << value << std::endl;
                        }
                    }
                } else std::cout << "[C++] No Predict Method or Error" << std::endl;
            } else std::cout << "[C++] No nParam or Error" << std::endl;
        }
    }
}

int testImage() {

    cv::Mat image = cv::imread("test.jpg", cv::IMREAD_UNCHANGED);

    PyObject *pName, *pModule, *pDict, *pFunc, *pArgs, *pValue;

    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append(\".\")");

//    PyObject* mydef = PyImport_ImportModule("ex_python_funcs");
    pName = PyUnicode_FromString("prog");
    if (pName == nullptr)
    {
        PyErr_Print();
        return 0;
    }

    pModule = PyImport_Import(pName);
    if (pModule == nullptr)
    {
        PyErr_Print();
        return 0;
    }
    pDict = PyModule_GetDict(pModule);
    pFunc = PyDict_GetItemString(pDict, "process");
    if (pFunc == nullptr)
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

    if(pResult == nullptr) {
        cout << "Calling the process method failed" << endl;
        return 0;
    }

    std::string m_gettextFunction = std::string(PyUnicode_AsUTF8(pResult));
    cout << "Result = " << m_gettextFunction << endl;

    return 1;
}

void test(int argc, char* argv[]) {

    CPyInstance pInstance(argc, argv);

    std::wcout << "GetProgramName: " << Py_GetProgramName() << std::endl;

    if (Py_IsInitialized()) {
        PySys_SetArgv(argc, pInstance.argv());
        std::wcout << "GetPath: " << Py_GetPath() << std::endl;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(0, 99);

        const int i_size = 4;
        int inputs[i_size];
        for (int & input : inputs) {
            input = dis(gen);
        }

        testMyClass(i_size, inputs);

        testImage();
    }
}


int main(int argc, char *argv[])
{
    // WARNING: Never call Py_Initialize and Py_Finalize twice!
    test(argc, argv);

    return 0;
}

