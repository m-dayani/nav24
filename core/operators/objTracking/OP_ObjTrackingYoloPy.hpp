//
// Created by masoud on 5/15/24.
//

#ifndef NAV24_OP_OBJTRACKINGYOLOPY_HPP
#define NAV24_OP_OBJTRACKINGYOLOPY_HPP

//#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include <Python.h>

#include "OP_ObjTrackingYolo.hpp"

namespace NAV24::OP {

    class ObjTrYoloPy : public ObjTrYoloOnnx {
    public:
        explicit ObjTrYoloPy(const ChannelPtr& pChannel);
        ~ObjTrYoloPy();

        void receive(const MsgPtr &msg) override;

    protected:
        void run() override;

        void init(const MsgPtr &msg) override;

        void setup(const MsgPtr &msg) override;

        void update(const ImagePtr &pImage) override;

        static PyObject* convertImage(const cv::Mat& image);

    protected:
        // this tracker requires:
        // model weights, python tracking class, other init params (like confidence)
        std::string mPyFile;

        // Python Module Object
        PyObject* mpObjMod{};
        // Python Class Object
        PyObject* mpObjClass{};
    };
} // NAV24::OP

#endif //NAV24_OP_OBJTRACKINGYOLOPY_HPP
