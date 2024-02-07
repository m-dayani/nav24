//
// Created by masoud on 2/6/24.
//

#ifndef NAV24_CAMERA_HPP
#define NAV24_CAMERA_HPP

#include "Sensor.hpp"
#include "Calibration.hpp"


namespace NAV24 {



    class Camera : public Sensor {
    public:
        explicit Camera(const ChannelPtr& pChannel) : Sensor(pChannel) {}

        void receive(const MsgPtr &msg) override;

        std::string toString() { return printStr(""); }

    protected:
        void loadParams(const MsgPtr &msg) override;
        [[nodiscard]] std::string printStr(const std::string &prefix) const override;

    protected:
        int imWidth{}, imHeight{};
        float fps{};
        CalibPtr mpCalib;
    };

    class CamStream : public Camera {
    public:

    };

    class CamOffline : public Camera {
    public:

    protected:

    };

    class CamMixed : public CamStream, CamOffline {
    public:

    };

}   //NAV24

#endif //NAV24_CAMERA_HPP
