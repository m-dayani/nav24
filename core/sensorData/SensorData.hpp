//
// Created by masoud on 2/6/24.
//

#ifndef NAV24_SENSORDATA_HPP
#define NAV24_SENSORDATA_HPP

#include <string>
#include <utility>
#include <memory>


namespace NAV24 {

    class SensorData {
    public:
        SensorData() : ts(-1.0), path() {}
        SensorData(double ts_, std::string path_) : ts(ts_), path(std::move(path_)) {}

        virtual std::string toString();
    protected:
        double ts;
        std::string path;
    };
    typedef std::shared_ptr<SensorData> SensorDataPtr;

}   //NAV24

#endif //NAV24_SENSORDATA_HPP
