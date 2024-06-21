//
// Created by masoud on 2/6/24.
//

#include "SensorData.hpp"

#include <sstream>


using namespace std;

namespace NAV24 {

    std::string SensorData::toString() {
        ostringstream oss;
        oss << "ts: " << ts << ", " << "path: " << path;
        return oss.str();
    }

}   // NAV24