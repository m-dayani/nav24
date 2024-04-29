//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_OBSERVATION_HPP
#define NAV24_OBSERVATION_HPP

#include <memory>

namespace NAV24::OB {

    class Observation {
    public:

    };
    typedef std::shared_ptr<Observation> ObsPtr;
    typedef std::weak_ptr<Observation> ObsPtrW;

} // NAV24::OB

#endif //NAV24_OBSERVATION_HPP
