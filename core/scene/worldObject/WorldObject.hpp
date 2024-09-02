//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_WORLDOBJECT_HPP
#define NAV24_WORLDOBJECT_HPP

#include <memory>

#include "SmartObject.hpp"

namespace NAV24::OB {
    class Observation;
    typedef std::shared_ptr<Observation> ObsPtr;
    typedef std::weak_ptr<Observation> ObsPtrW;
}

namespace NAV24::WO {

    class WorldObject : public SmartObject {
    public:

        void setObservation(const OB::ObsPtr& pObs) { mpObs = pObs; }
        OB::ObsPtr getObservation() { return mpObs.lock(); }

        virtual std::string toString() { return "World Object, Base Class\n"; }
    protected:
        OB::ObsPtrW mpObs{};
    };
    typedef std::shared_ptr<WorldObject> WoPtr;
    typedef std::weak_ptr<WorldObject> WoPtrW;

} // NAV24::WO

#endif //NAV24_WORLDOBJECT_HPP
