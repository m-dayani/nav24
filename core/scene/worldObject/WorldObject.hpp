//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_WORLDOBJECT_HPP
#define NAV24_WORLDOBJECT_HPP

#include <memory>

#include "SmartObject.hpp"

namespace NAV24::OB {
    class Observation;
    typedef std::shared_ptr<Observation> obsPtr;
    typedef std::weak_ptr<Observation> obsPtrW;
}

namespace NAV24::WO {

    class WorldObject : public SmartObject {
    public:

        void setObservation(const OB::obsPtr& pObs) { mpObs = pObs; }
        OB::obsPtr getObservation() { return mpObs.lock(); }

        virtual std::string toString() { return "World Object, Base Class\n"; }
    protected:
        OB::obsPtrW mpObs{};
    };
    typedef std::shared_ptr<WorldObject> woPtr;
    typedef std::weak_ptr<WorldObject> woPtrW;

} // NAV24::WO

#endif //NAV24_WORLDOBJECT_HPP
