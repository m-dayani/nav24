//
// Created by masoud on 2/11/24.
//

#ifndef NAV24_WORLDOBJECT_HPP
#define NAV24_WORLDOBJECT_HPP

#include <memory>
#include <set>
#include <vector>

#include "SmartObject.hpp"

namespace NAV24::OB {
    class Observation;
    typedef std::shared_ptr<Observation> ObsPtr;
    typedef std::weak_ptr<Observation> ObsPtrW;
}

namespace NAV24::WO {

    class WorldObject : public SmartObject {
    public:

        void addObservation(const OB::ObsPtr& pObs) { mvpObs.push_back(pObs); }
        std::vector<OB::ObsPtrW> getObservations() { return mvpObs; }

        virtual std::string toString() { return "World Object, Base Class\n"; }
    protected:
        std::vector<OB::ObsPtrW> mvpObs{};
    };
    typedef std::shared_ptr<WorldObject> WoPtr;
    typedef std::weak_ptr<WorldObject> WoPtrW;

} // NAV24::WO

#endif //NAV24_WORLDOBJECT_HPP
